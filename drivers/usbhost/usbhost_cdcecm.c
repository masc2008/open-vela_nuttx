/****************************************************************************
 * drivers/usbhost/usbhost_cdcecm.c
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <semaphore.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>
#include <poll.h>
#include <fcntl.h>

#include <arpa/inet.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/kmalloc.h>
#include <nuttx/kthread.h>
#include <nuttx/mutex.h>
#include <nuttx/fs/fs.h>
#include <nuttx/wqueue.h>
#include <nuttx/signal.h>
#include <nuttx/net/ip.h>
#include <nuttx/net/netdev.h>

#include <nuttx/usb/cdc.h>
#include <nuttx/usb/usb.h>
#include <nuttx/usb/usbhost.h>

#ifdef CONFIG_USBHOST_CDCECM

#define CDCECM_NETBUF_SIZE  2048

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#ifndef CONFIG_SCHED_WORKQUEUE
#  warning "Worker thread support is required (CONFIG_SCHED_WORKQUEUE)"
#endif

#ifndef CONFIG_USBHOST_ASYNCH
#  warning Asynchronous transfer support is required (CONFIG_USBHOST_ASYNCH)
#endif

#ifdef CONFIG_USBHOST_CDCECM_NTDELAY
#  define USBHOST_CDCECM_NTDELAY MSEC2TICK(CONFIG_USBHOST_CDCECM_NTDELAY)
#else
#  define USBHOST_CDCECM_NTDELAY MSEC2TICK(200)
#endif


/* Driver support ***********************************************************/

/* This format is used to construct the /dev/cdc-ecm[n] device driver path.
 * It defined here so that it will be used consistently in all places.
 */

#define DEV_FORMAT          "/dev/cdc-ecm%d"
#define DEV_NAMELEN         16

/* Used in usbhost_cfgdesc() */

#define USBHOST_CTRLIFFOUND 0x01
#define USBHOST_DATAIFFOUND 0x02
#define USBHOST_INTRIFFOUND 0x04
#define USBHOST_BINFOUND    0x08
#define USBHOST_BOUTFOUND   0x10
#define USBHOST_ALLFOUND    0x1f

/****************************************************************************
 * Private Types
 ****************************************************************************/
/* This is a helper pointer for accessing the contents of Ethernet header */

#define BUF ((FAR struct eth_hdr_s *)priv->netdev.d_buf)

#define IS_DIG(n) \
    (((n) >= '0') && (n) <= '9')

  #define IS_UPER(n) \
    (((n) >= 'A' && (n) <= 'Z'))

  #define CHAR2HEX(c) \
    (IS_DIG(c) ? c - '0' : \
      (IS_UPER(c) ? c - 'A' + 10 : c - 'a' + 10))

struct usb_csifdesc_s
{
  uint8_t len;
  uint8_t type;
  uint8_t subtype;
};

struct usb_ecm_desc_s
{
  uint8_t len;
  uint8_t type;
  uint8_t subtype;
  uint8_t addr;
  uint8_t eth_statistics[4];
  uint8_t max_segment_size[2];
  uint8_t num_mcf_filters[2];
  uint8_t num_power_filters;
};

 /* This structure contains the internal, private state of the USB host
 * CDC/ECM class.
 */

struct usbhost_cdcecm_s
{
  /* This is the externally visible portion of the state.  The usbclass must
   * the first element of the structure.  It is then cast compatible with
   * struct usbhost_cdcecm_s.
   */

  struct usbhost_class_s usbclass;

  /* The remainder of the fields are provide to the CDC/ECM class */

  uint8_t             minor;          /* Minor number identifying the /dev/cdc-ecm[n] device */
  volatile bool       disconnected;   /* TRUE: Device has been disconnected */
  uint16_t            ctrlif;         /* Control interface number */
  uint16_t            dataif;         /* Data interface number */
  int16_t             crefs;          /* Reference count on the driver instance */
  mutex_t             lock;           /* Used to maintain mutual exclusive access */
  struct work_s       ntwork;         /* Notification work */
  struct work_s       bulk_rxwork;
  struct work_s       txpollwork;
  struct work_s       destroywork;
  int16_t             nnbytes;        /* Number of bytes received in notification */
  int16_t             bulkinbytes;
  uint16_t            maxintsize;     /* Maximum size of interrupt IN packet */
  uint16_t            maxsegmentsize; /* Maximum size of ecm segment size */
  FAR uint8_t         *ctrlreq;       /* Allocated ctrl request structure */
  FAR uint8_t         *ctrlbuf;       /* Allocated ctrl buf */
  FAR uint8_t         *notification;  /* Allocated RX buffer for notifications */
  FAR uint8_t         *rxnetbuf;      /* Allocated RX buffer for net frames */
  FAR uint8_t         *txnetbuf;      /* Allocated TX buffer for net frames */
  uint8_t             macindex;       /* String index of mac number */
  char                *macnum;        /* Allocated buffer for mac number */
  char                *macaddr;       /* Allocated buffer for mac addr */
  usbhost_ep_t        intin;          /* Interrupt endpoint */
  usbhost_ep_t        bulkin;         /* Bulk IN endpoint */
  usbhost_ep_t        bulkout;        /* Bulk OUT endpoint */
  uint16_t            bulkmxpacket;   /* Max packet size for Bulk OUT endpoint */

  /* Network device members */

  bool                bifup;          /* true:ifup false:ifdown */
  struct net_driver_s netdev;         /* Interface understood by the network */
  uint16_t            txpktbuf[(MAX_NETDEV_PKTSIZE + 1) / 2];
};

/* This structure contains the internal, private state of the USB host
 * CDC/ACM class.
 */

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

 /* Memory allocation services */

static inline FAR struct usbhost_cdcecm_s *usbhost_allocclass(void);
static inline void usbhost_freeclass(FAR struct usbhost_cdcecm_s *usbclass);

/* Device name management */

static int usbhost_allocdevno(FAR struct usbhost_cdcecm_s *priv);
static void usbhost_freedevno(FAR struct usbhost_cdcecm_s *priv);
static inline void usbhost_mkdevname(FAR struct usbhost_cdcecm_s *priv,
                                     FAR char *devname);

/* Worker thread actions */

static void usbhost_notification_work(FAR void *arg);
static void usbhost_notification_callback(FAR void *arg, ssize_t nbytes);
static void usbhost_bulkin_work(FAR void *arg);
static void usbhost_bulkin_callback(FAR void *arg, ssize_t nbytes);

/* Worker thread actions */
static void usbhost_destroy(FAR void *arg);

/* Helpers for usbhost_connect() */

static int usbhost_cfgdesc(FAR struct usbhost_cdcecm_s *priv,
                           FAR const uint8_t *configdesc, int desclen);
static inline int usbhost_devinit(FAR struct usbhost_cdcecm_s *priv);

/* (Little Endian) Data helpers */

static inline uint16_t usbhost_getle16(FAR const uint8_t *val);
static inline void usbhost_putle16(FAR uint8_t *dest, uint16_t val);
static inline uint32_t usbhost_getle32(FAR const FAR uint8_t *val);
static void usbhost_putle32(FAR uint8_t *dest, uint32_t val);

/* Buffer memory management */

static int usbhost_alloc_buffers(FAR struct usbhost_cdcecm_s *priv);
static void usbhost_free_buffers(FAR struct usbhost_cdcecm_s *priv);

/* struct usbhost_registry_s methods */

static struct usbhost_class_s *usbhost_create(
              FAR struct usbhost_hubport_s *hport,
              FAR const struct usbhost_id_s *id);

/* struct usbhost_class_s methods */

static int usbhost_connect(FAR struct usbhost_class_s *usbclass,
                           FAR const uint8_t *configdesc, int desclen);
static int usbhost_disconnected(FAR struct usbhost_class_s *usbclass);

/* NuttX network callback functions */

static int cdcecm_ifup(FAR struct net_driver_s *dev);
static int cdcecm_ifdown(FAR struct net_driver_s *dev);
static int cdcecm_txavail(FAR struct net_driver_s *dev);

/* Network support functions */

static void cdcecm_receive(FAR struct usbhost_cdcecm_s *priv,
                            FAR uint8_t *buf, size_t len);

static int cdcecm_txpoll(FAR struct net_driver_s *dev);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* This structure provides the registry entry ID information that will  be
 * used to associate the USB class driver to a connected USB device.
 */

static const struct usbhost_id_s g_id =
{
  USB_CLASS_CDC,      /* base */
  CDC_SUBCLASS_ECM,   /* subclass */
  0,                  /* proto */
  0,                  /* vid */
  0                   /* pid */
};

/* This structure provides the registry entry ID information that will  be
 * used to associate the USB class driver to a connected USB device.
 */

/* This is the USB host CDC/ECM class's registry entry */

static struct usbhost_registry_s g_cdcecm =
{
  NULL,                   /* flink */
  usbhost_create,         /* create */
  1,                      /* nids */
  &g_id                   /* id[] */
};

static const struct file_operations g_cdcecm_fops =
{
  NULL,          /* open */
  NULL,          /* close */
  NULL,          /* read */
  NULL,          /* write */
  NULL,          /* seek */
  NULL,          /* ioctl */
  NULL,          /* mmap */
  NULL,          /* truncate */
  NULL           /* poll */
};

/* This is a bitmap that is used to allocate device names /dev/cdc-ecm[n]. */

static uint32_t g_devinuse;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int usbhost_ctrl_cmd(FAR struct usbhost_cdcecm_s *priv,
                            uint8_t type, uint8_t req, uint16_t value,
                            uint16_t iface, FAR uint8_t *payload,
                            uint16_t len)
{
  FAR struct usbhost_hubport_s *hport;
  FAR struct usb_ctrlreq_s *ctrlreq;
  int ret;

  hport = priv->usbclass.hport;

  ctrlreq       = (FAR struct usb_ctrlreq_s *)priv->ctrlreq;
  ctrlreq->type = type;
  ctrlreq->req  = req;

  usbhost_putle16(ctrlreq->value, value);
  usbhost_putle16(ctrlreq->index, iface);
  usbhost_putle16(ctrlreq->len,   len);

  if (type & USB_REQ_DIR_IN)
    {
      ret = DRVR_CTRLIN(hport->drvr, hport->ep0, ctrlreq, payload);
    }
  else
    {
      ret = DRVR_CTRLOUT(hport->drvr, hport->ep0, ctrlreq, payload);
    }

  return ret;
}

/****************************************************************************
 * Name: usbhost_allocclass
 *
 * Description:
 *   This is really part of the logic that implements the create() method
 *   of struct usbhost_registry_s.  This function allocates memory for one
 *   new class instance.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   On success, this function will return a non-NULL instance of struct
 *   usbhost_class_s.  NULL is returned on failure; this function will
 *   will fail only if there are insufficient resources to create another
 *   USB host class instance.
 *
 ****************************************************************************/

static inline FAR struct usbhost_cdcecm_s *usbhost_allocclass(void)
{
  FAR struct usbhost_cdcecm_s *priv;

  DEBUGASSERT(!up_interrupt_context());
  priv = kmm_malloc(sizeof(struct usbhost_cdcecm_s));
  uinfo("ECM Allocated: %p\n", priv);
  return priv;
}

/****************************************************************************
 * Name: usbhost_freeclass
 *
 * Description:
 *   Free a class instance previously allocated by usbhost_allocclass().
 *
 * Input Parameters:
 *   usbclass - A reference to the class instance to be freed.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void usbhost_freeclass(FAR struct usbhost_cdcecm_s *usbclass)
{
  DEBUGASSERT(usbclass != NULL);

  /* Free the class instance (perhaps calling sched_kmm_free() in case we are
   * executing from an interrupt handler.
   */

  uinfo("ECM Freeing: %p\n", usbclass);
  kmm_free(usbclass);
}

/****************************************************************************
 * Name: Device name management
 *
 * Description:
 *   Some tiny functions to coordinate management of device names.
 *
 ****************************************************************************/

static int usbhost_allocdevno(FAR struct usbhost_cdcecm_s *priv)
{
  irqstate_t flags;
  int devno;

  flags = enter_critical_section();
  for (devno = 0; devno < 32; devno++)
    {
      uint32_t bitno = 1 << devno;
      if ((g_devinuse & bitno) == 0)
        {
          g_devinuse |= bitno;
          priv->minor = devno;
          leave_critical_section(flags);
          return OK;
        }
    }

  leave_critical_section(flags);
  return -EMFILE;
}

static void usbhost_freedevno(FAR struct usbhost_cdcecm_s *priv)
{
  int devno = priv->minor;

  if (devno >= 0 && devno < 26)
    {
      irqstate_t flags = enter_critical_section();
      g_devinuse &= ~(1 << devno);
      leave_critical_section(flags);
    }
}

static inline void usbhost_mkdevname(FAR struct usbhost_cdcecm_s *priv,
                                     FAR char *devname)
{
  snprintf(devname, DEV_NAMELEN, DEV_FORMAT, priv->minor);
}

static void usbhost_bulkin_callback(FAR void *arg, ssize_t nbytes)
{
  FAR struct usbhost_cdcecm_s *priv = (FAR struct usbhost_cdcecm_s *)arg;
  uint32_t delay = 0;

  DEBUGASSERT(priv);

  if (priv->disconnected)
    {
      return;
    }

  priv->bulkinbytes = (int16_t)nbytes;

  if (nbytes < 0)
    {
      if (nbytes != -EAGAIN)
        {
          uerr("ECM ERROR: Transfer failed: %d\n", nbytes);
        }

      delay = MSEC2TICK(30);
    }

  if (work_available(&priv->bulk_rxwork))
    {
      work_queue(LPWORK, &priv->bulk_rxwork,
                 usbhost_bulkin_work, priv, delay);
    }
}

static void usbhost_bulkin_work(FAR void *arg)
{
  FAR struct usbhost_cdcecm_s *priv;
  FAR struct usbhost_hubport_s *hport;

  priv = (FAR struct usbhost_cdcecm_s *)arg;
  DEBUGASSERT(priv);

  hport = priv->usbclass.hport;
  DEBUGASSERT(hport);

  if (priv->disconnected || !priv->bifup)
    {
      return;
    }

  nxmutex_lock(&priv->lock);

  uinfo("ECM bulkinbytes: %d\n", priv->bulkinbytes);

  if (priv->bulkinbytes > 0)
    {
      cdcecm_receive(priv, priv->rxnetbuf, priv->bulkinbytes);
    }

  DRVR_ASYNCH(hport->drvr, priv->bulkin,
                (FAR uint8_t *)priv->rxnetbuf, priv->maxsegmentsize,
                usbhost_bulkin_callback, priv);
  nxmutex_unlock(&priv->lock);
}

/****************************************************************************
 * Name: usbhost_cfgdesc
 *
 * Description:
 *   This function implements the connect() method of struct
 *   usbhost_class_s.  This method is a callback into the class
 *   implementation.  It is used to provide the device's configuration
 *   descriptor to the class so that the class may initialize properly
 *
 * Input Parameters:
 *   priv - The USB host class instance.
 *   configdesc - A pointer to a uint8_t buffer container the configuration
 *     descriptor.
 *   desclen - The length in bytes of the configuration descriptor.
 *
 * Returned Value:
 *   On success, zero (OK) is returned. On a failure, a negated errno value
 *   is returned indicating the nature of the failure
 *
 * Assumptions:
 *   This function will *not* be called from an interrupt handler.
 *
 ****************************************************************************/

static int usbhost_cfgdesc(FAR struct usbhost_cdcecm_s *priv,
                           FAR const uint8_t *configdesc, int desclen)
{
  FAR struct usbhost_hubport_s *hport;
  FAR struct usb_cfgdesc_s *cfgdesc;
  FAR struct usb_desc_s *desc;
  struct usbhost_epdesc_s bindesc;
  struct usbhost_epdesc_s boutdesc;
  struct usbhost_epdesc_s iindesc;
  int remaining;
  uint8_t found = 0;
  int ret;

  DEBUGASSERT(priv != NULL && priv->usbclass.hport &&
              configdesc != NULL && desclen >= sizeof(struct usb_cfgdesc_s));
  hport = priv->usbclass.hport;

  /* Verify that we were passed a configuration descriptor */

  cfgdesc = (FAR struct usb_cfgdesc_s *)configdesc;
  if (cfgdesc->type != USB_DESC_TYPE_CONFIG)
    {
      return -EINVAL;
    }

  /* Get the total length of the configuration descriptor (little endian).
   * It might be a good check to get the number of interfaces here too.
   */

  remaining = (int)usbhost_getle16(cfgdesc->totallen);

  /* Skip to the next entry descriptor */

  configdesc += cfgdesc->len;
  remaining  -= cfgdesc->len;

  /* Loop where there are more descriptors to examine */

  while (remaining >= sizeof(struct usb_desc_s))
    {
      /* What is the next descriptor? */

      desc = (FAR struct usb_desc_s *)configdesc;
      switch (desc->type)
        {
        case USB_DESC_TYPE_CSINTERFACE:
          {
            FAR struct usb_csifdesc_s *csdesc =
                (FAR struct usb_csifdesc_s *)desc;

            /* ECM functional descriptor */

            if (csdesc->subtype == CDC_DSUBTYPE_ECM)
              {
                FAR struct usb_ecm_desc_s *ecm =
                    (FAR struct usb_ecm_desc_s *)desc;

                priv->macindex = ecm->addr;
                priv->maxsegmentsize = usbhost_getle16(ecm->max_segment_size);
                uinfo("ECM max segment size: %u, macindex: %u\n",
                       priv->maxsegmentsize, priv->macindex);
              }
          }
          break;
        /* Interface descriptor. We really should get the number of endpoints
        * from this descriptor too.
        */

        case USB_DESC_TYPE_INTERFACE:
          {
            FAR struct usb_ifdesc_s *ifdesc =
                (FAR struct usb_ifdesc_s *)configdesc;

            uinfo("ECM Interface descriptor\n");
            DEBUGASSERT(remaining >= USB_SIZEOF_IFDESC);

            /* Is this the control interface? */
            if (ifdesc->classid  == CDC_CLASS_COMM &&
                ifdesc->subclass == CDC_SUBCLASS_ECM &&
                ifdesc->protocol == CDC_PROTO_NONE)
              {
                priv->ctrlif  = ifdesc->ifno;
                found        |= USBHOST_CTRLIFFOUND;
              }

            /* Is this the data interface? */

            else if (ifdesc->classid  == USB_CLASS_CDC_DATA &&
                     ifdesc->subclass == CDC_SUBCLASS_NONE &&
                     ifdesc->protocol == CDC_DATA_PROTO_NONE)
              {
                priv->dataif  = ifdesc->ifno;
                found        |= USBHOST_DATAIFFOUND;
              }
          }
          break;

          /* Endpoint descriptor.  Here, we expect two bulk endpoints, an IN
         * and an OUT, an interrupt in endpoint
         */

        case USB_DESC_TYPE_ENDPOINT:
          {
            FAR struct usb_epdesc_s *epdesc =
                (FAR struct usb_epdesc_s *)configdesc;

            uinfo("ECM Endpoint descriptor\n");
            DEBUGASSERT(remaining >= USB_SIZEOF_EPDESC);

            /* Check for interrupt endpoint */

            if ((epdesc->attr & USB_EP_ATTR_XFERTYPE_MASK) ==
                USB_EP_ATTR_XFER_INT)
              {
                if (USB_ISEPIN(epdesc->addr))
                  {
                    found |= USBHOST_INTRIFFOUND;
                    iindesc.hport        = hport;
                    iindesc.addr         = epdesc->addr &
                                           USB_EP_ADDR_NUMBER_MASK;
                    iindesc.in           = true;
                    iindesc.xfrtype      = USB_EP_ATTR_XFER_INT;
                    iindesc.interval     = epdesc->interval;
                    iindesc.mxpacketsize =
                                usbhost_getle16(epdesc->mxpacketsize);
                    uinfo("ECM Interrupt IN EP addr:%d mxpacketsize:%d\n",
                          iindesc.addr, iindesc.mxpacketsize);

                    priv->maxintsize = iindesc.mxpacketsize;
                  }
              }

            /* Check for a bulk endpoint. */

            else if ((epdesc->attr & USB_EP_ATTR_XFERTYPE_MASK) ==
                     USB_EP_ATTR_XFER_BULK)
              {
                /* Yes.. it is a bulk endpoint.  IN or OUT? */

                if (USB_ISEPOUT(epdesc->addr))
                  {
                    /* It is an OUT bulk endpoint.  There should be only one
                     * bulk OUT endpoint.
                     */

                    if ((found & USBHOST_BOUTFOUND) != 0)
                      {
                        /* Oops.. more than one endpoint.  We don't know
                         * what to do with this.
                         */

                        return -EINVAL;
                      }

                    found |= USBHOST_BOUTFOUND;

                    /* Save the bulk OUT endpoint information */

                    boutdesc.hport        = hport;
                    boutdesc.addr         = epdesc->addr &
                                            USB_EP_ADDR_NUMBER_MASK;
                    boutdesc.in           = false;
                    boutdesc.xfrtype      = USB_EP_ATTR_XFER_BULK;
                    boutdesc.interval     = epdesc->interval;
                    boutdesc.mxpacketsize =
                                usbhost_getle16(epdesc->mxpacketsize);
                    uinfo("ECM Bulk OUT EP addr:%d mxpacketsize:%d\n",
                          boutdesc.addr, boutdesc.mxpacketsize);

                    priv->bulkmxpacket = boutdesc.mxpacketsize;
                  }
                else
                  {
                    /* It is an IN bulk endpoint.  There should be only one
                     * bulk IN endpoint.
                     */

                    if ((found & USBHOST_BINFOUND) != 0)
                      {
                        /* Oops.. more than one endpoint.  We don't know
                         * what to do with this.
                         */

                        return -EINVAL;
                      }

                      found |= USBHOST_BINFOUND;

                    /* Save the bulk IN endpoint information */

                    bindesc.hport        = hport;
                    bindesc.addr         = epdesc->addr &
                                           USB_EP_ADDR_NUMBER_MASK;
                    bindesc.in           = 1;
                    bindesc.xfrtype      = USB_EP_ATTR_XFER_BULK;
                    bindesc.interval     = epdesc->interval;
                    bindesc.mxpacketsize =
                                usbhost_getle16(epdesc->mxpacketsize);
                    uinfo("ECM Bulk IN EP addr:%d mxpacketsize:%d\n",
                          bindesc.addr, bindesc.mxpacketsize);
                  }
              }
          }
          break;

        /* Other descriptors are just ignored for now */

        default:
          break;
        }

      /* If we found everything we need with this interface, then break out
       * of the loop early.
       */

      if (found == USBHOST_ALLFOUND)
        {
          break;
        }

      /* Increment the address of the next descriptor */

      configdesc += desc->len;
      remaining  -= desc->len;
    }

  /* Sanity checking... did we find all of things that we need? */

  if (found != USBHOST_ALLFOUND)
    {
      uerr("ECM ERROR: Found CTRLIF:%s DATAIF: %s BIN:%s BOUT:%s\n",
           (found & USBHOST_CTRLIFFOUND) != 0 ? "YES" : "NO",
           (found & USBHOST_DATAIFFOUND) != 0 ? "YES" : "NO",
           (found & USBHOST_BINFOUND) != 0 ? "YES" : "NO",
           (found & USBHOST_BOUTFOUND) != 0 ? "YES" : "NO");
      return -EINVAL;
    }

  /* We are good... Allocate the endpoints */

  ret = DRVR_EPALLOC(hport->drvr, &boutdesc, &priv->bulkout);
  if (ret < 0)
    {
      uerr("ECM ERROR: Failed to allocate Bulk OUT endpoint\n");
      return ret;
    }

  ret = DRVR_EPALLOC(hport->drvr, &bindesc, &priv->bulkin);
  if (ret < 0)
    {
      uerr("ECM ERROR: Failed to allocate Bulk IN endpoint\n");
      (void)DRVR_EPFREE(hport->drvr, priv->bulkout);
      return ret;
    }

  ret = DRVR_EPALLOC(hport->drvr, &iindesc, &priv->intin);
  if (ret < 0)
    {
      uerr("ECM ERROR: Failed to allocate Interrupt IN endpoint\n");
      (void)DRVR_EPFREE(hport->drvr, priv->bulkout);
      (void)DRVR_EPFREE(hport->drvr, priv->bulkin);
      return ret;
    }

  uinfo("ECM Endpoints allocated\n");
  return OK;
}

static int ecm_set_ethernet_packet_filter(FAR struct usbhost_cdcecm_s *priv,
                                uint16_t iface, uint16_t setting)
{
  return usbhost_ctrl_cmd(priv,
                          USB_REQ_DIR_OUT | USB_REQ_TYPE_CLASS | USB_REQ_RECIPIENT_INTERFACE,
                          ECM_SET_PACKET_FILTER, setting, iface, NULL, 0);
}

static int usbhost_setinterface(FAR struct usbhost_cdcecm_s *priv,
                                uint16_t iface, uint16_t setting)
{
  return usbhost_ctrl_cmd(priv,
                          USB_REQ_DIR_OUT | USB_REQ_RECIPIENT_INTERFACE,
                          USB_REQ_SETINTERFACE, setting, iface, NULL, 0);
}

static int usbhost_get_macnum(FAR struct usbhost_cdcecm_s *priv, uint8_t idx)
{
  uint16_t value = (uint16_t)USB_DESC_TYPE_STRING << 8 | (uint16_t)idx;
  uint16_t wIdex = 0x0409;
  return usbhost_ctrl_cmd(priv,
                          USB_REQ_DIR_IN,
                          USB_REQ_GETDESCRIPTOR, value, wIdex, priv->ctrlbuf, 0xff);
}

/****************************************************************************
 * Name: usbhost_devinit
 *
 * Description:
 *   The USB device has been successfully connected.  This completes the
 *   initialization operations.  It is first called after the
 *   configuration descriptor has been received.
 *
 *   This function is called from the connect() method.  This function always
 *   executes on the thread of the caller of connect().
 *
 * Input Parameters:
 *   priv - A reference to the class instance.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline int usbhost_devinit(FAR struct usbhost_cdcecm_s *priv)
{
  FAR struct usbhost_hubport_s *hport;
  char high_half = 0, low_half = 0;
  int ret = OK;

  hport = priv->usbclass.hport;

  /* Increment the reference count.  This will prevent usbhost_destroy() from
   * being called asynchronously if the device is removed.
   */

  priv->crefs++;
  DEBUGASSERT(priv->crefs == 2);

  /* Configure the device */

  /* Set aside transfer buffers for exclusive use by the class driver */
  ret = usbhost_alloc_buffers(priv);
  if (ret)
    {
      uerr("ECM ERROR: failed to allocate buffers\n");
      return ret;
    }

  /* Set alternate setting 1 on data interface */

  usbhost_setinterface(priv, priv->dataif, 1);

  /* Set Ethernet packet filter*/

  ret = ecm_set_ethernet_packet_filter(priv, priv->ctrlif, PACKET_TYPE_DIRECTED | PACKET_TYPE_BROADCAST);
  if (ret)
    {
      uerr("ECM ERROR: failed to set eth filter parameters: %d\n", ret);
    }

  /* Register the driver */

  if (ret >= 0)
    {
      char devname[DEV_NAMELEN];

      uinfo("ECM Register character driver\n");
      usbhost_mkdevname(priv, devname);
      ret = register_driver(devname, &g_cdcecm_fops, 0666, priv);
    }

  if (priv->intin)
    {
      /* Begin monitoring of message available events */

      uinfo("ECM Start notification monitoring\n");
      ret = DRVR_ASYNCH(hport->drvr, priv->intin,
                        (FAR uint8_t *)priv->notification,
                        SIZEOF_NOTIFICATION_S(0),
                        usbhost_notification_callback,
                        priv);
      if (ret < 0)
        {
          uerr("ECM ERROR: DRVR_ASYNCH failed on intin: %d\n", ret);
        }
    }

  /* Setup the network interface */

  memset(&priv->netdev, 0, sizeof(struct net_driver_s));
  priv->netdev.d_ifup    = cdcecm_ifup;
  priv->netdev.d_ifdown  = cdcecm_ifdown;
  priv->netdev.d_txavail = cdcecm_txavail;
  priv->netdev.d_private = priv;

  /* get the mac addr */

  usbhost_get_macnum(priv, priv->macindex);

  if (priv->ctrlbuf)
    {
      /* Begin get macnum from ctrlbuf */

      for (int index = 0; index < (IFHWADDRLEN * 2); index++)
        {
          priv->macnum[index] = priv->ctrlbuf[index * 2 + 2];
        }
    }

  /* Setup the mac addr */

  for (int index = 0; index < IFHWADDRLEN; index++)
    {
      high_half = CHAR2HEX(priv->macnum[index * 2]);
      low_half  = CHAR2HEX(priv->macnum[index * 2 + 1]);
      priv->macaddr[index] = high_half << 4 | low_half;

      uinfo("ECM macnum [0]: %x  [1]: %x", priv->macnum[index * 2], priv->macnum[index * 2 + 1]);
      uinfo("ECM high_half: %x low_half: %x", high_half, low_half);
      uinfo("ECM macaddr: %x", priv->macaddr[index]);
    }

  memcpy(priv->netdev.d_mac.ether.ether_addr_octet,
         priv->macaddr, IFHWADDRLEN);

  /* Register the network device */

  uinfo("ECM netdev: %p, IFHWADDRLEN: %d", &priv->netdev, IFHWADDRLEN);
  netdev_register(&priv->netdev, NET_LL_ETHERNET);

  /* Check if we successfully initialized. We now have to be concerned
   * about asynchronous modification of crefs because the character
   * driver has been registered.
   */

  if (ret >= 0)
    {
      nxmutex_lock(&priv->lock);
      DEBUGASSERT(priv->crefs >= 2);

      /* Handle a corner case where (1) open() has been called so the
       * reference count is > 2, but the device has been disconnected.
       * In this case, the class instance needs to persist until close()
       * is called.
       */

      if (priv->crefs <= 2 && priv->disconnected)
        {
          /* We don't have to give the semaphore because it will be
           * destroyed when usb_destroy is called.
           */

          ret = -ENODEV;
        }
      else
        {
          /* Ready for normal operation */

          syslog(0,"ECM Successfully initialized\n");
          priv->crefs--;
          nxmutex_unlock(&priv->lock);
        }
    }

  return ret;
}

/****************************************************************************
 * Name: usbhost_getle16
 *
 * Description:
 *   Get a (possibly unaligned) 16-bit little endian value.
 *
 * Input Parameters:
 *   val - A pointer to the first byte of the little endian value.
 *
 * Returned Value:
 *   A uint16_t representing the whole 16-bit integer value
 *
 ****************************************************************************/

static inline uint16_t usbhost_getle16(FAR const uint8_t *val)
{
  return (uint16_t)val[1] << 8 | (uint16_t)val[0];
}

/****************************************************************************
 * Name: usbhost_putle16
 *
 * Description:
 *   Put a (possibly unaligned) 16-bit little endian value.
 *
 * Input Parameters:
 *   dest - A pointer to the first byte to save the little endian value.
 *   val - The 16-bit value to be saved.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void usbhost_putle16(FAR uint8_t *dest, uint16_t val)
{
  dest[0] = val & 0xff; /* Little endian means LS byte first in byte stream */
  dest[1] = val >> 8;
}

/****************************************************************************
 * Name: usbhost_getle32
 *
 * Description:
 *   Get a (possibly unaligned) 32-bit little endian value.
 *
 * Input Parameters:
 *   dest - A pointer to the first byte to save the big endian value.
 *   val - The 32-bit value to be saved.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline uint32_t usbhost_getle32(FAR const uint8_t *val)
{
  /* Little endian means LS halfword first in byte stream */

  return (uint32_t)usbhost_getle16(&val[2]) << 16 |
         (uint32_t)usbhost_getle16(val);
}

/****************************************************************************
 * Name: usbhost_putle32
 *
 * Description:
 *   Put a (possibly unaligned) 32-bit little endian value.
 *
 * Input Parameters:
 *   dest - A pointer to the first byte to save the little endian value.
 *   val - The 32-bit value to be saved.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void usbhost_putle32(FAR uint8_t *dest, uint32_t val)
{
  /* Little endian means LS halfword first in byte stream */

  usbhost_putle16(dest, (uint16_t)(val & 0xffff));
  usbhost_putle16(dest + 2, (uint16_t)(val >> 16));
}

/****************************************************************************
 * Name: usbhost_alloc_buffers
 *
 * Description:
 *   Allocate transfer buffer memory.
 *
 * Input Parameters:
 *   priv - A reference to the class instance.
 *
 * Returned Value:
 *   On success, zero (OK) is returned.  On failure, an negated errno value
 *   is returned to indicate the nature of the failure.
 *
 ****************************************************************************/

 static int usbhost_alloc_buffers(FAR struct usbhost_cdcecm_s *priv)
{
  FAR struct usbhost_hubport_s *hport;
  size_t maxlen;
  int ret;

  DEBUGASSERT(priv != NULL && priv->usbclass.hport != NULL &&
              priv->ctrlreq == NULL);
  hport = priv->usbclass.hport;

  /* Allocate memory for control requests */

  ret = DRVR_ALLOC(hport->drvr, (FAR uint8_t **)&priv->ctrlreq, &maxlen);
  if (ret < 0)
    {
      uerr("ECM ERROR: DRVR_ALLOC of ctrlreq failed: %d\n", ret);
      goto errout;
    }

  DEBUGASSERT(maxlen >= sizeof(struct usb_ctrlreq_s));

  /* Allocate memory for control buf */

  ret = DRVR_IOALLOC(hport->drvr, &priv->ctrlbuf, IFHWADDRLEN * 5);
  if (ret < 0)
    {
      uerr("ECM ERROR: DRVR_IOALLOC of ctrlbuf failed: %d (%d bytes)\n",
           ret, IFHWADDRLEN * 5);
      goto errout;
    }

  /* Allocate buffer for interrupt IN notifications */

  ret = DRVR_IOALLOC(hport->drvr, &priv->notification, priv->maxintsize);
  if (ret < 0)
    {
      uerr("ECM ERROR: DRVR_IOALLOC of notification buf failed: %d (%d bytes)\n",
           ret, priv->maxintsize);
      goto errout;
    }

  ret = DRVR_IOALLOC(hport->drvr, &priv->rxnetbuf, CDCECM_NETBUF_SIZE);
  if (ret < 0)
    {
      uerr("ECM ERROR: DRVR_IOALLOC of net rx buf failed: %d (%d bytes)\n",
           ret, CDCECM_NETBUF_SIZE);
      goto errout;
    }

  ret = DRVR_IOALLOC(hport->drvr, &priv->txnetbuf, CDCECM_NETBUF_SIZE);
  if (ret < 0)
    {
      uerr("ECM ERROR: DRVR_IOALLOC of net tx buf failed: %d (%d bytes)\n",
           ret, CDCECM_NETBUF_SIZE);
      goto errout;
    }

  ret = DRVR_IOALLOC(hport->drvr, &priv->macnum, IFHWADDRLEN * 2);
  if (ret < 0)
    {
      uerr("ECM ERROR: DRVR_IOALLOC of mac number buf failed: %d (%d bytes)\n",
           ret, IFHWADDRLEN * 2);
      goto errout;
    }

  ret = DRVR_IOALLOC(hport->drvr, &priv->macaddr, IFHWADDRLEN);
  if (ret < 0)
    {
      uerr("ECM ERROR: DRVR_IOALLOC of mac addr failed: %d (%d bytes)\n",
           ret, IFHWADDRLEN);
      goto errout;
    }

  return OK;

  errout:
  usbhost_free_buffers(priv);
  return ret;
}

/****************************************************************************
 * Name: usbhost_free_buffers
 *
 * Description:
 *   Free transfer buffer memory.
 *
 * Input Parameters:
 *   priv - A reference to the class instance.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

 static void usbhost_free_buffers(FAR struct usbhost_cdcecm_s *priv)
{
  FAR struct usbhost_hubport_s *hport;

  DEBUGASSERT(priv != NULL && priv->usbclass.hport != NULL);
  hport = priv->usbclass.hport;

  if (priv->ctrlreq)
    {
      (void)DRVR_FREE(hport->drvr, priv->ctrlreq);
    }

  if (priv->ctrlbuf)
    {
      (void)DRVR_IOFREE(hport->drvr, priv->ctrlbuf);
    }

  if (priv->notification)
    {
      (void)DRVR_IOFREE(hport->drvr, priv->notification);
    }

  if (priv->rxnetbuf)
    {
      (void)DRVR_IOFREE(hport->drvr, priv->rxnetbuf);
    }

  if (priv->txnetbuf)
    {
      (void)DRVR_IOFREE(hport->drvr, priv->txnetbuf);
    }

  if (priv->macnum)
    {
      (void)DRVR_IOFREE(hport->drvr, priv->macnum);
    }

  if (priv->macaddr)
    {
      (void)DRVR_IOFREE(hport->drvr, priv->macaddr);
    }

  priv->ctrlreq      = NULL;
  priv->ctrlbuf      = NULL;
  priv->notification = NULL;
  priv->rxnetbuf     = NULL;
  priv->txnetbuf     = NULL;
  priv->macnum   = NULL;
  priv->macaddr     = NULL;
}

/****************************************************************************
 * Name: usbhost_notification_work
 *
 * Description:
 *   Handle receipt of an asynchronous notification from the CDC device
 *
 * Input Parameters:
 *   arg - The argument provided with the asynchronous I/O was setup
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Probably called from an interrupt handler.
 *
 ****************************************************************************/

static void usbhost_notification_work(FAR void *arg)
{
  FAR struct usbhost_cdcecm_s *priv;
  FAR struct usbhost_hubport_s *hport;
  FAR struct cdc_notification_s *inmsg;
  int ret;

  priv = (FAR struct usbhost_cdcecm_s *)arg;
  DEBUGASSERT(priv);

  hport = priv->usbclass.hport;
  DEBUGASSERT(hport);

  /* Are we still connected? */

  if (!priv->disconnected && priv->intin)
    {
      /* Yes.. Was an interrupt IN message received correctly? */

      uinfo("ECM recv %d bytes int message: %x %x %x %x\n", priv->nnbytes,
        priv->notification[0], priv->notification[1], priv->notification[2],
        priv->notification[3]);

      if (priv->nnbytes >= 0)
        {
          /* Yes.. decode the notification */

          inmsg = (FAR struct cdc_notification_s *)priv->notification;

          /* We care only about the ResponseAvailable notification */

          if ((inmsg->type         == (USB_REQ_DIR_IN | USB_REQ_TYPE_CLASS |
                                       USB_REQ_RECIPIENT_INTERFACE)) &&
              (inmsg->notification == ACM_NETWORK_CONNECTION))
            {
              uinfo("ECM connect: %d\n", usbhost_getle16(inmsg->value));
            }
        }

      /* Setup to receive the next notification */
      ret = DRVR_ASYNCH(hport->drvr, priv->intin,
                        (FAR uint8_t *)priv->notification,
                        SIZEOF_NOTIFICATION_S(0),
                        usbhost_notification_callback,
                        priv);
      if (ret < 0)
        {
          uerr("ECM ERROR: DRVR_ASYNCH failed: %d\n", ret);
        }
    }

}

/****************************************************************************
 * Name: usbhost_notification_callback
 *
 * Description:
 *   Handle receipt of Response Available from the CDC/ECM device
 *
 * Input Parameters:
 *   arg - The argument provided with the asynchronous I/O was setup
 *   nbytes - The number of bytes actually transferred (or a negated errno
 *     value;
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Probably called from an interrupt handler.
 *
 ****************************************************************************/

static void usbhost_notification_callback(FAR void *arg, ssize_t nbytes)
{
  FAR struct usbhost_cdcecm_s *priv = (FAR struct usbhost_cdcecm_s *)arg;
  uint32_t delay = 0;

  DEBUGASSERT(priv);

  /* Are we still connected? */

  if (!priv->disconnected)
    {
      /* Check for a failure.  On higher end host controllers, the
       * asynchronous transfer will pend until data is available (OHCI and
       * EHCI).  On lower end host controllers (like STM32 and EFM32), the
       * transfer will fail immediately when the device NAKs the first
       * attempted interrupt IN transfer (with nbytes == -EAGAIN).  In that
       * case (or in the case of other errors), we must fall back to
       * polling.
       */

      DEBUGASSERT(nbytes >= INT16_MIN && nbytes <= INT16_MAX);
      priv->nnbytes = (int16_t)nbytes;

      uinfo("ECM nbytes: %d\n", nbytes);

      if (nbytes < 0)
        {
          /* This debug output is good to know, but really a nuisance for
           * those configurations where we have to fall back to polling.
           * FIX:  Don't output the message is the result is -EAGAIN.
           */

#if defined(CONFIG_DEBUG_USB) && !defined(CONFIG_DEBUG_INFO)
          if (nbytes != -EAGAIN)
#endif
            {
              uerr("ECM ERROR: Transfer failed: %d\n", nbytes);
            }

          /* We don't know the nature of the failure, but we need to do all
           * that we can do to avoid a CPU hog error loop.
           *
           * Use the low-priority work queue and delay polling for the next
           * event.  We want to use as little CPU bandwidth as possible in
           * this case.
           */

          delay = USBHOST_CDCECM_NTDELAY;
        }

      /* Make sure that the work structure available.  There is a remote
       * chance that this may collide with a device disconnection event.
       */

      if (work_available(&priv->ntwork))
        {
          work_queue(LPWORK, &priv->ntwork,
                     usbhost_notification_work,
                     priv, delay);
        }
    }
}

/****************************************************************************
 * Name: usbhost_destroy
 *
 * Description:
 *   The USB device has been disconnected and the reference count on the USB
 *   host class instance has gone to 1.. Time to destroy the USB host class
 *   instance.
 *
 * Input Parameters:
 *   arg - A reference to the class instance to be destroyed.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void usbhost_destroy(FAR void *arg)
{
  FAR struct usbhost_cdcecm_s *priv = (FAR struct usbhost_cdcecm_s *)arg;
  FAR struct usbhost_hubport_s *hport;
  FAR struct usbhost_driver_s *drvr;

  DEBUGASSERT(priv != NULL && priv->usbclass.hport != NULL);
  hport = priv->usbclass.hport;

  DEBUGASSERT(hport->drvr);
  drvr = hport->drvr;

  uinfo("ECM crefs: %d\n", priv->crefs);

  /* Unregister the driver */

  /* Release the device name used by this connection */

  usbhost_freedevno(priv);

  /* Free the endpoints */

  if (priv->intin)
    {
      DRVR_EPFREE(hport->drvr, priv->intin);
    }

  if (priv->bulkin)
    {
      DRVR_EPFREE(hport->drvr, priv->bulkin);
    }

  if (priv->bulkout)
    {
      DRVR_EPFREE(hport->drvr, priv->bulkout);
    }

  /* Free any transfer buffers */

  /* Free the function address assigned to this device */

  usbhost_devaddr_destroy(hport, hport->funcaddr);
  hport->funcaddr = 0;

  /* Disconnect the USB host device */

  DRVR_DISCONNECT(drvr, hport);

  /* And free the class instance.  Hmmm.. this may execute on the worker
   * thread and the work structure is part of what is getting freed.  That
   * should be okay because once the work contained is removed from the
   * queue, it should not longer be accessed by the worker thread.
   */

  usbhost_freeclass(priv);
}

/****************************************************************************
 * struct usbhost_registry_s methods
 ****************************************************************************/

/****************************************************************************
 * Name: usbhost_create
 *
 * Description:
 *   This function implements the create() method of struct
 *   usbhost_registry_s.  The create() method is a callback into the class
 *   implementation.  It is used to (1) create a new instance of the USB
 *   host class state and to (2) bind a USB host driver "session" to the
 *   class instance.  Use of this create() method will support environments
 *   where there may be multiple USB ports and multiple USB devices
 *   simultaneously connected.
 *
 * Input Parameters:
 *   hport - The hub port that manages the new class instance.
 *   id - In the case where the device supports multiple base classes,
 *     subclasses, or protocols, this specifies which to configure for.
 *
 * Returned Value:
 *   On success, this function will return a non-NULL instance of struct
 *   usbhost_class_s that can be used by the USB host driver to communicate
 *   with the USB host class.  NULL is returned on failure; this function
 *   will fail only if the hport input parameter is NULL or if there are
 *   insufficient resources to create another USB host class instance.
 *
 ****************************************************************************/

 static struct usbhost_class_s *usbhost_create(
              FAR struct usbhost_hubport_s *hport,
              FAR const struct usbhost_id_s *id)
{
  FAR struct usbhost_cdcecm_s *priv;

  /* Allocate a USB host CDC/ECM class instance */

  priv = usbhost_allocclass();
  if (priv)
    {
      /* Initialize the allocated CDC/ECM class instance */

      memset(priv, 0, sizeof(struct usbhost_cdcecm_s));

      /* Assign a device number to this class instance */

      if (usbhost_allocdevno(priv) == OK)
        {
          /* Initialize class method function pointers */

          priv->usbclass.hport        = hport;
          priv->usbclass.connect      = usbhost_connect;
          priv->usbclass.disconnected = usbhost_disconnected;

          /* The initial reference count is 1... One reference is held by
           * the driver.
           */

          priv->crefs = 1;

          /* Initialize mutex (this works in the interrupt context) */

          nxmutex_init(&priv->lock);

          /* Return the instance of the USB class driver */

          return &priv->usbclass;
        }
    }

  /* An error occurred. Free the allocation and return NULL on all failures */

  if (priv)
    {
      usbhost_freeclass(priv);
    }

  return NULL;
}

/****************************************************************************
 * struct usbhost_class_s methods
 ****************************************************************************/

/****************************************************************************
 * Name: usbhost_connect
 *
 * Description:
 *   This function implements the connect() method of struct
 *   usbhost_class_s.  This method is a callback into the class
 *   implementation.  It is used to provide the device's configuration
 *   descriptor to the class so that the class may initialize properly
 *
 * Input Parameters:
 *   usbclass - The USB host class entry previously obtained from a call to
 *     create().
 *   configdesc - A pointer to a uint8_t buffer container the configuration
 *     descriptor.
 *   desclen - The length in bytes of the configuration descriptor.
 *
 * Returned Value:
 *   On success, zero (OK) is returned. On a failure, a negated errno value
 *   is returned indicating the nature of the failure
 *
 *   NOTE that the class instance remains valid upon return with a failure.
 *   It is the responsibility of the higher level enumeration logic to call
 *   CLASS_DISCONNECTED to free up the class driver resources.
 *
 * Assumptions:
 *   - This function will *not* be called from an interrupt handler.
 *   - If this function returns an error, the USB host controller driver
 *     must call to DISCONNECTED method to recover from the error
 *
 ****************************************************************************/

static int usbhost_connect(FAR struct usbhost_class_s *usbclass,
                           FAR const uint8_t *configdesc, int desclen)
{
  FAR struct usbhost_cdcecm_s *priv =
      (FAR struct usbhost_cdcecm_s *)usbclass;
  int ret;

  DEBUGASSERT(priv != NULL &&
              configdesc != NULL &&
              desclen >= sizeof(struct usb_cfgdesc_s));

  /* Parse the configuration descriptor to get the endpoints */

  ret = usbhost_cfgdesc(priv, configdesc, desclen);
  if (ret < 0)
    {
      uerr("ECM ERROR: usbhost_cfgdesc() failed: %d\n", ret);
    }
  else
    {
      /* Now configure the device and register the NuttX driver */

      ret = usbhost_devinit(priv);
      if (ret < 0)
        {
          uerr("ECM ERROR: usbhost_devinit() failed: %d\n", ret);
        }
    }

  return ret;
}

/****************************************************************************
 * Name: usbhost_disconnected
 *
 * Description:
 *   This function implements the disconnected() method of struct
 *   usbhost_class_s.  This method is a callback into the class
 *   implementation.  It is used to inform the class that the USB device has
 *   been disconnected.
 *
 * Input Parameters:
 *   usbclass - The USB host class entry previously obtained from a call to
 *     create().
 *
 * Returned Value:
 *   On success, zero (OK) is returned. On a failure, a negated errno value
 *   is returned indicating the nature of the failure
 *
 * Assumptions:
 *   This function may be called from an interrupt handler.
 *
 ****************************************************************************/

static int usbhost_disconnected(FAR struct usbhost_class_s *usbclass)
{
  FAR struct usbhost_cdcecm_s *priv =
      (FAR struct usbhost_cdcecm_s *)usbclass;

  irqstate_t flags;

  DEBUGASSERT(priv != NULL);

  /* Set an indication to any users of the device that the device is no
   * longer available.
   */

  flags              = enter_critical_section();
  priv->disconnected = true;

  /* Now check the number of references on the class instance.  If it is one,
   * then we can free the class instance now.  Otherwise, we will have to
   * wait until the holders of the references free them by closing the
   * block driver.
   */

  uinfo("ECM crefs: %d\n", priv->crefs);

  if (priv->crefs == 1)
    {
      /* Destroy the class instance.  If we are executing from an interrupt
       * handler, then defer the destruction to the worker thread.
       * Otherwise, destroy the instance now.
       */

      if (up_interrupt_context())
        {
          /* Destroy the instance on the worker thread. */

          uinfo("ECM Queuing destruction: worker %p->%p\n",
                priv->destroywork.worker, usbhost_destroy);
          DEBUGASSERT(priv->destroywork.worker == NULL);
          work_queue(LPWORK, &priv->destroywork,
                           usbhost_destroy, priv, 0);
        }
      else
        {
          /* Do the work now */

          usbhost_destroy(priv);
        }
    }

  leave_critical_section(flags);
  return OK;
}

/****************************************************************************
 * Name: cdcecm_transmit
 *
 * Description:
 *   Start hardware transmission.  Called either from the txdone interrupt
 *   handling or from watchdog based polling.
 *
 * Input Parameters:
 *   priv - Reference to the driver state structure
 *
 * Returned Value:
 *   OK on success; a negated errno on failure
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

static int cdcecm_transmit(FAR struct usbhost_cdcecm_s *priv)
{
  FAR struct usbhost_hubport_s *hport;
  ssize_t ret;

  hport = priv->usbclass.hport;

  uinfo("ECM transmit packet: %d bytes\n", priv->netdev.d_len);

  memcpy(priv->txnetbuf, priv->netdev.d_buf, priv->netdev.d_len);

  ret = DRVR_TRANSFER(hport->drvr, priv->bulkout, priv->txnetbuf, priv->netdev.d_len);
  if (ret < 0)
    {
      uerr("ECM transfer returned error: %d\n", ret);
      return ret;
    }

  NETDEV_TXDONE(&priv->netdev);
  return OK;
}

/****************************************************************************
 * Name: cdcecm_receive
 *
 * Description:
 *   Handle a received packet.
 *
 * Input Parameters:
 *   priv - Reference to the driver state structure
 *
 * Returned Value:
 *   OK on success; a negated errno on failure
 *
 ****************************************************************************/

static void cdcecm_receive(FAR struct usbhost_cdcecm_s *priv,
                            FAR uint8_t *buf, size_t len)
{
  uinfo("ECM received packet: %d len\n", len);

  net_lock();

  NETDEV_RXPACKETS(&priv->netdev);

  /* Any ACK or other response packet generated by the network stack
   * will always be shorter than the received packet, therefore it is
   * safe to pass the received frame buffer directly.
   */

  priv->netdev.d_buf = buf;
  priv->netdev.d_len = len;

  uinfo("ECM buf type= %x, ipv4= %x, ipv6= %x, net_arp= %x\n", BUF->type,
  HTONS(ETHTYPE_IP), HTONS(ETHTYPE_IP6), HTONS(ETHTYPE_ARP));

#ifdef CONFIG_NET_IPv4
  if (BUF->type == HTONS(ETHTYPE_IP))
    {
      ninfo("IPv4 frame\n");
      NETDEV_RXIPV4(&priv->netdev);

     /* Receive an IPv4 packet from the network device */

      ipv4_input(&priv->netdev);

      /* Check for a reply to the IPv4 packet */

      if (priv->netdev.d_len > 0)
        {
          cdcecm_transmit(priv);
        }
    }
  else
#endif
#ifdef CONFIG_NET_IPv6
  if (BUF->type == HTONS(ETHTYPE_IP6))
    {
      ninfo("IPv6 frame\n");
      NETDEV_RXIPV6(&priv->netdev);

      /* Dispatch IPv6 packet to the network layer */

      ipv6_input(&priv->netdev);

      /* Check for a reply to the IPv6 packet */

      if (priv->netdev.d_len > 0)
        {
          cdcecm_transmit(priv);
        }
    }
  else
#endif
#ifdef CONFIG_NET_ARP
  if (BUF->type == HTONS(ETHTYPE_ARP))
    {
      ninfo("ARP frame\n");
      /* Dispatch ARP packet to the network layer */

      arp_input(&priv->netdev);
      NETDEV_RXARP(&priv->netdev);

      /* If the above function invocation resulted in data that should be
       * sent out on the network, d_len field will set to a value > 0.
       */

      if (priv->netdev.d_len > 0)
        {
          cdcecm_transmit(priv);
        }
    }
  else
#endif
    {
      NETDEV_RXERRORS(&priv->netdev);
    }

  net_unlock();
}

/****************************************************************************
 * Name: cdcecm_txpoll
 *
 * Description:
 *   The transmitter is available, check if the network has any outgoing
 *   packets ready to send.  This is a callback from devif_poll().
 *   devif_poll() may be called:
 *
 *   1. When the preceding TX packet send is complete,
 *   2. When the preceding TX packet send timesout and the interface is reset
 *   3. During normal TX polling
 *
 * Input Parameters:
 *   dev - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   OK on success; a negated errno on failure
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

static int cdcecm_txpoll(FAR struct net_driver_s *dev)
{
  FAR struct usbhost_cdcecm_s *priv =
      (FAR struct usbhost_cdcecm_s *)dev->d_private;

  /* If the polling resulted in data that should be sent out on the network,
   * the field d_len is set to a value > 0.
   */

  DEBUGASSERT(priv->netdev.d_buf == (FAR uint8_t *)priv->txpktbuf);

  nxmutex_lock(&priv->lock);

  /* Send the packet */

  cdcecm_transmit(priv);

  nxmutex_unlock(&priv->lock);

  return 0;
}

/****************************************************************************
 * Name: cdcecm_ifup
 *
 * Description:
 *   NuttX Callback: Bring up the ECM interface when an IP address is
 *   provided
 *
 * Input Parameters:
 *   dev - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static int cdcecm_ifup(FAR struct net_driver_s *dev)
{
  FAR struct usbhost_cdcecm_s *priv =
      (FAR struct usbhost_cdcecm_s *)dev->d_private;
  FAR struct usbhost_hubport_s *hport = priv->usbclass.hport;
  int ret;

#ifdef CONFIG_NET_IPv4
  ninfo("Bringing up: %u.%u.%u.%u\n",
        ip4_addr1(dev->d_ipaddr), ip4_addr2(dev->d_ipaddr),
        ip4_addr3(dev->d_ipaddr), ip4_addr4(dev->d_ipaddr));
#endif
#ifdef CONFIG_NET_IPv6
  ninfo("Bringing up: %04x:%04x:%04x:%04x:%04x:%04x:%04x:%04x\n",
        dev->d_ipv6addr[0], dev->d_ipv6addr[1], dev->d_ipv6addr[2],
        dev->d_ipv6addr[3], dev->d_ipv6addr[4], dev->d_ipv6addr[5],
        dev->d_ipv6addr[6], dev->d_ipv6addr[7]);
#endif

  if (priv->bulkin)
    {
      ret = DRVR_ASYNCH(hport->drvr, priv->bulkin,
            priv->rxnetbuf, priv->maxsegmentsize,
            usbhost_bulkin_callback, priv);
      if (ret < 0)
        {
          uerr("ECM ERROR: DRVR_ASYNCH failed on bulkin: %d\n", ret);
        }
    }

  priv->bifup = true;
  return OK;
}

/****************************************************************************
 * Name: cdcecm_ifdown
 *
 * Description:
 *   NuttX Callback: Stop the interface.
 *
 * Input Parameters:
 *   dev - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static int cdcecm_ifdown(FAR struct net_driver_s *dev)
{
  FAR struct usbhost_cdcecm_s *priv =
      (FAR struct usbhost_cdcecm_s *)dev->d_private;
  irqstate_t flags;

  flags = enter_critical_section();

  /* Mark the device "down" */

  priv->bifup = false;

  leave_critical_section(flags);
  return OK;
}

/****************************************************************************
 * Name: cdcecm_txavail_work
 *
 * Description:
 *   Driver callback invoked when new TX data is available.  This is a
 *   stimulus perform an out-of-cycle poll and, thereby, reduce the TX
 *   latency.
 *
 * Input Parameters:
 *   dev - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Called in normal user mode
 *
 ****************************************************************************/

static void cdcecm_txavail_work(FAR void *arg)
{
  FAR struct usbhost_cdcecm_s *priv = (FAR struct usbhost_cdcecm_s *)arg;

  net_lock();

  priv->netdev.d_buf = (FAR uint8_t *)priv->txpktbuf;

  if (priv->bifup)
    {
      devif_poll(&priv->netdev, cdcecm_txpoll);
    }

  net_unlock();
}

/****************************************************************************
 * Name: cdcecm_txavail
 *
 * Description:
 *   Driver callback invoked when new TX data is available.  This is a
 *   stimulus perform an out-of-cycle poll and, thereby, reduce the TX
 *   latency.
 *
 * Input Parameters:
 *   dev - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Called from the network stack with the network locked.
 *
 ****************************************************************************/

static int cdcecm_txavail(FAR struct net_driver_s *dev)
{
  FAR struct usbhost_cdcecm_s *priv =
      (FAR struct usbhost_cdcecm_s *)dev->d_private;

  ninfo("transmit maxlen: %d\n", priv->netdev.d_len, MAX_NETDEV_PKTSIZE);

  if (work_available(&priv->txpollwork))
    {
      work_queue(LPWORK, &priv->txpollwork, cdcecm_txavail_work, priv, 0);
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: usbhost_cdcecm_initialize
 *
 * Description:
 *   Initialize the USB host CDC/ECM class.  This function should be called
 *   be platform-specific code in order to initialize and register support
 *   for the USB host CDC/ECM class.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   On success this function will return zero (OK);  A negated errno value
 *   will be returned on failure.
 *
 ****************************************************************************/
int usbhost_cdcecm_initialize(void)
{
  /* Advertise our availability to support (certain) CDC/ACM devices */
  return usbhost_registerclass(&g_cdcecm);
}

#endif /* CONFIG_USBHOST_CDCECM */
