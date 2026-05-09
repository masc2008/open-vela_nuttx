/****************************************************************************
 * drivers/usbdev/hidmouse.c
 *
 * SPDX-License-Identifier: Apache-2.0
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

/* This is a driver for the USB Device Firmware Upgrade protocol v1.1.
 * Currently it supports the app-side ("Run-Time") part of the protocol:
 * a sequence of DFU_DETACH and USB reset commands, which will reboot into
 * a separate USB DFU bootloader.
 *
 * The bootloader is provided by board-specific logic, or STM32's
 * built-in ROM bootloader can be used.
 *
 * https://www.usb.org/sites/default/files/DFU_1.1.pdf
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/usb/usb.h>
#include <nuttx/usb/hid.h>
#include <nuttx/usb/usbdev.h>
#include <nuttx/usb/usbdev_trace.h>
#include <nuttx/usb/hidmouse.h>
#include <nuttx/kmalloc.h>
#include <nuttx/wqueue.h>
#include <debug.h>
#include <errno.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

/****************************************************************************
 * Pre-processor definitions
 ****************************************************************************/

#define USBMOUSE_CHARDEV_PATH       "/dev/mouse"

#define USBMOUSE_REPORT_ID          (1)

#define HID_CLASS_VERSION           (0x0111)

/* Buffer big enough for any of our descriptors (the config descriptor is the
 * biggest).
 */

#define USBMOUSE_MXDESCLEN          (128)
#define USBMOUSE_MAXSTRLEN          (USBMOUSE_MXDESCLEN - 2)


#define USBMOUSE_INTERFACESTRID     (1)
#define USBMOUSE_NSTRIDS            (1)

#define USBMOUSE_NCONFIGS           (1)

/****************************************************************************
 * Private Types
 ****************************************************************************/


/* Container to support a list of requests */

struct hid_wrreq_s
{
  FAR struct hid_wrreq_s *flink;    /* Implements a singly linked list */
  FAR struct usbdev_req_s *req;     /* The contained request */
};

struct hid_report_s
{
  uint8_t report_id;                       /* Count of opened instances */
  FAR struct usbmouse_report_s report;        /* The contained request */
};

struct mouse_driver_s
{
  struct usbdevclass_driver_s drvr;
  struct usbdev_devinfo_s  devinfo;
  struct work_s work;
  uint8_t nwrq;                     /* Number of queue write requests (in txfree) */

  bool linked;                      /* Indicates if the driver has been linked */
  uint8_t crefs;                    /* Count of opened instances */
  mutex_t lock;                     /* Enforces device exclusive access */

  FAR struct usbdev_req_s *ctrlreq;  /* Pointer to preallocated control request */
  FAR struct usbdev_ep_s *epintin;   /* Interrupt IN endpoint structure */
  struct sq_queue_s txfree;          /* Available write request containers */

  /* Pre-allocated write request containers.  The write requests will
   * be linked in a free list (txfree), and used to send requests to
   * EPINTIN.
   */

  struct hid_wrreq_s wrreqs[CONFIG_USBMOUSE_NWRREQS];
  FAR struct hid_wrreq_s *wrcontainer;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Completion event handlers ************************************************/

static void usbclass_ep0incomplete(FAR struct usbdev_ep_s *ep,
                 FAR struct usbdev_req_s *req);
static void hidep_wrcomplete(FAR struct usbdev_ep_s *ep,
                 FAR struct usbdev_req_s *req);

/* usbclass callbacks */

static int  usbclass_setup(FAR struct usbdevclass_driver_s *driver,
                           FAR struct usbdev_s *dev,
                           FAR const struct usb_ctrlreq_s *ctrl,
                           FAR uint8_t *dataout, size_t outlen);
static int  usbclass_bind(FAR struct usbdevclass_driver_s *driver,
                          FAR struct usbdev_s *dev);
static void usbclass_unbind(FAR struct usbdevclass_driver_s *driver,
                            FAR struct usbdev_s *dev);
static void usbclass_disconnect(FAR struct usbdevclass_driver_s *driver,
                                FAR struct usbdev_s *dev);

/* Char device Operations ***************************************************/

static int usbdev_fs_open(FAR struct file *filep);
static int usbdev_fs_close(FAR struct file *filep);
static ssize_t usbdev_fs_read(FAR struct file *filep, FAR char *buffer,
                              size_t len);
static ssize_t usbdev_fs_write(FAR struct file *filep,
                               FAR const char *buffer, size_t len);


/* Transfer helpers *********************************************************/

static int hidmouse_sndpacket(FAR struct mouse_driver_s *priv, uint8_t *buffer);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* USB driver operations */

const static struct usbdevclass_driverops_s g_mouse_driverops =
{
  &usbclass_bind,
  &usbclass_unbind,
  &usbclass_setup,
  &usbclass_disconnect,
  NULL,
  NULL
};

/* Char device **************************************************************/

static const struct file_operations g_mouse_fs_fops =
{
  usbdev_fs_open,  /* open */
  usbdev_fs_close, /* close */
  usbdev_fs_read,  /* read */
  usbdev_fs_write, /* write */
  NULL,            /* seek */
  NULL,            /* ioctl */
  NULL,            /* mmap */
  NULL,            /* truncate */
  NULL             /* poll */
};

/* USB descriptor ***********************************************************/
static const uint8_t mouse_report_descriptor[] = {
    0x05, 0x01,         // USAGE_PAGE (Generic Desktop)
    0x09, 0x02,         // USAGE (Mouse)
    0xA1, USBMOUSE_REPORT_ID, // Collection (Application)
    0x85, 0x01,         // Report ID (0x1)
    0x09, 0x01,         // USAGE (Pointer)
    0xA1, 0x00,         // Collection (Physical)
    0x05, 0x09,         // USAGE_PAGE (Button)
    0x19, 0x01,         // USAGE_Minimum (0x01)
    0x29, 0x05,         // USAGE_Maximum (0x05)
    0x15, 0x00,         // LOGICAL_Minimum (0x00)
    0x25, 0x01,         // LOGICAL_Maximum (0x01)
    0x95, 0x05,         // Report Count (0x05)
    0x75, 0x01,         // Report Size (0x01)
    0x81, 0x02,         // Input (Data, Var, Abs)
    0x95, 0x01,         // Report Count (0x01)
    0x75, 0x03,         // Report Size (0x03)
    0x81, 0x01,         // Input (Cnst, Ary, Abs)
    0x05, 0x01,         // USAGE_PAGE (Generic Desktop)
    0x09, 0x30,         // USAGE (X)
    0x09, 0x31,         // USAGE (Y)
    0x16, 0x01, 0x80,   // LOGICAL_Minimum (0x8001)
    0x26, 0xFF, 0x7F,   // LOGICAL_Maximum (0x7FFF)
    0x75, 0x10,         // Report Size (0x10)
    0x95, 0x02,         // Report Count (0x02)
    0x81, 0x06,         // Input (Data, Var, Rel)
    0x09, 0x38,         // USAGE (Wheel)
    0x15, 0x81,         // LOGICAL_Minimum (0x81)
    0x25, 0x7F,         // LOGICAL_Maximum (0x7F)
    0x75, 0x08,         // Report Size (0x08)
    0x95, 0x01,         // Report Count (0x01)
    0x81, 0x06,         // Input (Data, Var, Rel)
    0xC0,               // END_COLLECTION
    0xC0,               // END_COLLECTION
    0x05, 0x01,         // USAGE_PAGE (Generic Desktop)
    0x09, 0x00,         // USAGE (Undefined)
    0xA1, 0x01,         // Collection (Application)
    0x85, 0x05,         // Report ID (0x5)
    0x06, 0x00, 0xFF,   // USAGE_PAGE (Undefined)
    0x09, 0x01,         // USAGE (1)
    0x15, 0x81,         // LOGICAL_Minimum (0x81)
    0x25, 0x7F,         // LOGICAL_Maximum (0x7F)
    0x75, 0x08,         // Report Size (0x08)
    0x95, 0x07,         // Report Count (0x07)
    0xB1, 0x02,         // Feature (Data, Var, Abs)
    0xC0,               // END_COLLECTION
};

static const struct usbhid_descriptor_s g_mouse_hiddesc =
{
  .len          = USB_SIZEOF_IFDESC,
  .type         = USBHID_DESCTYPE_HID,
  .hid[0]       = LSBYTE(HID_CLASS_VERSION),
  .hid[1]       = MSBYTE(HID_CLASS_VERSION),
  .country      = 0x21,
  .ndesc        = 0x01,
  .classdesc    = USBHID_DESCTYPE_REPORT,
  .desclen[0]   = LSBYTE(sizeof(mouse_report_descriptor)),
  .desclen[1]   = MSBYTE(sizeof(mouse_report_descriptor))
};


static struct usb_iaddesc_s g_mouse_iaddesc =
{
  .len      = USB_SIZEOF_IADDESC,
  .type     = USB_DESC_TYPE_INTERFACEASSOCIATION,
  .firstif  = 0,
  .nifs     = 1,
  .classid  = USB_CLASS_HID,
  .subclass = 0x01,
  .protocol = 0x02,
  .ifunction = 0
};

static const struct usb_ifdesc_s g_mouse_ifdesc =
{
  .len      = USB_SIZEOF_IFDESC,
  .type     = USB_DESC_TYPE_INTERFACE,
  .ifno     = 0,
  .alt      = 0,
  .neps     = 1,
  .classid  = USB_CLASS_HID,
  .subclass = 0x01,
  .protocol = 0x02,
  .iif      = 0
};

static const struct usbdev_epinfo_s g_mouse_epintin =
{
  .desc =
    {
      .len       = USB_SIZEOF_EPDESC,
      .type      = USB_DESC_TYPE_ENDPOINT,
      .addr      = USB_DIR_IN,
      .attr      = USB_EP_ATTR_XFER_INT |
                   USB_EP_ATTR_NO_SYNC   |
                   USB_EP_ATTR_USAGE_DATA,
#ifdef CONFIG_USBDEV_DUALSPEED
      .interval  = CONFIG_USBMOUSE_FSEPINTIN_INTERVAL,
#else
      .interval  = CONFIG_USBMOUSE_HSEPINTIN_INTERVAL,
#endif
    },
  .reqnum        = CONFIG_USBMOUSE_NWRREQS,
  .fssize        = 8,
#ifdef CONFIG_USBDEV_DUALSPEED
  .hssize        = 8,
#endif
};

static const FAR struct usbdev_epinfo_s *g_mouse_epinfos[USBMOUSE_NUM_EPS] =
{
  &g_mouse_epintin,
};

/****************************************************************************
 * Name: usbdev_fs_open
 *
 * Description:
 *   Open usbdev fs device. Only one open() instance is supported.
 *
 ****************************************************************************/

static int usbdev_fs_open(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct mouse_driver_s *priv = inode->i_private;
  int ret;

  uinfo("entry: <%s> %d\n", inode->i_name, priv->crefs);

  /* Get exclusive access to the device structures */

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  if (!priv->linked)
    {
      nxmutex_unlock(&priv->lock);
      return -ENOTCONN;
    }

  priv->crefs += 1;

  ASSERT(priv->crefs != 0);

  nxmutex_unlock(&priv->lock);

  return ret;
}

/****************************************************************************
 * Name: usbdev_fs_close
 *
 * Description:
 *   Close usbdev fs device.
 *
 ****************************************************************************/

static int usbdev_fs_close(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct mouse_driver_s *priv = inode->i_private;
  int ret;

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  uinfo("entry: <%s> %d\n", inode->i_name, priv->crefs);

  priv->crefs -= 1;

  nxmutex_unlock(&priv->lock);

  return OK;
}

/****************************************************************************
 * Name: usbdev_fs_read
 *
 * Description:
 *   Read usbdev fs device.
 *
 ****************************************************************************/

static ssize_t usbdev_fs_read(FAR struct file *filep, FAR char *buffer,
                              size_t len)
{
  /* Mouse dev won't try to read the host */

  return -ENOSYS;
}

/****************************************************************************
 * Name: usbdev_fs_write
 *
 * Description:
 *   Write usbdev fs device.
 *
 ****************************************************************************/

static ssize_t usbdev_fs_write(FAR struct file *filep,
                               FAR const char *buffer, size_t len)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct mouse_driver_s *priv = inode->i_private;
  int ret;

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      goto errout;
    }

  if (len > (CONFIG_USBMOUSE_EPINTIN_SIZE - 1))
    {
        len = CONFIG_USBMOUSE_EPINTIN_SIZE - 1;
    }

  len = hidmouse_sndpacket(priv, buffer);

  uinfo("wrote %u bytes, nwrq %d\n", len, priv->nwrq);


errout:
  nxmutex_unlock(&priv->lock);

  return len;
}

/****************************************************************************
 * Name: usbclass_ep0incomplete
 *
 * Description:
 *   Handle completion of EP0 control operations
 *
 ****************************************************************************/

static void usbclass_ep0incomplete(FAR struct usbdev_ep_s *ep,
                                 FAR struct usbdev_req_s *req)
{
  if (req->result || req->xfrd != req->len)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_REQRESULT),
               (uint16_t)-req->result);
    }
}

/****************************************************************************
 * Name: hidmouse_sndpacket
 *
 * Description:
 *   This function obtains write requests, transfers the TX data into the
 *   request, and submits the requests to the USB controller.  This
 *   continues until either (1) there are no further packets available, or
 *   (2) there is no further data to send.
 *
 ****************************************************************************/

static int hidmouse_sndpacket(FAR struct mouse_driver_s *priv, uint8_t *buffer)
{
  FAR struct usbdev_ep_s *ep;
  FAR struct usbdev_req_s *req;
  FAR struct hid_wrreq_s *wrcontainer;
  FAR struct hid_report_s *report;
  irqstate_t flags;
  int ret;

  DEBUGASSERT(priv != NULL && priv->epintin != NULL);

  flags = enter_critical_section();

  /* Use our interrupt IN endpoint for the transfer */

  ep = priv->epintin;

  /* Remove the next container from the request list */

  wrcontainer = (FAR struct hid_wrreq_s *)sq_remfirst(&priv->txfree);
  if (wrcontainer == NULL)
    {
      ret = -ENOMEM;
      goto errout_with_flags;
    }

  /* Decrement the count of write requests */

  priv->nwrq--;

  /* Format the mouse report data */

  DEBUGASSERT(wrcontainer->req != NULL);
  req                  = wrcontainer->req;

  DEBUGASSERT(req->buf != NULL);
  report               = (FAR struct usbmouse_report_s *)req->buf;

  report->report_id    = USBMOUSE_REPORT_ID;
  memcpy((&report->report), buffer, (CONFIG_USBMOUSE_EPINTIN_SIZE - 1));

  /* Then submit the request to the endpoint */

  req->len             = CONFIG_USBMOUSE_EPINTIN_SIZE;
  req->priv            = wrcontainer;
  req->flags           = USBDEV_REQFLAGS_NULLPKT;
  ret                  = EP_SUBMIT(ep, req);

  if (ret < 0)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_SUBMITFAIL), (uint16_t)-ret);
    }
  else
    {
        ret = req->len - 1;
    }

errout_with_flags:

  leave_critical_section(flags);
  uinfo("ep_submit ret %d \n", ret);
  return ret;
}

/****************************************************************************
 * Name: hidep_wrcomplete
 *
 * Description:
 *   Handle completion of write request.  This function probably executes
 *   in the context of an interrupt handler.
 *
 ****************************************************************************/

static void hidep_wrcomplete(FAR struct usbdev_ep_s *ep,
                              FAR struct usbdev_req_s *req)
{
  FAR struct mouse_driver_s *priv;
  FAR struct hid_wrreq_s *wrcontainer;
  irqstate_t flags;

  /* Extract references to our private data */

  priv        = (FAR struct mouse_driver_s *)ep->priv;
  wrcontainer = (FAR struct hid_wrreq_s *)req->priv;

  /* Return the write request to the free list */

  flags = enter_critical_section();
  sq_addlast((FAR sq_entry_t *)wrcontainer, &priv->txfree);
  priv->nwrq++;
  leave_critical_section(flags);

  uinfo("nwrq %d", priv->nwrq);

  /* Send the next packet unless this was some unusual termination
   * condition
   */

  switch (req->result)
    {
    case OK: /* Normal completion */
      {
        usbtrace(TRACE_CLASSWRCOMPLETE, priv->nwrq);
      }
      break;

    case -ESHUTDOWN: /* Disconnection */
      {
        usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_WRSHUTDOWN), priv->nwrq);
      }
      break;

    default: /* Some other error occurred */
      {
        usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_WRUNEXPECTED),
                 (uint16_t)-req->result);
      }
      break;
    }
}

/****************************************************************************
 * Name: usbclass_mkcfgdesc
 *
 * Description:
 *   Construct the configuration descriptor
 *
 ****************************************************************************/

static int16_t usbclass_mkcfgdesc(FAR uint8_t *buf,
                                  FAR struct usbdev_devinfo_s *devinfo,
                                  uint8_t speed, uint8_t type)
{
  FAR uint8_t *epdesc;
  FAR struct usb_iaddesc_s *iaddesc;
  FAR struct usb_ifdesc_s *itfdesc;
  FAR struct usbhid_descriptor_s *hiddesc;
  uint32_t totallen = 0;
  int ret;

  /* Check for switches between high and full speed */

  if (type == USB_DESC_TYPE_OTHERSPEEDCONFIG && speed < USB_SPEED_SUPER)
    {
      speed = speed == USB_SPEED_HIGH ? USB_SPEED_FULL : USB_SPEED_HIGH;
    }

  iaddesc = (FAR struct usb_iaddesc_s *)buf;
  itfdesc = (FAR struct usb_ifdesc_s *)(buf + USB_SIZEOF_IADDESC);
  hiddesc = (FAR struct usbhid_descriptor_s *)(buf + USB_SIZEOF_IADDESC + USB_SIZEOF_IFDESC);
  epdesc = (FAR uint8_t *)(buf + USB_SIZEOF_IADDESC + 2 * USB_SIZEOF_IFDESC);

  g_mouse_iaddesc.firstif = devinfo->ifnobase;
  memcpy(iaddesc, &g_mouse_iaddesc, sizeof(g_mouse_iaddesc));
  totallen += sizeof(struct usb_iaddesc_s);

  memcpy(itfdesc, &g_mouse_ifdesc, sizeof(g_mouse_ifdesc));
  totallen += sizeof(struct usb_ifdesc_s);

  memcpy(hiddesc, &g_mouse_hiddesc, sizeof(g_mouse_hiddesc));
  totallen += sizeof(struct usbhid_descriptor_s);

  ret = usbdev_copy_epdesc((FAR struct usb_epdesc_s *)epdesc,
                           devinfo->epno[USBMOUSE_EP_INTIN_IDX],
                           speed, &g_mouse_epintin);
  totallen += ret;
  epdesc += ret;

  /* For composite device, apply possible offset to the interface numbers */

  itfdesc->ifno = devinfo->ifnobase;

  return totallen;
}

/****************************************************************************
 * Name: usbclass_mkstrdesc
 *
 * Description:
 *   Construct the string descriptor
 *
 ****************************************************************************/

static int usbclass_mkstrdesc(uint8_t id, FAR struct usb_strdesc_s *strdesc)
{
  FAR uint8_t *data = (FAR uint8_t *)(strdesc + 1);
  FAR const char *str;
  int len;
  int ndata;
  int i;

  uinfo("id=%02x\n", id);

  switch (id)
    {
      /* Composite driver removes offset before calling mkstrdesc() */

      case USBMOUSE_INTERFACESTRID:
        str = CONFIG_USBMOUSE_INTERFACESTR;
        break;

      default:
        return -EINVAL;
    }

  /* The string is utf16-le.  The poor man's utf-8 to utf16-le
   * conversion below will only handle 7-bit en-us ascii
   */

  len = strlen(str);
  if (len > (USBMOUSE_MAXSTRLEN / 2))
    {
      len = (USBMOUSE_MAXSTRLEN / 2);
    }

  for (i = 0, ndata = 0; i < len; i++, ndata += 2)
    {
      data[ndata]     = str[i];
      data[ndata + 1] = 0;
    }

  strdesc->len  = ndata + 2;
  strdesc->type = USB_DESC_TYPE_STRING;
  return strdesc->len;
}

/****************************************************************************
 * Name: usbclass_classobject
 *
 * Description:
 *   Allocate memory for the driver class object
 *
 * Returned Value:
 *   0 on success, negative error code on failure.
 *
 ****************************************************************************/

static int usbclass_classobject(int minor,
                                FAR struct usbdev_devinfo_s *devinfo,
                                FAR struct usbdevclass_driver_s **classdev)
{
  FAR struct mouse_driver_s *alloc;

  alloc = kmm_zalloc(sizeof(struct mouse_driver_s));
  if (alloc == NULL)
    {
      return -ENOMEM;
    }

  *classdev = &alloc->drvr;

  /* Initialize the USB serial driver structure */
  sq_init(&alloc->txfree);

#ifdef CONFIG_USBDEV_DUALSPEED
  alloc->drvr.speed = USB_SPEED_HIGH;
#else
  alloc->drvr.speed = USB_SPEED_FULL;
#endif
  alloc->drvr.ops = &g_mouse_driverops;

  /* Save the caller provided device description (composite only) */

  memcpy(&alloc->devinfo, devinfo,
         sizeof(struct usbdev_devinfo_s));

  uinfo("enter speed %d\n", alloc->drvr.speed);

  return OK;
}

/****************************************************************************
 * Name: usbdev_register_driver
 *
 * Description:
 *   Register the driver after successful set configuration.
 *
 ****************************************************************************/

static void usbdev_register_driver(FAR void *arg)
{
  FAR struct mouse_driver_s *priv = arg;

  register_driver(USBMOUSE_CHARDEV_PATH, &g_mouse_fs_fops, 0666, priv);

  priv->linked = true;
}

static int  usbclass_setup(FAR struct usbdevclass_driver_s *driver,
                           FAR struct usbdev_s *dev,
                           FAR const struct usb_ctrlreq_s *ctrl,
                           FAR uint8_t *dataout, size_t outlen)
{
  FAR struct mouse_driver_s *priv = (FAR struct mouse_driver_s *)driver;
  FAR struct usbdev_req_s *ctrlreq = priv->ctrlreq;
  FAR struct usbdev_devinfo_s *devinfo = &priv->devinfo;
  struct usb_ss_epdesc_s epdesc;
  uint16_t value;
  uint16_t index;
  uint16_t len;
  int ret = -EOPNOTSUPP;

  usbtrace(TRACE_CLASSSETUP, ctrl->req);

  /* Extract the little-endian 16-bit values to host order */

  value = GETUINT16(ctrl->value);
  index = GETUINT16(ctrl->index);
  len   = GETUINT16(ctrl->len);

  uinfo("type=%02x req=%02x value=%04x len=%04x\n",
        ctrl->type, ctrl->req, value, len);

  if ((ctrl->type & USB_REQ_TYPE_MASK) == USB_REQ_TYPE_STANDARD)
    {
      /**********************************************************************
       * Standard Requests
       **********************************************************************/

      switch (ctrl->req)
        {
        case USB_REQ_GETDESCRIPTOR:
          {
            if(ctrl->value[1] == USBHID_DESCTYPE_REPORT)
              {
                ret = sizeof(mouse_report_descriptor);
                memcpy(ctrlreq->buf, &mouse_report_descriptor, ret);
              }
          }
          break;

        case USB_REQ_SETCONFIGURATION:
          {
            /* activate ep */
            usbdev_copy_epdesc(&epdesc.epdesc, devinfo->epno[USBMOUSE_EP_INTIN_IDX],
                         priv->drvr.speed, &g_mouse_epintin);
            EP_CONFIGURE(priv->epintin, &epdesc.epdesc, true);

            work_queue(HPWORK, &priv->work, usbdev_register_driver, priv, 0);

            return 0; /* Composite driver will send the reply */
          }
          break;

        default:
          usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_UNSUPPORTEDSTDREQ),
                   ctrl->req);
          break;
        }

    }
  else if ((ctrl->type & USB_REQ_TYPE_MASK) == USB_REQ_TYPE_CLASS)
    {
      /**********************************************************************
       * HID-Specific Requests
       **********************************************************************/

      switch (ctrl->req)
        {
        case USBHID_REQUEST_SETIDLE:
          {
            return 0;
          }
        default:
          usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_UNSUPPORTEDCLASSREQ),
                   ctrl->req);
          break;
        }
    }

  /* Respond to the setup command if data was returned.  On an error return
   * value (ret < 0), the USB driver will stall.
   */

  if (ret >= 0)
    {
      /* Configure the response */

      ctrlreq->len   = (len < ret) ? len : ret;
      ctrlreq->flags = USBDEV_REQFLAGS_NULLPKT;

      ret = composite_ep0submit(driver, dev, ctrlreq, ctrl);
      if (ret < 0)
        {
          usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_EPRESPQ), (uint16_t)-ret);
          ctrlreq->result = OK;
          usbclass_ep0incomplete(dev->ep0, ctrlreq);
        }
    }

  /* Returning a negative value will cause a STALL */

  return ret;
}

/****************************************************************************
 * USB Class Driver Methods
 ****************************************************************************/

/****************************************************************************
 * Name: usbclass_bind
 *
 * Description:
 *   Invoked when the driver is bound to a USB device driver
 *
 ****************************************************************************/

static int  usbclass_bind(FAR struct usbdevclass_driver_s *driver,
                          FAR struct usbdev_s *dev)
{
  FAR struct mouse_driver_s *priv = (FAR struct mouse_driver_s *)driver;
  FAR struct hid_wrreq_s *wrcontainer;
  irqstate_t flags;
  size_t reqlen;
  int ret;
  int i;

  uinfo("enter, speed=%d\n",priv->drvr.speed);

  /* Initialize lock */

  nxmutex_init(&priv->lock);

  /* Preallocate control request */
  priv->ctrlreq = usbdev_allocreq(dev->ep0, USBMOUSE_MAXSTRLEN);
  if (priv->ctrlreq == NULL)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_ALLOCCTRLREQ), 0);
      return -ENOMEM;
    }

  priv->ctrlreq->callback = usbclass_ep0incomplete;

  /* Pre-allocate all endpoints... the endpoints will not be functional
   * until the SET CONFIGURATION request is processed.
   * This is done here because there may be calls to kmm_malloc and the SET
   * CONFIGURATION processing probably occurs within interrupt handling
   * logic where kmm_malloc calls will fail.
   */

  /* Pre-allocate the IN interrupt endpoint */

  priv->epintin = DEV_ALLOCEP(dev, HIDMOUSE_MKEPINTIN(&priv->devinfo),
                              true, USB_EP_ATTR_XFER_INT);
  if (!priv->epintin)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_EPINTINALLOCFAIL), 0);
      ret = -ENODEV;
      goto errout;
    }

  priv->epintin->priv = priv;

  /* Pre-allocate write request containers and put in a free list.  The
   * buffer size should be larger than a full build IN packet.  Otherwise,
   * we will send a bogus null packet at the end of each packet.
   *
   * Pick the larger of the max packet size and the configured request size.
   *
   * NOTE: These write requests are sized for the bulk IN endpoint but are
   * shared with interrupt IN endpoint which does not need a large buffer.
   */


    reqlen = CONFIG_USBMOUSE_EPINTIN_SIZE;

    for (i = 0; i < CONFIG_USBMOUSE_NWRREQS; i++)
    {
      wrcontainer      = &priv->wrreqs[i];
      wrcontainer->req = usbdev_allocreq(priv->epintin, reqlen);
      if (wrcontainer->req == NULL)
        {
          usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_WRALLOCREQ), -ENOMEM);
          ret = -ENOMEM;
          goto errout;
        }

      wrcontainer->req->priv     = wrcontainer;
      wrcontainer->req->callback = hidep_wrcomplete;

      flags = enter_critical_section();
      sq_addlast((FAR sq_entry_t *)wrcontainer, &priv->txfree);
      priv->nwrq++;     /* Count of write requests available */
      leave_critical_section(flags);
    }

  return OK;

errout:
  usbclass_unbind(driver, dev);
  return ret;
}

static void usbclass_unbind(FAR struct usbdevclass_driver_s *driver,
                            FAR struct usbdev_s *dev)
{
  FAR struct mouse_driver_s *priv = (FAR struct mouse_driver_s *)driver;
  FAR struct hid_wrreq_s *wrcontainer;
  irqstate_t flags;
  int i;

  if (priv->ctrlreq != NULL)
    {
      usbdev_freereq(dev->ep0, priv->ctrlreq);
      priv->ctrlreq = NULL;
    }

  /* Free write requests that are not in use (which should be all
   * of them)
   */

  flags = enter_critical_section();
  DEBUGASSERT(priv->nwrq == CONFIG_USBMOUSE_NWRREQS);

  while (!sq_empty(&priv->txfree))
    {
        wrcontainer = (struct hid_wrreq_s *)sq_remfirst(&priv->txfree);
        if (wrcontainer->req != NULL)
        {
            usbdev_freereq(priv->epintin, wrcontainer->req);
            priv->nwrq--;     /* Number of write requests queued */
        }
    }

  DEBUGASSERT(priv->nwrq == 0);
  leave_critical_section(flags);

  priv->linked = false;

  nxmutex_destroy(&priv->lock);

  if (priv->epintin)
    {
        DEV_FREEEP(dev, priv->epintin);
        priv->epintin = NULL;
    }
}

static void usbclass_disconnect(FAR struct usbdevclass_driver_s *driver,
                                FAR struct usbdev_s *dev)
{
    FAR struct mouse_driver_s *priv = (FAR struct mouse_driver_s *)driver;

    priv->linked = false;
}

/****************************************************************************
 * Name: usbclass_uninitialize
 *
 * Description:
 *   Un-initialize the USB hid class driver.  This function is used
 *   internally by the USB composite driver to uninitialize the
 *   driver.
 *
 * Input Parameters:
 *   There is one parameter, it differs in typing depending upon whether the
 *   CDC/ACM driver is an internal part of a composite device, or a
 *   standalone USB driver:
 *
 *     classdev - The class object returned by classobject()
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void usbclass_uninitialize(FAR struct usbdevclass_driver_s *classdev)
{
  FAR struct mouse_driver_s *drvr = (FAR struct mouse_driver_s *)classdev;
  int ret;

  /* Un-register the device */

  ret = unregister_driver(USBMOUSE_CHARDEV_PATH);
  if (ret < 0)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_UARTUNREGISTER),
               (uint16_t)-ret);
    }

  kmm_free(classdev);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: usbdev_mouse_get_composite_devdesc
 *
 * Description:
 *   Helper function to fill in some constants into the composite
 *   configuration struct.
 *
 * Input Parameters:
 *     dev - Pointer to the configuration struct we should fill
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void usbdev_mouse_get_composite_devdesc(FAR struct composite_devdesc_s *dev)
{
    memset(dev, 0, sizeof(struct composite_devdesc_s));

    dev->mkconfdesc          = usbclass_mkcfgdesc;
    dev->mkstrdesc           = usbclass_mkstrdesc;
    dev->classobject         = usbclass_classobject;
    dev->uninitialize        = usbclass_uninitialize;
    dev->nconfigs            = USBMOUSE_NCONFIGS;
    dev->configid            = 1;
    dev->cfgdescsize         = sizeof(g_mouse_ifdesc)
                                + sizeof(g_mouse_hiddesc)
                                + 1 * USB_SIZEOF_EPDESC;
    dev->devinfo.ninterfaces = 1;
    dev->devinfo.nstrings    = USBMOUSE_NSTRIDS;
    dev->devinfo.nendpoints  = USBMOUSE_NUM_EPS;
    dev->devinfo.epinfos     = g_mouse_epinfos;
    dev->devinfo.name        = USBMOUSE_CHARDEV_PATH;
}
