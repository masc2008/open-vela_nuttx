/****************************************************************************
 * drivers/usbdev/rndis.c
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

/* References:
 *   [MS-RNDIS]:
 *     Remote Network Driver Interface Specification (RNDIS) Protocol
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <assert.h>
#include <debug.h>
#include <errno.h>
#include <inttypes.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>

#include <sys/param.h>

#include <nuttx/queue.h>
#include <nuttx/net/net.h>
#include <nuttx/net/netdev.h>
#include <nuttx/kmalloc.h>
#include <nuttx/arch.h>
#include <nuttx/usb/usb.h>
#include <nuttx/usb/cdc.h>
#include <nuttx/usb/usbdev.h>
#include <nuttx/usb/usbdev_trace.h>
#include <nuttx/usb/rndis.h>
#include <nuttx/mutex.h>
#include <nuttx/wqueue.h>

#ifdef CONFIG_BOARD_USBDEV_SERIALSTR
#include <nuttx/board.h>
#endif

#include "rndis_std.h"

#ifdef CONFIG_RNDIS_COMPOSITE
#  include <nuttx/usb/composite.h>
#endif

/****************************************************************************
 * Pre-processor definitions
 ****************************************************************************/

#define RNDIS_MKEPINTIN(desc)     (USB_DIR_IN | (desc)->epno[RNDIS_EP_INTIN_IDX])
#define RNDIS_EPINTIN_ATTR        (USB_EP_ATTR_XFER_INT)

#define RNDIS_MKEPBULKIN(desc)    (USB_DIR_IN | (desc)->epno[RNDIS_EP_BULKIN_IDX])
#define RNDIS_EPOUTBULK_ATTR      (USB_EP_ATTR_XFER_BULK)

#define RNDIS_MKEPBULKOUT(desc)   ((desc)->epno[RNDIS_EP_BULKOUT_IDX])
#define RNDIS_EPINBULK_ATTR       (USB_EP_ATTR_XFER_BULK)

#define CONFIG_RNDIS_EP0MAXPACKET 64

#ifndef CONFIG_RNDIS_NWRREQS
#  define CONFIG_RNDIS_NWRREQS  (2)
#endif

#define RNDIS_PACKET_HDR_SIZE   (sizeof(struct rndis_packet_msg))
#define CONFIG_RNDIS_BULKIN_REQLEN \
  (CONFIG_NET_ETH_PKTSIZE + CONFIG_NET_GUARDSIZE + RNDIS_PACKET_HDR_SIZE)
#define CONFIG_RNDIS_BULKOUT_REQLEN CONFIG_RNDIS_BULKIN_REQLEN
#define RNDIS_RECIVE_LEN (CONFIG_RNDIS_BULKIN_REQLEN - RNDIS_PACKET_HDR_SIZE)

static_assert(CONFIG_NET_LL_GUARDSIZE >= RNDIS_PACKET_HDR_SIZE + ETH_HDRLEN,
             "CONFIG_NET_LL_GUARDSIZE cannot be less than ETH_HDRLEN"
             " + RNDIS_PACKET_HDR_SIZE");

static_assert((CONFIG_NET_LL_GUARDSIZE % 4) == 2,
             "CONFIG_NET_LL_GUARDSIZE - ETH_HDRLEN "
             "should be aligned to 4 bytes");

#define RNDIS_NCONFIGS          (1)
#define RNDIS_CONFIGID          (1)
#define RNDIS_CONFIGIDNONE      (0)
#define RNDIS_NINTERFACES       (2)
#define RNDIS_NSTRIDS           (0)

#ifndef CONFIG_RNDIS_COMPOSITE
#  define RNDIS_EPINTIN_ADDR    USB_EPIN(CONFIG_RNDIS_EPINTIN)
#  define RNDIS_EPBULKIN_ADDR   USB_EPIN(CONFIG_RNDIS_EPBULKIN)
#  define RNDIS_EPBULKOUT_ADDR  USB_EPOUT(CONFIG_RNDIS_EPBULKOUT)
#endif
#define RNDIS_NUM_EPS           (3)

#define RNDIS_MANUFACTURERSTRID (1)
#define RNDIS_PRODUCTSTRID      (2)
#define RNDIS_SERIALSTRID       (3)

#define RNDIS_STR_LANGUAGE      (0x0409) /* en-us */

#define RNDIS_MXDESCLEN         (128)
#define RNDIS_MAXSTRLEN         (RNDIS_MXDESCLEN-2)
#define RNDIS_CTRLREQ_LEN       (256)
#define RNDIS_RESP_QUEUE_WORDS  (64)

#define RNDIS_BUFFER_SIZE       CONFIG_NET_ETH_PKTSIZE
#define RNDIS_BUFFER_COUNT      4

/* Work queue to use for network operations. LPWORK should be used here */

#define ETHWORK                 HPWORK

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Container to support a list of requests */

struct rndis_req_s
{
  FAR struct rndis_req_s  *flink;  /* Implements a singly linked list */
  FAR struct usbdev_req_s *req;    /* The contained request */
  FAR struct iob_s        *iob;    /* IOB offload */
  FAR uint8_t             *buf;    /* Use malloc buffer when config IOB_LEN < CONFIG_RNDIS_BULKIN_REQLEN */
};

struct rndis_netpkt_s
{
  FAR struct rndis_netpkt_s *flink;  /* Implements a singly linked list */
  FAR uint8_t               buf[CONFIG_RNDIS_BULKOUT_REQLEN];
  FAR size_t                datasize;
};

/* This structure describes the internal state of the driver */

struct rndis_dev_s
{
  struct net_driver_s      netdev;       /* Network driver structure */
  struct usbdev_devinfo_s  devinfo;

  FAR struct usbdev_s     *usbdev;       /* usbdev driver pointer */
  FAR struct usbdev_ep_s  *epintin;      /* Interrupt IN endpoint structure */
  FAR struct usbdev_ep_s  *epbulkin;     /* Bulk IN endpoint structure */
  FAR struct usbdev_ep_s  *epbulkout;    /* Bulk OUT endpoint structure */
  FAR struct usbdev_req_s *ctrlreq;      /* Pointer to preallocated control request */
  FAR struct usbdev_req_s *epintin_req;  /* Pointer to preallocated interrupt in endpoint request */
  FAR struct usbdev_req_s *rdreq;        /* Pointer to Preallocated control endpoint read request */
  struct sq_queue_s reqlist;             /* List of free write request containers */

  /* Preallocated USB request buffers */

  struct rndis_req_s wrreqs[CONFIG_RNDIS_NWRREQS];

  struct work_s rxwork;                  /* Worker for dispatching RX packets */
  struct work_s pollwork;                /* TX poll worker */

  uint8_t config;                        /* USB Configuration number */
  FAR struct rndis_netpkt_s *net_packet; /* Pointer netpacket container that holds payload */
  FAR struct rndis_req_s *net_req;       /* Pointer to request whose buffer is assigned to network */
  FAR struct rndis_req_s *dummy_req;     /* Pointer to request whose buffer is assigned to network */
  FAR struct rndis_req_s *rx_req;        /* Pointer request container that holds RX buffer */
  size_t current_rx_received;            /* Number of bytes of current RX datagram received over USB */
  size_t current_rx_datagram_size;       /* Total number of bytes of the current RX datagram */
  size_t current_rx_datagram_offset;     /* Offset of current RX datagram */
  size_t current_rx_msglen;              /* Length of the entire message to be received */
  bool rdreq_submitted;                  /* Indicates if the read request is submitted */
  bool rx_blocked;                       /* Indicates if we can receive packets on bulk in endpoint */
  bool connected;                        /* Connection status indicator */
  uint32_t rndis_packet_filter;          /* RNDIS packet filter value */
  uint32_t rndis_host_tx_count;          /* TX packet counter */
  uint32_t rndis_host_rx_count;          /* RX packet counter */
  uint8_t host_mac_address[6];           /* Host side MAC address */
  mutex_t lock;                          /* Mutex to protect the driver state */

  size_t response_queue_words;           /* Count of words waiting in response_queue. */
  uint32_t response_queue[RNDIS_RESP_QUEUE_WORDS];
};

/* The internal version of the class driver */

struct rndis_driver_s
{
  struct usbdevclass_driver_s drvr;
  FAR struct rndis_dev_s      *dev;
};

/* This is what is allocated */

struct rndis_alloc_s
{
  struct rndis_dev_s    dev;
  struct rndis_driver_s drvr;
};

/* RNDIS USB configuration descriptor */

struct rndis_cfgdesc_s
{
#ifndef CONFIG_RNDIS_COMPOSITE
  struct usb_cfgdesc_s cfgdesc;        /* Configuration descriptor */
#elif defined(CONFIG_COMPOSITE_IAD)
  struct usb_iaddesc_s assoc_desc;     /* Interface association descriptor */
#endif
  struct usb_ifdesc_s  comm_ifdesc;    /* Communication interface descriptor */
  struct usb_epdesc_s  epintindesc;    /* Interrupt endpoint descriptor */
  struct usb_ifdesc_s  data_ifdesc;    /* Data interface descriptor */
  struct usb_epdesc_s  epbulkindesc;   /* Bulk in interface descriptor */
  struct usb_epdesc_s  epbulkoutdesc;  /* Bulk out interface descriptor */
};

/* RNDIS object ID - value pair structure */

struct rndis_oid_value_s
{
  uint32_t objid;
  uint32_t length;
  uint32_t value;
  FAR const void *data;                /* Data pointer overrides value if non-NULL. */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Netdev driver callbacks */

static int rndis_ifup(FAR struct net_driver_s *dev);
static int rndis_ifdown(FAR struct net_driver_s *dev);
static int rndis_txavail(FAR struct net_driver_s *dev);
static int rndis_transmit(FAR struct rndis_dev_s *priv);
static int rndis_txpoll(FAR struct net_driver_s *dev);

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
static int  usbclass_setconfig(FAR struct rndis_dev_s *priv, uint8_t config);
static void usbclass_resetconfig(FAR struct rndis_dev_s *priv);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* USB driver operations */

const static struct usbdevclass_driverops_s g_driverops =
{
  &usbclass_bind,
  &usbclass_unbind,
  &usbclass_setup,
  &usbclass_disconnect,
  NULL,
  NULL
};

#ifndef CONFIG_RNDIS_COMPOSITE
static const struct usb_devdesc_s g_devdesc =
{
  USB_SIZEOF_DEVDESC,                           /* len */
  USB_DESC_TYPE_DEVICE,                         /* type */
  {LSBYTE(0x0200), MSBYTE(0x0200)},             /* usb */
  0,                                            /* classid */
  0,                                            /* subclass */
  0,                                            /* protocol */
  CONFIG_RNDIS_EP0MAXPACKET,                    /* maxpacketsize */
  { LSBYTE(CONFIG_RNDIS_VENDORID),              /* vendor */
    MSBYTE(CONFIG_RNDIS_VENDORID) },
  { LSBYTE(CONFIG_RNDIS_PRODUCTID),             /* product */
    MSBYTE(CONFIG_RNDIS_PRODUCTID) },
  { LSBYTE(CONFIG_RNDIS_VERSIONNO),             /* device */
    MSBYTE(CONFIG_RNDIS_VERSIONNO) },
  RNDIS_MANUFACTURERSTRID,                      /* imfgr */
  RNDIS_PRODUCTSTRID,                           /* iproduct */
  RNDIS_SERIALSTRID,                            /* serno */
  RNDIS_NCONFIGS                                /* nconfigs */
};
#endif

#ifndef CONFIG_RNDIS_COMPOSITE

  /* Configuration descriptor */

static const struct usb_cfgdesc_s g_rndis_cfgdesc =
{
  .len          = USB_SIZEOF_CFGDESC,
  .type         = USB_DESC_TYPE_CONFIG,
  .totallen     =
  {
    0, 0
  },
  .ninterfaces  = RNDIS_NINTERFACES,
  .cfgvalue     = RNDIS_CONFIGID,
  .icfg         = 0,
  .attr         = USB_CONFIG_ATTR_ONE | USB_CONFIG_ATTR_SELFPOWER,
  .mxpower      = (CONFIG_USBDEV_MAXPOWER + 1) / 2
};
#elif defined(CONFIG_COMPOSITE_IAD)

  /* Interface association descriptor */

static const struct usb_iaddesc_s g_rndis_assoc_desc =
{
  .len          = USB_SIZEOF_IADDESC,
  .type         = USB_DESC_TYPE_INTERFACEASSOCIATION,
  .firstif      = 0,
  .nifs         = RNDIS_NINTERFACES,
  .classid      = 0xef,
  .subclass     = 0x04,
  .protocol     = 0x01,
  .ifunction    = 0
};
#endif

  /* Communication interface descriptor */

static const struct usb_ifdesc_s g_rndis_comm_ifdesc =
{
  .len          = USB_SIZEOF_IFDESC,
  .type         = USB_DESC_TYPE_INTERFACE,
  .ifno         = 0,
  .alt          = 0,
  .neps         = 1,
  .classid      = USB_CLASS_CDC,
  .subclass     = CDC_SUBCLASS_ACM,
  .protocol     = CDC_PROTO_VENDOR,
  .iif          = 0
};

  /* Interrupt endpoint descriptor */

static const struct usbdev_epinfo_s g_rndis_epintindesc =
{
  .desc =
    {
      .len       = USB_SIZEOF_EPDESC,
      .type      = USB_DESC_TYPE_ENDPOINT,
#ifndef CONFIG_RNDIS_COMPOSITE
      .addr      = RNDIS_EPINTIN_ADDR,
#else
      .addr      = USB_DIR_IN,
#endif
      .attr      = USB_EP_ATTR_XFER_INT,
      .interval  = 10
    },
  .reqnum        = 1,
  .fssize        = CONFIG_RNDIS_EPINTIN_FSSIZE,
#ifdef CONFIG_USBDEV_DUALSPEED
  .hssize        = CONFIG_RNDIS_EPINTIN_HSSIZE,
#endif
#ifdef CONFIG_USBDEV_SUPERSPEED
  .sssize        = CONFIG_RNDIS_EPINTIN_HSSIZE,
  .compdesc      =
    {
      .len       = USB_SIZEOF_SS_EPCOMPDESC,
      .type      = USB_DESC_TYPE_ENDPOINT_COMPANION,
      .mxburst   = CONFIG_RNDIS_EPINTIN_MAXBURST,
      .attr      = 0,
      .wbytes[0] = LSBYTE((CONFIG_RNDIS_EPINTIN_MAXBURST + 1) *
                           CONFIG_RNDIS_EPINTIN_HSSIZE),
      .wbytes[1] = MSBYTE((CONFIG_RNDIS_EPINTIN_MAXBURST + 1) *
                           CONFIG_RNDIS_EPINTIN_HSSIZE),
    },
#endif
};

  /* Data interface descriptor */

static const struct usb_ifdesc_s g_rndis_data_ifdesc =
{
  .len          = USB_SIZEOF_IFDESC,
  .type         = USB_DESC_TYPE_INTERFACE,
  .ifno         = 1,
  .alt          = 0,
  .neps         = 2,
  .classid      = USB_CLASS_CDC_DATA,
  .subclass     = 0,
  .protocol     = 0,
  .iif          = 0
};

  /* Bulk in interface descriptor */

static const struct usbdev_epinfo_s g_rndis_epbulkindesc =
{
  .desc =
    {
      .len       = USB_SIZEOF_EPDESC,
      .type      = USB_DESC_TYPE_ENDPOINT,
#ifndef CONFIG_RNDIS_COMPOSITE
      .addr      = RNDIS_EPBULKIN_ADDR,
#else
      .addr      = USB_DIR_IN,
#endif
      .attr      = USB_EP_ATTR_XFER_BULK,
#ifdef CONFIG_USBDEV_DUALSPEED
      .interval  = 0
#else
      .interval  = 1
#endif
    },
  .reqnum        = 1,
  .fssize        = CONFIG_RNDIS_EPBULKIN_FSSIZE,
#ifdef CONFIG_USBDEV_DUALSPEED
  .hssize        = CONFIG_RNDIS_EPBULKIN_HSSIZE,
#endif
#ifdef CONFIG_USBDEV_SUPERSPEED
  .sssize        = CONFIG_RNDIS_EPBULKIN_SSSIZE,
  .compdesc      =
    {
      .len       = USB_SIZEOF_SS_EPCOMPDESC,
      .type      = USB_DESC_TYPE_ENDPOINT_COMPANION,
      .mxburst   = CONFIG_RNDIS_EPBULKIN_MAXBURST,
      .attr      = CONFIG_RNDIS_EPBULKIN_MAXSTREAM,
      .wbytes[0] = 0,
      .wbytes[1] = 0,
    },
#endif
};

  /* Bulk out interface descriptor */

static const struct usbdev_epinfo_s g_rndis_epbulkoutdesc =
{
  .desc =
    {
      .len       = USB_SIZEOF_EPDESC,
      .type      = USB_DESC_TYPE_ENDPOINT,
#ifndef CONFIG_RNDIS_COMPOSITE
      .addr      = RNDIS_EPBULKOUT_ADDR,
#else
      .addr      = USB_DIR_OUT,
#endif
      .attr      = USB_EP_ATTR_XFER_BULK,
#ifdef CONFIG_USBDEV_DUALSPEED
      .interval  = 0
#else
      .interval  = 1
#endif
    },
  .reqnum        = 1,
  .fssize        = CONFIG_RNDIS_EPBULKOUT_FSSIZE,
#ifdef CONFIG_USBDEV_DUALSPEED
  .hssize        = CONFIG_RNDIS_EPBULKOUT_HSSIZE,
#endif
#ifdef CONFIG_USBDEV_SUPERSPEED
  .sssize        = CONFIG_RNDIS_EPBULKOUT_SSSIZE,
  .compdesc      =
    {
      .len       = USB_SIZEOF_SS_EPCOMPDESC,
      .type      = USB_DESC_TYPE_ENDPOINT_COMPANION,
      .mxburst   = CONFIG_RNDIS_EPBULKOUT_MAXBURST,
      .attr      = CONFIG_RNDIS_EPBULKOUT_MAXSTREAM,
      .wbytes[0] = 0,
      .wbytes[1] = 0,
    },
#endif
};

/* Default MAC address given to the host side of the interface. */

static uint8_t g_rndis_default_mac_addr[6] =
{
  0x00, 0x80, 0x43, 0x76, 0x6A, 0xDD
};

static uint8_t g_rndis_dummy_router_mac_addr[6] =
{
  0xd8, 0xef, 0x42, 0xae, 0x07, 0x49
};

#ifndef min
#define min(a,b) (((a)<(b))?(a):(b))
#endif

static bool rndis_inited           = false;
static bool rndis_conn_stat_cur    = false;
static bool rndis_conn_stat_expect = false;
static struct net_driver_s  *g_rndis_netdev = NULL;

static struct net_driver_s  *g_media_netdev = NULL;
extern FAR struct dhcpd_state_s g_ds_data;

struct rndis_netpkt_s rndis_netpackets[CONFIG_RNDIS_NWRREQS];
struct sq_queue_s rndis_free_netpkt_lst;     /* List of free netpackets containers */
struct sq_queue_s rndis_pending_netpkt_lst;  /* List of pending netpackets containers */

#define MEDIA_LL_TYPE       ((!g_media_netdev) ? NET_LL_IEEE80211 : g_media_netdev->d_lltype)
#define MEDIA_LL_HDRLEN     ((!g_media_netdev) ? ETH_HDRLEN : NET_LL_HDRLEN(g_media_netdev))

/* These lists give dummy responses to be returned to PC. The values are
 * chosen so that Windows is happy - other operating systems don't really
 * care much.
 */

static const uint32_t g_rndis_supported_oids[] =
{
  RNDIS_OID_GEN_SUPPORTED_LIST,
  RNDIS_OID_GEN_HARDWARE_STATUS,
  RNDIS_OID_GEN_MEDIA_SUPPORTED,
  RNDIS_OID_GEN_MEDIA_IN_USE,
  RNDIS_OID_GEN_MAXIMUM_FRAME_SIZE,
  RNDIS_OID_GEN_LINK_SPEED,
  RNDIS_OID_GEN_TRANSMIT_BLOCK_SIZE,
  RNDIS_OID_GEN_RECEIVE_BLOCK_SIZE,
  RNDIS_OID_GEN_VENDOR_ID,
  RNDIS_OID_GEN_VENDOR_DESCRIPTION,
  RNDIS_OID_GEN_VENDOR_DRIVER_VERSION,
  RNDIS_OID_GEN_CURRENT_PACKET_FILTER,
  RNDIS_OID_GEN_MAXIMUM_TOTAL_SIZE,
  RNDIS_OID_GEN_MEDIA_CONNECT_STATUS,
  RNDIS_OID_GEN_PHYSICAL_MEDIUM,
  RNDIS_OID_GEN_XMIT_OK,
  RNDIS_OID_GEN_RCV_OK,
  RNDIS_OID_GEN_XMIT_ERROR,
  RNDIS_OID_GEN_RCV_ERROR,
  RNDIS_OID_GEN_RCV_NO_BUFFER,
  RNDIS_OID_802_3_PERMANENT_ADDRESS,
  RNDIS_OID_802_3_CURRENT_ADDRESS,
  RNDIS_OID_802_3_MULTICAST_LIST,
  RNDIS_OID_802_3_MAC_OPTIONS,
  RNDIS_OID_802_3_MAXIMUM_LIST_SIZE,
  RNDIS_OID_802_3_RCV_ERROR_ALIGNMENT,
  RNDIS_OID_802_3_XMIT_ONE_COLLISION,
  RNDIS_OID_802_3_XMIT_MORE_COLLISION,
};

static const struct rndis_oid_value_s g_rndis_oid_values[] =
{
  {
    RNDIS_OID_GEN_SUPPORTED_LIST,
    sizeof(g_rndis_supported_oids), 0,
    g_rndis_supported_oids
  },
  {RNDIS_OID_GEN_MAXIMUM_FRAME_SIZE,    4, 1500,  NULL},
#if defined(CONFIG_USBDEV_DUALSPEED) || defined(CONFIG_USBDEV_SUPERSPEED)
  {RNDIS_OID_GEN_LINK_SPEED,            4, 100000,              NULL},
#else
  {RNDIS_OID_GEN_LINK_SPEED,            4, 2000000,             NULL},
#endif
  {RNDIS_OID_GEN_TRANSMIT_BLOCK_SIZE,   4, CONFIG_NET_ETH_PKTSIZE,  NULL},
  {RNDIS_OID_GEN_RECEIVE_BLOCK_SIZE,    4, CONFIG_NET_ETH_PKTSIZE,  NULL},
  {RNDIS_OID_GEN_VENDOR_ID,             4, 0x00ffffff,          NULL},
  {RNDIS_OID_GEN_VENDOR_DESCRIPTION,    6, 0,                   "RNDIS"},
  {RNDIS_OID_GEN_CURRENT_PACKET_FILTER, 4, 0,                   NULL},
  {RNDIS_OID_GEN_MAXIMUM_TOTAL_SIZE,    4, 2048,                NULL},
  {RNDIS_OID_GEN_XMIT_OK,               4, 0,                   NULL},
  {RNDIS_OID_GEN_RCV_OK,                4, 0,                   NULL},
  {RNDIS_OID_802_3_PERMANENT_ADDRESS,   6, 0,                   NULL},
  {RNDIS_OID_802_3_CURRENT_ADDRESS,     6, 0,                   NULL},
  {RNDIS_OID_802_3_MULTICAST_LIST,      4, 0xe0000000,          NULL},
  {RNDIS_OID_802_3_MAXIMUM_LIST_SIZE,   4, 1,                   NULL},
  {0x0,                                 4, 0,                   NULL}, /* Default fallback */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Buffering of data is implemented in the following manner:
 *
 * RNDIS driver holds a number of preallocated bulk IN endpoint write
 * requests along with buffers large enough to hold an Ethernet packet and
 * the corresponding RNDIS header.
 *
 * One of these is always reserved for packet reception - when data arrives
 * on the bulk OUT endpoint, it is copied to the reserved request buffer.
 * When the reception of an Ethernet packet is complete, a worker to process
 * the packet is scheduled and bulk OUT endpoint is set to NAK.
 *
 * The processing worker passes the buffer to the network. When the network
 * is done processing the packet, the buffer might contain data to be sent.
 * If so, the corresponding write request is queued on the bulk IN endpoint.
 * The NAK state on bulk OUT endpoint is cleared to allow new packets to
 * arrive. If there's no data to send, the request is returned to the list of
 * free requests.
 *
 * When a bulk IN write operation is complete, the request is added to the
 * list of free requests.
 *
 ****************************************************************************/

/****************************************************************************
 * Name: rndis_submit_rdreq
 *
 * Description:
 *   Submits the bulk OUT read request. Takes care not to submit the request
 *   when the RX packet buffer is already in use.
 *
 * Input Parameters:
 *   priv: pointer to RNDIS device driver structure
 *
 * Returned Value:
 *   The return value of the EP_SUBMIT operation
 *
 ****************************************************************************/

static int rndis_submit_rdreq(FAR struct rndis_dev_s *priv)
{
  int ret = OK;

  uinfo("rndis_debug enter, rdreq_submitted:%d rx_blocked:%d\n", priv->rdreq_submitted, priv->rx_blocked);
  if (!priv->rdreq_submitted && !priv->rx_blocked)
    {
      priv->rdreq->len = priv->epbulkout->maxpacket;
      ret = EP_SUBMIT(priv->epbulkout, priv->rdreq);
      if (ret != OK)
        {
          usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_RDSUBMIT),
                   (uint16_t)-priv->rdreq->result);
        }
      else
        {
          priv->rdreq_submitted = true;
        }
    }

  return ret;
}

/****************************************************************************
 * Name: rndis_cancel_rdreq
 *
 * Description:
 *   Cancels the bulk OUT endpoint read request.
 *
 * Input Parameters:
 *   priv: pointer to RNDIS device driver structure
 *
 ****************************************************************************/

static void rndis_cancel_rdreq(FAR struct rndis_dev_s *priv)
{
  if (priv->rdreq_submitted)
    {
      EP_CANCEL(priv->epbulkout, priv->rdreq);
      priv->rdreq_submitted = false;
    }
}

/****************************************************************************
 * Name: rndis_block_rx
 *
 * Description:
 *   Blocks reception of further bulk OUT endpoint data.
 *
 * Input Parameters:
 *   priv: pointer to RNDIS device driver structure
 *
 ****************************************************************************/

static void rndis_block_rx(FAR struct rndis_dev_s *priv)
{
  priv->rx_blocked = true;
  rndis_cancel_rdreq(priv);
}

/****************************************************************************
 * Name: rndis_unblock_rx
 *
 * Description:
 *   Unblocks reception of bulk OUT endpoint data.
 *
 * Input Parameters:
 *   priv: pointer to RNDIS device driver structure
 *
 * Assumptions:
 *   Called from critical section
 *
 ****************************************************************************/

static void rndis_unblock_rx(FAR struct rndis_dev_s *priv)
{
  priv->rx_blocked = false;
}

static FAR bool rndis_allocnetpkts(FAR struct rndis_dev_s *priv)
{
  irqstate_t flags = enter_critical_section();
  if (priv->net_packet != NULL)
  {
    leave_critical_section(flags);
    return true;
  }

  priv->net_packet = (FAR struct rndis_req_s *)sq_remfirst(&rndis_free_netpkt_lst);
  leave_critical_section(flags);
  return priv->net_packet != NULL;
}

static void rndis_freenetpkts(FAR struct rndis_dev_s *priv, FAR struct rndis_netpkt_s *netpkt)
{
  irqstate_t flags = enter_critical_section();
  DEBUGASSERT(netpkt != NULL);
  sq_addlast((FAR sq_entry_t *)netpkt, &rndis_free_netpkt_lst);
  leave_critical_section(flags);
}

static bool rndis_hasfreenetpkts(FAR struct rndis_dev_s *priv)
{
  return sq_count(&rndis_free_netpkt_lst) > 1;
}

static void rndis_addpendingnetpkts(FAR struct rndis_dev_s *priv)
{
  irqstate_t flags = enter_critical_section();
  DEBUGASSERT(priv->net_packet != NULL);
  sq_addlast((FAR sq_entry_t *)priv->net_packet, &rndis_pending_netpkt_lst);
  priv->net_packet = NULL;
  leave_critical_section(flags);
}

static FAR struct rndis_req_s *rndis_remfirstpendingnetpkts(FAR struct rndis_dev_s *priv)
{
  FAR struct rndis_netpkt_s *netpkt = NULL;
  irqstate_t flags = enter_critical_section();
  netpkt = (FAR struct rndis_req_s *)sq_remfirst(&rndis_pending_netpkt_lst);
  leave_critical_section(flags);
  return netpkt;
}

static bool rndis_haspendingnetpkts(FAR struct rndis_dev_s *priv)
{
  return sq_count(&rndis_pending_netpkt_lst) > 1;
}

/****************************************************************************
 * Name: rndis_allocwrreq
 *
 * Description:
 *   Allocates a bulk IN endpoint request from the list of free request
 *   buffers.
 *
 * Input Parameters:
 *   priv: pointer to RNDIS device driver structure
 *
 * Returned Value:
 *   NULL if allocation failed; pointer to allocated request if succeeded
 *
 * Assumptions:
 *   Called from critical section
 *
 ****************************************************************************/

static FAR struct rndis_req_s *rndis_allocwrreq(FAR struct rndis_dev_s *priv)
{
  return (FAR struct rndis_req_s *)sq_remfirst(&priv->reqlist);
}

/****************************************************************************
 * Name: rndis_hasfreereqs
 *
 * Description:
 *   Checks if there are free requests usable for TX data.
 *
 * Input Parameters:
 *   priv: pointer to RNDIS device driver structure
 *
 * Returned Value:
 *   true if requests available; false if no requests available
 *
 * Assumptions:
 *   Called from critical section
 *
 ****************************************************************************/

static bool rndis_hasfreereqs(FAR struct rndis_dev_s *priv)
{
  return sq_count(&priv->reqlist) > 1;
}

/****************************************************************************
 * Name: rndis_freewrreq
 *
 * Description:
 *   Returns a bulk IN endpoint write requests to the list of free requests.
 *
 * Input Parameters:
 *   priv: pointer to RNDIS device driver structure
 *   req: pointer to the request
 *
 * Assumptions:
 *   Called with interrupts disabled.
 *
 ****************************************************************************/

static void rndis_freewrreq(FAR struct rndis_dev_s *priv,
                            FAR struct rndis_req_s *req)
{
  DEBUGASSERT(req != NULL);
  // if (req->iob)
  //   {
  //     /* In ep submit case, need release iob chain when write complete */

  //     iob_free_chain(req->iob);
  //     req->iob = NULL;
  //   }

  sq_addlast((FAR sq_entry_t *)req, &priv->reqlist);
  // rndis_submit_rdreq(priv);
}

/****************************************************************************
 * Name: rndis_allocnetreq
 *
 * Description:
 *   Allocates a request buffer to be used on the network.
 *
 * Input Parameters:
 *   priv: pointer to RNDIS device driver structure
 *
 * Returned Value:
 *   true if succeeded; false if failed
 *
 * Assumptions:
 *   Caller holds the network lock
 *
 ****************************************************************************/

static bool rndis_allocnetreq(FAR struct rndis_dev_s *priv)
{
  while (nxmutex_lock(&priv->lock) < 0);

  DEBUGASSERT(priv->net_req == NULL);

  if (!rndis_hasfreereqs(priv))
    {
      uerr("%d, have no free req", __LINE__);
      nxmutex_unlock(&priv->lock);
      return false;
    }

  priv->net_req = rndis_allocwrreq(priv);

  nxmutex_unlock(&priv->lock);
  return priv->net_req != NULL;
}

static bool rndis_allocdummyreq(FAR struct rndis_dev_s *priv)
{
  while (priv->dummy_req != NULL) {
      uinfo("%d, priv->net_req != NULL, priv->net_req:%p", __LINE__, priv->dummy_req);
      osDelay(1);
  }

  irqstate_t flags = enter_critical_section();
  DEBUGASSERT(priv->dummy_req == NULL);

  if (!rndis_hasfreereqs(priv))
    {
      uerr("%d, have no free req", __LINE__);
      leave_critical_section(flags);
      return false;
    }

  priv->dummy_req = rndis_allocwrreq(priv);

  leave_critical_section(flags);
  return priv->dummy_req != NULL;
}

/****************************************************************************
 * Name: rndis_sendnetreq
 *
 * Description:
 *   Submits the request buffer held by the network.
 *
 * Input Parameters:
 *   priv: pointer to RNDIS device driver structure
 *
 * Assumptions:
 *   Caller holds the network lock
 *
 ****************************************************************************/

static void rndis_sendnetreq(FAR struct rndis_dev_s *priv)
{
  while (nxmutex_lock(&priv->lock) < 0);

  DEBUGASSERT(priv->net_req != NULL);

  priv->net_req->req->priv = priv->net_req;
  uinfo("rndis_debug EP_SUBMIT");
  EP_SUBMIT(priv->epbulkin, priv->net_req->req);

  priv->net_req            = NULL;
  nxmutex_unlock(&priv->lock);
}

/****************************************************************************
 * Name: rndis_freenetreq
 *
 * Description:
 *   Frees the request buffer held by the network.
 *
 * Input Parameters:
 *   priv: pointer to RNDIS device driver structure
 *
 * Assumptions:
 *   Caller holds the network lock
 *
 ****************************************************************************/

static void rndis_freenetreq(FAR struct rndis_dev_s *priv)
{
  while (nxmutex_lock(&priv->lock) < 0);

  uinfo("rndis_debug enter");
  rndis_freewrreq(priv, priv->net_req);
  priv->net_req = NULL;

  nxmutex_unlock(&priv->lock);
}

/****************************************************************************
 * Name: rndis_iob2buf
 *
 * Description:
 *   Map the appropriate location of req iob to buf.
 *
 * Input Parameters:
 *   priv: pointer to RNDIS device driver structure
 *   req: the request whose buffer we should fill
 * Assumptions:
 *   Caller holds the network lock
 *
 ****************************************************************************/

static void rndis_iob2buf(FAR struct rndis_dev_s *priv,
                          FAR struct rndis_req_s *req)
{
  uint16_t llhdrlen = ETH_HDRLEN;
  uint32_t offset   = CONFIG_NET_LL_GUARDSIZE - llhdrlen -
                      RNDIS_PACKET_HDR_SIZE;

  /*  ----------------------------------------------------------------
   *  |<--- CONFIG_NET_LL_GUARDSIZE ---->|<-- io_len/io_pktlen(0) -->|
   *  ---------------------------------------------------------------|
   *  |unused | rndis hdr size |llhdrlen |<-- io_len/io_pktlen(0) -->|
   *  ---------------------------------------------------------------|
   *  |unused | req->buf(0)                                          |
   *  ---------------------------------------------------------------|
   */

  if (req->iob->io_flink == NULL)
    {
      req->req->buf = &req->iob->io_data[offset];
      req->req->len = CONFIG_RNDIS_BULKIN_REQLEN;
    }
  else
    {
      req->req->buf = req->buf;
      iob_copyout(&req->req->buf[RNDIS_PACKET_HDR_SIZE], req->iob,
                  req->iob->io_pktlen + llhdrlen, -llhdrlen);
      // iob_free_chain(req->iob);
      // req->iob = NULL;
    }
}

/****************************************************************************
 * Name: rndis_allocrxreq
 *
 * Description:
 *   Allocates a buffer for packet reception if there already isn't one.
 *
 * Input Parameters:
 *   priv: pointer to RNDIS device driver structure
 *
 * Returned Value:
 *   true if succeeded; false if failed
 *
 * Assumptions:
 *   Called from critical section
 *
 ****************************************************************************/

static bool rndis_allocrxreq(FAR struct rndis_dev_s *priv)
{
  // FAR struct iob_s *iob;

  if (priv->rx_req != NULL)
    {
      return true;
    }

  /* Prepare buffer to receivce data from usb driver */

  // iob = iob_tryalloc(false);
  // if (iob == NULL)
  //   {
  //     return false;
  //   }

  // iob_reserve(iob, CONFIG_NET_LL_GUARDSIZE);

  if ((priv->rx_req = rndis_allocwrreq(priv)) == NULL)
    {
      // iob_free_chain(iob);
      return false;
    }

  // priv->rx_req->iob = iob;
  // rndis_iob2buf(priv, priv->rx_req);

  return true;
}

/****************************************************************************
 * Name: rndis_giverxreq
 *
 * Description:
 *   Passes the RX packet buffer to the network
 *
 * Input Parameters:
 *   priv: pointer to RNDIS device driver structure
 *
 * Assumptions:
 *   Caller holds the network lock
 *
 ****************************************************************************/

static void rndis_giverxreq(FAR struct rndis_dev_s *priv)
{
  DEBUGASSERT(priv->rx_req != NULL);
  DEBUGASSERT(priv->net_req == NULL);

  priv->net_req = priv->rx_req;
  priv->rx_req  = NULL;

  /* Move iob from net_req to netdev */

  netdev_iob_release(&priv->netdev);

  priv->netdev.d_iob = priv->net_req->iob;
  priv->netdev.d_len = priv->net_req->iob->io_pktlen;
  priv->net_req->iob = NULL;
}

/****************************************************************************
 * Name: rndis_fillrequest
 *
 * Description:
 *   Fills the RNDIS header to the request buffer
 *
 * Input Parameters:
 *   priv: pointer to RNDIS device driver structure
 *   req: the request whose buffer we should fill
 *
 * Returned Value:
 *   The total length of the request data
 *
 * Assumptions:
 *   Caller holds the network lock
 *
 ****************************************************************************/

static uint16_t rndis_fillrequest(FAR struct rndis_dev_s *priv,
                                  FAR struct rndis_req_s *req)
{
  size_t datalen;

  req->req->len = 0;

  datalen = MIN(priv->netdev.d_len,
                CONFIG_RNDIS_BULKIN_REQLEN - RNDIS_PACKET_HDR_SIZE);
  if (datalen > 0)
    {
      /* Move iob from netdev to net_req and send the required headers */

      req->iob = priv->netdev.d_iob;
      netdev_iob_clear(&priv->netdev);
      // rndis_iob2buf(priv, req);
      FAR struct rndis_packet_msg *msg =
        (FAR struct rndis_packet_msg *)req->req->buf;
      memset(msg, 0, RNDIS_PACKET_HDR_SIZE);

      msg->msgtype    = RNDIS_PACKET_MSG;
      msg->msglen     = RNDIS_PACKET_HDR_SIZE + datalen;
      msg->dataoffset = RNDIS_PACKET_HDR_SIZE - 8;
      msg->datalen    = datalen;

      req->req->flags = USBDEV_REQFLAGS_NULLPKT;
      req->req->len   = datalen + RNDIS_PACKET_HDR_SIZE;
    }

  return req->req->len;
}

void net_dump_hex(unsigned char *buf, int len)
{
  uint32_t i,j;
  unsigned char tmp[4*16];
  int buf_len = 4*16;
  for (i = 0;i < len; i++)
  {
    if ((i % 16) == 0)
    {
      if (i > 0)
        uerr("%s\n",  (unsigned char *)tmp);
      memset(tmp,0, buf_len);
      j=0;
    }
    sprintf((char *)(tmp+j*3), "%02x ", buf[i]);
    j++;
  }
  uerr("%s\n", (unsigned char *)tmp);
}

static int rndis_send_data_to_media(FAR struct rndis_dev_s *priv, FAR struct rndis_netpkt_s *netpkt)
{
  FAR struct rndis_dev_s *priv = (FAR struct rndis_dev_s *)arg;
  FAR struct eth_hdr_s *hdr;

  net_lock();
  while (nxmutex_lock(&priv->lock) < 0);
  rndis_giverxreq(priv);
  priv->netdev.d_len = priv->current_rx_datagram_size;
  nxmutex_unlock(&priv->lock);

  size_t datalen;

  datalen = MIN(netpkt->datasize, CONFIG_RNDIS_BULKIN_REQLEN - RNDIS_PACKET_HDR_SIZE);
  uinfo("enter, datalen=%d", datalen);
  if (datalen > 0)
  {
    // net_dump_hex(&priv->rx_req->req->buf[RNDIS_PACKET_HDR_SIZE], datalen);
#if defined(CONFIG_IEEE80211_BESTECHNIC_BES2003)
    if (NET_LL_IEEE80211 == MEDIA_LL_TYPE)
      bes_wifi_data_send(&netpkt->buf[RNDIS_PACKET_HDR_SIZE], datalen);
    else
#endif
#if defined(CONFIG_BES_MODEM)
    if (NET_LL_CELL == MEDIA_LL_TYPE)
    {
      bes_lte_data_send(&netpkt->buf[RNDIS_PACKET_HDR_SIZE + ETH_HDRLEN], datalen - ETH_HDRLEN);
    }
    else
#endif
    {
    }
  }

  return OK;
}

#if defined(CONFIG_BES_MODEM)

#include <nuttx/net/ip.h>

#define ARP_REQUEST    1
#define ARP_REPLY      2

struct arp_hdr_s
{
  uint16_t ah_hwtype;        /* 16-bit Hardware type (Ethernet=0x001) */
  uint16_t ah_protocol;      /* 16-bit Protocol type (IP=0x0800) */
  uint8_t  ah_hwlen;         /*  8-bit Hardware address size (6) */
  uint8_t  ah_protolen;      /*  8-bit Protocol address size (4) */
  uint16_t ah_opcode;        /* 16-bit Operation */
  uint8_t  ah_shwaddr[6];    /* 48-bit Sender hardware address */
  uint16_t ah_sipaddr[2];    /* 32-bit Sender IP address */
  uint8_t  ah_dhwaddr[6];    /* 48-bit Target hardware address */
  uint16_t ah_dipaddr[2];    /* 32-bit Target IP address */
};

#define ARP_REPLY_SIZE   (sizeof(struct eth_hdr_s) + sizeof(struct arp_hdr_s))
uint8_t arp_output_packet[ARP_REPLY_SIZE] = {0};
static void rndis_handle_arp(FAR struct rndis_dev_s *priv, FAR struct rndis_netpkt_s *netpkt)
{
  size_t datalen = 0;
  FAR struct eth_hdr_s *req_eth_hdr   = NULL;
  FAR struct arp_hdr_s *req_arp_hdr   = NULL;
  FAR uint8_t *reply_buff = NULL;
  FAR struct eth_hdr_s *reply_eth_hdr = NULL;
  FAR struct arp_hdr_s *reply_arp_hdr = NULL;
  in_addr_t ipaddr;

  uinfo("rndis_debug %d enter\n", __LINE__);
  datalen = MIN(netpkt->datasize, CONFIG_RNDIS_BULKIN_REQLEN - RNDIS_PACKET_HDR_SIZE);
  if (0 == datalen || NULL == g_media_netdev)
    return;

  // net_dump_hex(&priv->rx_req->req->buf[RNDIS_PACKET_HDR_SIZE], datalen);
  req_eth_hdr = (FAR struct eth_hdr_s *)&netpkt->buf[RNDIS_PACKET_HDR_SIZE];
  if (HTONS(ETHTYPE_ARP) != req_eth_hdr->type)
  {
    uinfo("rndis_debug %d, type = %d\n", __LINE__, req_eth_hdr->type);
    return;
  }

  req_arp_hdr = (FAR struct arp_hdr_s *)&netpkt->buf[RNDIS_PACKET_HDR_SIZE + ETH_HDRLEN];

  ipaddr = net_ip4addr_conv32((req_arp_hdr->ah_dipaddr));

  if (HTONS(ARP_REQUEST) != req_arp_hdr->ah_opcode)
  {
    uinfo("rndis_debug %d, ah_opcode = %04x\n", __LINE__, req_arp_hdr->ah_opcode);
    return;
  }

  if (!net_ipv4addr_cmp(ipaddr, g_media_netdev->d_draddr))
  {
    uinfo("rndis_debug %d, ipaddr=0x%08x, d_draddr==0x%08x\n", __LINE__, ipaddr, g_media_netdev->d_draddr);
    return;
  }

  uinfo("rndis_debug %d\n", __LINE__);
  reply_eth_hdr = (FAR struct eth_hdr_s *)arp_output_packet;
  memcpy(reply_eth_hdr->dest, req_eth_hdr->src, 6);
  memcpy(reply_eth_hdr->src, g_rndis_dummy_router_mac_addr, 6);
  reply_eth_hdr->type = req_eth_hdr->type;

  reply_arp_hdr = (FAR struct arp_hdr_s *)&arp_output_packet[ETH_HDRLEN];
  reply_arp_hdr->ah_hwtype = req_arp_hdr->ah_hwtype;
  reply_arp_hdr->ah_protocol = req_arp_hdr->ah_protocol;
  reply_arp_hdr->ah_hwlen = req_arp_hdr->ah_hwlen;
  reply_arp_hdr->ah_protolen = req_arp_hdr->ah_protolen;
  reply_arp_hdr->ah_opcode = HTONS(ARP_REPLY);
  memcpy(reply_arp_hdr->ah_shwaddr, g_rndis_dummy_router_mac_addr, 6);
  memcpy(reply_arp_hdr->ah_sipaddr, req_arp_hdr->ah_dipaddr, 4);
  memcpy(reply_arp_hdr->ah_dhwaddr, req_arp_hdr->ah_shwaddr, 6);
  memcpy(reply_arp_hdr->ah_dipaddr, req_arp_hdr->ah_sipaddr, 4);

  rndis_receive_eth_data_with_hdr(arp_output_packet, ARP_REPLY_SIZE, true);

  uinfo("rndis_debug %d exit\n", __LINE__);
}

#define UDP_HDRLEN        (8)
#define DHCP_MSGLEN       (548)
#define DHCPD_OUTPUT_SIZE (ETH_HDRLEN + IPv4_HDRLEN + UDP_HDRLEN + DHCP_MSGLEN)
uint8_t dhcpd_output_packet[DHCPD_OUTPUT_SIZE] = {0};
static bool rndis_handler_dhcp(FAR struct rndis_dev_s *priv, FAR struct rndis_netpkt_s *netpkt)
{
  size_t datalen = 0;
  FAR struct eth_hdr_s *input_eth_hdr  = NULL;
  FAR struct eth_hdr_s *output_eth_hdr = NULL;
  uint16_t output_rel_len = 0;

  uinfo("rndis_debug %d enter\n", __LINE__);
  datalen = MIN(netpkt->datasize, CONFIG_RNDIS_BULKIN_REQLEN - RNDIS_PACKET_HDR_SIZE);
  if (ETH_HDRLEN > datalen || NULL == g_media_netdev)
    return false;

  // rndis_iob2buf(priv, priv->rx_req);
  // net_dump_hex(&priv->rx_req->req->buf[RNDIS_PACKET_HDR_SIZE], datalen);
  input_eth_hdr = (FAR struct eth_hdr_s *)&netpkt->buf[RNDIS_PACKET_HDR_SIZE];
  if (HTONS(ETHTYPE_IP) != input_eth_hdr->type)
  {
    uinfo("rndis_debug %d, eth type=0x%04x\n", __LINE__, input_eth_hdr->type);
    return false;
  }

  extern uint16_t rndis_dhcpd_ipv4(FAR struct net_driver_s *netdev, uint8_t *input_buf, uint16_t input_len, uint8_t *output_buf, uint16_t output_len);
  output_rel_len = rndis_dhcpd_ipv4(g_media_netdev,
                                    &netpkt->buf[RNDIS_PACKET_HDR_SIZE + ETH_HDRLEN], datalen - ETH_HDRLEN,
                                    dhcpd_output_packet + ETH_HDRLEN, DHCPD_OUTPUT_SIZE - ETH_HDRLEN);
  if (0 == output_rel_len)
  {
    return false;
  }

  output_rel_len += ETH_HDRLEN;
  output_eth_hdr = (FAR struct eth_hdr_s *)dhcpd_output_packet;
  memcpy(output_eth_hdr->dest, input_eth_hdr->src, 6);
  memcpy(output_eth_hdr->src, g_rndis_dummy_router_mac_addr, 6);
  output_eth_hdr->type = input_eth_hdr->type;

  rndis_receive_eth_data_with_hdr(dhcpd_output_packet, output_rel_len, true);

  uinfo("rndis_debug %d exit\n", __LINE__);
  return true;
}

#endif  // defined(CONFIG_BES_MODEM)

static int rndis_rxdispatch_task_run(int argc, char **argv)
{
  FAR struct rndis_dev_s    *priv   = NULL;
  FAR struct rndis_netpkt_s *netpkt = NULL;
  FAR struct eth_hdr_s      *hdr    = NULL;
  irqstate_t flags;

  do
  {
    osDelay(10);
  } while (NULL == g_rndis_netdev);

  priv = (struct rndis_dev_s *)g_rndis_netdev->d_private;

  do
  {
    netpkt = rndis_remfirstpendingnetpkts(priv);
    if (NULL == netpkt)
    {
      osDelay(10);
      continue;
    }

    net_lock();

    hdr = (FAR struct eth_hdr_s *)&netpkt->buf[RNDIS_PACKET_HDR_SIZE];

    // memcpy(hdr->src, priv->host_mac_address,6);

    // uinfo("hdr dest = %02x %02x %02x %02x %02x %02x\n",
    //       hdr->dest[0], hdr->dest[1], hdr->dest[2], hdr->dest[3], hdr->dest[4], hdr->dest[5]);
    // uinfo("hdr src = %02x %02x %02x %02x %02x %02x\n",
    //       hdr->src[0], hdr->src[1], hdr->src[2], hdr->src[3], hdr->src[4], hdr->src[5]);
    // uinfo("hdr type = %04x\n", hdr->type);

    /* We only accept IP packets of the configured type and ARP packets */

  #ifdef CONFIG_NET_IPv4
    if (hdr->type == HTONS(ETHTYPE_IP))
      {
        uinfo("rndis_debug IPv4\n");
        if (netpkt->datasize > 0)
          {
  #if defined(CONFIG_BES_MODEM)
            if(NET_LL_CELL != MEDIA_LL_TYPE || !rndis_handler_dhcp(priv, netpkt))
  #endif
              rndis_send_data_to_media(priv, netpkt);
          }
      }
    else
  #endif
  #ifdef CONFIG_NET_IPv6
    if (hdr->type == HTONS(ETHTYPE_IP6))
      {
        uinfo("rndis_debug IPv6\n");
        if (netpkt->datasize > 0)
          {
            rndis_send_data_to_media(priv, netpkt);
          }
      }
    else
  #endif
  #ifdef CONFIG_NET_ARP
    if (hdr->type == HTONS(ETHTYPE_ARP))
      {
        uinfo("rndis_debug ARP size=%d, type=%d\n", priv->current_rx_datagram_size, MEDIA_LL_TYPE);
        if (netpkt->datasize > 0)
          {
  #if defined(CONFIG_BES_MODEM)
            if (NET_LL_CELL == MEDIA_LL_TYPE)
            {
              rndis_handle_arp(priv, netpkt);
            }
            else
  #endif
              rndis_send_data_to_media(priv, netpkt);
          }
      }
    else
  #endif
      {
        uerr("rndis_debug ERROR: Unsupported packet type dropped (%02x)\n", HTONS(hdr->type));
      }

    netpkt->datasize = 0;
    rndis_freenetpkts(priv, netpkt);
    rndis_submit_rdreq(priv);
    net_unlock();
  } while (1);
}

void rndis_set_wifi_host_mac_addr(FAR const uint8_t *mac_address)
{
  if (mac_address)
    memcpy(g_rndis_default_mac_addr, mac_address, 6);
}

void rndis_set_media_netdev(FAR struct net_driver_s *netdev)
{
  if (netdev)
    g_media_netdev = netdev;
}

void rndis_notify_netdev_conn_stat(bool connected)
{
  int ret = 0;
  uinfo("notify %d", connected);
  rndis_conn_stat_expect = connected;
}

bool rndis_is_inited(void)
{
  return rndis_inited;
}

void rndis_receive_eth_data(uint8_t *data, uint16_t len, bool has_eth_hdr)
{
  struct rndis_dev_s *priv = NULL;
  size_t datalen = 0;

  if (!rndis_inited || !g_rndis_netdev)
  {
    uerr("rndis not connect, rndis_inited = %d, g_rndis_netdev=%p\n", rndis_inited, g_rndis_netdev);
    return;
  }

  priv = (struct rndis_dev_s *)g_rndis_netdev->d_private;
  if (!priv)
  {
    uerr("rndis not inited\n");
    return;
  }

  if (len == 0 || NULL == data)
  {
    uerr("no data\n");
    return;
  }

  uinfo("enter rx data len=%d, time: %d ticks, epbulkin", len, (hal_sys_timer_get()));
  // net_dump_hex(data,len);

  if (!rndis_allocnetreq(priv))
  {
    uerr("net_req is null, epbulkin\n");
    return;
  }

#if defined(CONFIG_IEEE80211_BESTECHNIC_BES2003)
  if (NET_LL_IEEE80211 == MEDIA_LL_TYPE)
  {
    memcpy(&priv->net_req->req->buf[RNDIS_PACKET_HDR_SIZE], data, min(RNDIS_RECIVE_LEN, len));
    datalen = min(RNDIS_RECIVE_LEN, len);
  }
  else
#endif
#if defined(CONFIG_BES_MODEM)
  if (NET_LL_CELL == MEDIA_LL_TYPE)
  {
    if (has_eth_hdr)
    {
      memcpy(&priv->net_req->req->buf[RNDIS_PACKET_HDR_SIZE], data, min(RNDIS_RECIVE_LEN, len));
      datalen = min(RNDIS_RECIVE_LEN, len);
    }
    else
    {
      FAR struct eth_hdr_s *hdr;
      hdr = (FAR struct eth_hdr_s *)(&priv->net_req->req->buf[RNDIS_PACKET_HDR_SIZE]);
      memcpy(hdr->dest, g_rndis_default_mac_addr, 6);
      memcpy(hdr->src, g_rndis_dummy_router_mac_addr, 6);
      hdr->type = HTONS(ETHTYPE_IP);
      memcpy(&priv->net_req->req->buf[RNDIS_PACKET_HDR_SIZE + ETH_HDRLEN], data, min(RNDIS_RECIVE_LEN - ETH_HDRLEN, len));
      datalen = min(RNDIS_RECIVE_LEN, len + ETH_HDRLEN);
    }
  }
  else
#endif
  {
  }

  FAR struct rndis_packet_msg *msg = (FAR struct rndis_packet_msg *)priv->net_req->req->buf;
  memset(msg, 0, RNDIS_PACKET_HDR_SIZE);

  msg->msgtype    = RNDIS_PACKET_MSG;
  msg->msglen     = RNDIS_PACKET_HDR_SIZE + datalen;
  msg->dataoffset = RNDIS_PACKET_HDR_SIZE - 8;
  msg->datalen    = datalen;

  priv->net_req->req->flags = USBDEV_REQFLAGS_NULLPKT;
  priv->net_req->req->len   = datalen + RNDIS_PACKET_HDR_SIZE;

  irqstate_t flags = enter_critical_section();
  uinfo("EP_SUBMIT, epbulkin");
  priv->net_req->req->priv = priv->net_req;
  EP_SUBMIT(priv->epbulkin, priv->net_req->req);
  priv->net_req = NULL;
  leave_critical_section(flags);

  uinfo("exit, time: %d ticks, epbulkin", (hal_sys_timer_get()));
}

void rndis_receive_eth_data_with_hdr(uint8_t *data, uint16_t len, bool has_eth_hdr)
{
  struct rndis_dev_s *priv = NULL;
  size_t datalen = 0;

  if (!rndis_inited || !g_rndis_netdev)
  {
    uerr("rndis not connect, rndis_inited = %d, g_rndis_netdev=%p\n", rndis_inited, g_rndis_netdev);
    return;
  }

  priv = (struct rndis_dev_s *)g_rndis_netdev->d_private;
  if (!priv)
  {
    uerr("rndis not inited\n");
    return;
  }

  if (len == 0 || NULL == data)
  {
    uerr("no data\n");
    return;
  }

  uinfo("enter rx data len=%d, time: %d ticks, epbulkin", len, (hal_sys_timer_get()));
  // net_dump_hex(data,len);

  if (!rndis_allocdummyreq(priv))
  {
    uerr("dummy_req is null, epbulkin\n");
    return;
  }

#if defined(CONFIG_IEEE80211_BESTECHNIC_BES2003)
  if (NET_LL_IEEE80211 == MEDIA_LL_TYPE)
  {
    memcpy(&priv->dummy_req->req->buf[RNDIS_PACKET_HDR_SIZE], data, min(RNDIS_RECIVE_LEN, len));
    datalen = min(RNDIS_RECIVE_LEN, len);
  }
  else
#endif
#if defined(CONFIG_BES_MODEM)
  if (NET_LL_CELL == MEDIA_LL_TYPE)
  {
    if (has_eth_hdr)
    {
      memcpy(&priv->dummy_req->req->buf[RNDIS_PACKET_HDR_SIZE], data, min(RNDIS_RECIVE_LEN, len));
      datalen = min(RNDIS_RECIVE_LEN, len);
    }
    else
    {
      FAR struct eth_hdr_s *hdr;
      hdr = (FAR struct eth_hdr_s *)(&priv->dummy_req->req->buf[RNDIS_PACKET_HDR_SIZE]);
      memcpy(hdr->dest, g_rndis_default_mac_addr, 6);
      memcpy(hdr->src, g_rndis_dummy_router_mac_addr, 6);
      hdr->type = HTONS(ETHTYPE_IP);
      memcpy(&priv->dummy_req->req->buf[RNDIS_PACKET_HDR_SIZE + ETH_HDRLEN], data, min(RNDIS_RECIVE_LEN - ETH_HDRLEN, len));
      datalen = min(RNDIS_RECIVE_LEN, len + ETH_HDRLEN);
    }
  }
  else
#endif
  {
  }

  FAR struct rndis_packet_msg *msg = (FAR struct rndis_packet_msg *)priv->dummy_req->req->buf;
  memset(msg, 0, RNDIS_PACKET_HDR_SIZE);

  msg->msgtype    = RNDIS_PACKET_MSG;
  msg->msglen     = RNDIS_PACKET_HDR_SIZE + datalen;
  msg->dataoffset = RNDIS_PACKET_HDR_SIZE - 8;
  msg->datalen    = datalen;

  priv->dummy_req->req->flags = USBDEV_REQFLAGS_NULLPKT;
  priv->dummy_req->req->len   = datalen + RNDIS_PACKET_HDR_SIZE;

  irqstate_t flags = enter_critical_section();
  uinfo("EP_SUBMIT, epbulkin");
  priv->dummy_req->req->priv = priv->dummy_req;
  EP_SUBMIT(priv->epbulkin, priv->dummy_req->req);
  priv->dummy_req = NULL;
  leave_critical_section(flags);

  uinfo("exit, time: %d ticks, epbulkin", (hal_sys_timer_get()));
}

/****************************************************************************
 * Name: rndis_txpoll
 *
 * Description:
 *   Sends the packet that is stored in the network packet buffer. Called
 *   from work queue by e.g. txavail and txpoll callbacks.
 *
 * Input Parameters:
 *   dev: pointer to network driver structure
 *
 * Assumptions:
 *   Caller holds the network lock
 *
 ****************************************************************************/

static int rndis_txpoll(FAR struct net_driver_s *dev)
{
  FAR struct rndis_dev_s *priv = (FAR struct rndis_dev_s *)dev->d_private;

  if (!priv->connected)
    {
      return -EBUSY;
    }

  return rndis_transmit(priv);
}

/****************************************************************************
 * Name: rndis_transmit
 *
 * Description:
 *   Start hardware transmission.
 *
 ****************************************************************************/

static int rndis_transmit(FAR struct rndis_dev_s *priv)
{
  int ret = OK;

  /* Queue the packet */

  rndis_fillrequest(priv, priv->net_req);
  rndis_sendnetreq(priv);

  if (!rndis_allocnetreq(priv))
    {
      ret = -EBUSY;
    }

  return ret;
}

/****************************************************************************
 * Name: rndis_ifup
 *
 * Description:
 *   Network ifup callback
 *
 ****************************************************************************/

static int rndis_ifup(FAR struct net_driver_s *dev)
{
  return OK;
}

/****************************************************************************
 * Name: rndis_ifdown
 *
 * Description:
 *   Network ifdown callback
 *
 ****************************************************************************/

static int rndis_ifdown(FAR struct net_driver_s *dev)
{
  return OK;
}

/****************************************************************************
 * Name: rndis_txavail_work
 *
 * Description:
 *   txavail worker function
 *
 ****************************************************************************/

static void rndis_txavail_work(FAR void *arg)
{
  FAR struct rndis_dev_s *priv = (FAR struct rndis_dev_s *)arg;

  net_lock();

  if (rndis_allocnetreq(priv))
    {
      devif_poll(&priv->netdev, rndis_txpoll);
      if (priv->net_req != NULL)
        {
          rndis_freenetreq(priv);
        }
    }

  net_unlock();
}

/****************************************************************************
 * Name: rndis_txavail
 *
 * Description:
 *   Network txavail callback that's called when there are buffers available
 *   for sending data. May be called from an interrupt, so we must queue a
 *   worker to do the actual processing.
 *
 ****************************************************************************/

static int rndis_txavail(FAR struct net_driver_s *dev)
{
  FAR struct rndis_dev_s *priv = (FAR struct rndis_dev_s *)dev->d_private;

  if (work_available(&priv->pollwork))
    {
      work_queue(ETHWORK, &priv->pollwork, rndis_txavail_work, priv, 0);
    }

  return OK;
}

/****************************************************************************
 * Name: rndis_recvpacket
 *
 * Description:
 *   Handles a USB packet arriving on the data bulk out endpoint.
 *
 * Assumptions:
 *   Called from the USB interrupt handler with interrupts disabled.
 *
 ****************************************************************************/

static inline int rndis_recvpacket(FAR struct rndis_dev_s *priv,
                                   FAR uint8_t *reqbuf, uint16_t reqlen)
{
  if (!rndis_allocnetpkts(priv))
    {
      uerr("no free netpkt\n");
      return -ENOMEM;
    }

  if (!priv->connected)
    {
      uerr("not connect\n");
      return -EBUSY;
    }

  uinfo("enter\n");
  // net_dump_hex(reqbuf, reqlen);

  if (!priv->current_rx_datagram_size)
    {
      if (reqlen < 16)
        {
          /* Packet too small to contain a message header */
        }
      else
        {
          /* The packet contains a RNDIS packet message header */

          FAR struct rndis_packet_msg *msg =
            (FAR struct rndis_packet_msg *)reqbuf;
          if (msg->msgtype == RNDIS_PACKET_MSG)
            {
              priv->current_rx_received = reqlen;
              priv->current_rx_datagram_size = msg->datalen;
              priv->net_packet->datasize = msg->datalen;
              priv->current_rx_msglen = msg->msglen;

              /* According to RNDIS-over-USB send, if the message length is a
               * multiple of endpoint max packet size, the host must send an
               * additional single-byte zero packet. Take that in account
               * here.
               */

              if (!(priv->current_rx_msglen % priv->epbulkout->maxpacket))
                {
                  priv->current_rx_msglen += 1;
                }

              /* Data offset is defined as an offset from the beginning of
               * the offset field itself
               */

              priv->current_rx_datagram_offset = msg->dataoffset + 8;
              if (priv->current_rx_datagram_offset < reqlen)
                {
                  memcpy(&priv->net_packet->buf[RNDIS_PACKET_HDR_SIZE],
                         &reqbuf[priv->current_rx_datagram_offset],
                         reqlen - priv->current_rx_datagram_offset);
                }
            }
          else
            {
              uerr("Unknown RNDIS message type %" PRIu32 "\n", msg->msgtype);
            }
        }
    }
  else
    {
      if (priv->current_rx_received >= priv->current_rx_datagram_offset &&
          priv->current_rx_received <= priv->current_rx_datagram_size +
          priv->current_rx_datagram_offset)
        {
          size_t index = priv->current_rx_received -
                         priv->current_rx_datagram_offset;
          size_t copysize = MIN(reqlen,
                                priv->current_rx_datagram_size - index);

          /* Check if the received packet exceeds request buffer */

          if ((index + copysize) <= CONFIG_NET_ETH_PKTSIZE)
            {
              memcpy(&priv->net_packet->buf[RNDIS_PACKET_HDR_SIZE + index], reqbuf, copysize);
            }
          else
            {
              uerr("The packet exceeds request buffer (reqlen=%d)\n", reqlen);
            }
        }
      priv->current_rx_received += reqlen;
    }

  if (priv->current_rx_received >= priv->current_rx_msglen)
    {
      /* Check for a usable packet length (4 added for the CRC) */

      if (priv->current_rx_datagram_size > (CONFIG_NET_ETH_PKTSIZE + 4) ||
          priv->current_rx_datagram_size <= (ETH_HDRLEN + 4))
        {
          uerr("ERROR: Bad packet size dropped (%zu)\n",
               priv->current_rx_datagram_size);
          NETDEV_RXERRORS(&priv->netdev);
          priv->current_rx_datagram_size = 0;
        }
      else
        {
          priv->current_rx_datagram_size = 0;
          // if (priv->net_packet)
          {
            rndis_block_rx(priv);
            uinfo("rndis_addpendingnetpkts\n");
            rndis_addpendingnetpkts(priv);
            priv->rndis_host_tx_count++;
            rndis_unblock_rx(priv);
          }
          // priv->current_rx_datagram_size = 0;
          // DEBUGASSERT(work_available(&priv->rxwork));
          // ret = work_queue(ETHWORK, &priv->rxwork, rndis_rxdispatch,
          //                  priv, 0);
          // DEBUGASSERT(ret == 0);
          // UNUSED(ret);

          // rndis_block_rx(priv);
          // priv->rndis_host_tx_count++;
          // return -EBUSY;
        }
    }
 
  return OK;
}

/****************************************************************************
 * Name: rndis_prepare_response
 *
 * Description:
 *   Passes the RX packet buffer to the network
 *
 * Input Parameters:
 *   priv: pointer to RNDIS device driver structure
 *
 * Returns:
 *   pointer to response buffer
 *
 * Assumptions:
 *   Called from critical section
 *
 ****************************************************************************/

static FAR void *
rndis_prepare_response(FAR struct rndis_dev_s *priv, size_t size,
                       FAR struct rndis_command_header *request_hdr)
{
  size_t size_words = size / sizeof(uint32_t);
  uint32_t *buf = priv->response_queue + priv->response_queue_words;
  FAR struct rndis_response_header *hdr =
    (FAR struct rndis_response_header *)buf;

  if (priv->response_queue_words + size_words > RNDIS_RESP_QUEUE_WORDS)
    {
      uerr("RNDIS response queue full, dropping command %08x",
           (unsigned int)request_hdr->msgtype);
      return NULL;
    }

  hdr->msgtype = request_hdr->msgtype | RNDIS_MSG_COMPLETE;
  hdr->msglen  = size;
  hdr->reqid   = request_hdr->reqid;
  hdr->status  = RNDIS_STATUS_SUCCESS;

  return hdr;
}

static FAR void *
rndis_prepare_indicate_status(FAR struct rndis_dev_s *priv, size_t size,
                              FAR struct rndis_command_header *request_hdr, uint32_t status)
{
  size_t size_words = size / sizeof(uint32_t);
  uint32_t *buf = priv->response_queue + priv->response_queue_words;
  struct rndis_indicate_msg *hdr = (struct rndis_indicate_msg *)buf;

  if (priv->response_queue_words + size_words > RNDIS_RESP_QUEUE_WORDS)
  {
    uerr("RNDIS response queue full, dropping command %08x", (unsigned int)request_hdr->msgtype);
    return NULL;
  }

  hdr->msgtype   = RNDIS_INDICATE_MSG;
  hdr->msglen    = size;
  hdr->status    = status;
  hdr->buflen    = 0;
  hdr->bufoffset = 0;

  return hdr;
}

/****************************************************************************
 * Name: rndis_send_encapsulated_response
 *
 * Description:
 *   Give a notification to the host that there is an encapsulated response
 *   available.
 *
 * Input Parameters:
 *   priv: pointer to RNDIS device driver structure
 *
 * Assumptions:
 *   Called from critical section
 *
 ****************************************************************************/

static int rndis_send_encapsulated_response(FAR struct rndis_dev_s *priv,
                                            size_t size)
{
  size_t size_words = size / sizeof(uint32_t);
  FAR struct rndis_notification *notif =
    (FAR struct rndis_notification *)priv->epintin_req->buf;

  /* RNDIS packets should always be multiple of 4 bytes in size */

  DEBUGASSERT(size_words * sizeof(uint32_t) == size);

  /* Mark the response as available in the queue */

  priv->response_queue_words += size_words;
  DEBUGASSERT(priv->response_queue_words <= RNDIS_RESP_QUEUE_WORDS);

  /* Send notification on IRQ endpoint, to tell host to read the data. */

  notif->notification = RNDIS_NOTIFICATION_RESPONSE_AVAILABLE;
  notif->reserved = 0;
  priv->epintin_req->len = sizeof(struct rndis_notification);

  EP_CANCEL(priv->epintin, priv->epintin_req);
  EP_SUBMIT(priv->epintin, priv->epintin_req);

  return OK;
}

/****************************************************************************
 * Name: rndis_handle_control_message
 *
 * Description:
 *   Handle a RNDIS control message.
 *
 * Input Parameters:
 *   priv: pointer to RNDIS device driver structure
 *
 * Assumptions:
 *   Called from critical section
 *
 ****************************************************************************/

static int rndis_handle_control_message(FAR struct rndis_dev_s *priv,
                                        FAR uint8_t *dataout,
                                        uint16_t outlen)
{
  FAR struct rndis_command_header *cmd_hdr =
    (FAR struct rndis_command_header *)dataout;

  switch (cmd_hdr->msgtype)
    {
      case RNDIS_INITIALIZE_MSG:
        {
          FAR struct rndis_initialize_cmplt *resp;
          size_t respsize = sizeof(struct rndis_initialize_cmplt);

          resp = rndis_prepare_response(priv, respsize, cmd_hdr);
          if (!resp)
            {
              return -ENOMEM;
            }

          resp->major      = RNDIS_MAJOR_VERSION;
          resp->minor      = RNDIS_MINOR_VERSION;
          resp->devflags   = RNDIS_DEVICEFLAGS;
          resp->medium     = RNDIS_MEDIUM_802_3;
          resp->pktperxfer = 1;
          resp->xfrsize    = (4 + 44 + 22) + RNDIS_BUFFER_SIZE;
          resp->pktalign   = 2;

          rndis_send_encapsulated_response(priv, respsize);
        }
        break;

      case RNDIS_HALT_MSG:
        {
          uinfo("RNDIS_HALT_MSG");
          priv->response_queue_words = 0;
          priv->connected = false;
          rndis_inited = false;
        }
        break;

      case RNDIS_QUERY_MSG:
        {
          int i;
          size_t max_reply_size = sizeof(struct rndis_query_cmplt) +
                                  sizeof(g_rndis_supported_oids);
          FAR struct rndis_query_cmplt *resp;
          FAR struct rndis_query_msg *req =
            (FAR struct rndis_query_msg *)dataout;

          resp = rndis_prepare_response(priv, max_reply_size, cmd_hdr);
          if (!resp)
            {
              return -ENOMEM;
            }

          resp->hdr.msglen = sizeof(struct rndis_query_cmplt);
          resp->bufoffset  = 0;
          resp->buflen     = 0;
          resp->hdr.status = RNDIS_STATUS_NOT_SUPPORTED;

          for (i = 0;
               i < sizeof(g_rndis_oid_values) /
                   sizeof(g_rndis_oid_values[0]);
               i++)
            {
              bool match = (g_rndis_oid_values[i].objid == req->objid);

              if (!match && g_rndis_oid_values[i].objid == 0)
                {
                  int j;

                  /* Check whether to apply the fallback entry */

                  for (j = 0;
                       j < sizeof(g_rndis_supported_oids) / sizeof(uint32_t);
                       j++)
                    {
                      if (g_rndis_supported_oids[j] == req->objid)
                        {
                          match = true;
                          break;
                        }
                    }
                }

              if (match)
                {
                  resp->hdr.status = RNDIS_STATUS_SUCCESS;
                  resp->bufoffset  = 16;
                  resp->buflen     = g_rndis_oid_values[i].length;

                  if (req->objid == RNDIS_OID_GEN_CURRENT_PACKET_FILTER)
                    {
                      resp->buffer[0] = priv->rndis_packet_filter;
                    }
                  else if (req->objid == RNDIS_OID_GEN_XMIT_OK)
                    {
                      resp->buffer[0] = priv->rndis_host_tx_count;
                    }
                  else if (req->objid == RNDIS_OID_GEN_RCV_OK)
                    {
                      resp->buffer[0] = priv->rndis_host_rx_count;
                    }
                  else if (req->objid == RNDIS_OID_802_3_CURRENT_ADDRESS ||
                           req->objid == RNDIS_OID_802_3_PERMANENT_ADDRESS)
                    {
                      memcpy(resp->buffer, priv->host_mac_address, 6);
                    }
                  else if (req->objid == RNDIS_OID_GEN_MEDIA_CONNECT_STATUS) {
                    }
                  else if (g_rndis_oid_values[i].data)
                    {
                      memcpy(resp->buffer, g_rndis_oid_values[i].data,
                             resp->buflen);
                    }
                  else
                    {
                      memcpy(resp->buffer, &g_rndis_oid_values[i].value,
                             resp->buflen);
                    }
                  break;
                }
            }

          uinfo("RNDIS Query RID=%08x OID=%08x LEN=%d DAT=%08x",
                (unsigned)req->hdr.reqid, (unsigned)req->objid,
                (int)resp->buflen, (unsigned)resp->buffer[0]);

          resp->hdr.msglen += resp->buflen;

          /* Align to word boundary */

          if ((resp->hdr.msglen & 3) != 0)
            {
              resp->hdr.msglen += 4 - (resp->hdr.msglen & 3);
            }

          rndis_send_encapsulated_response(priv, resp->hdr.msglen);
        }
        break;

      case RNDIS_SET_MSG:
        {
          FAR struct rndis_set_msg *req;
          FAR struct rndis_response_header *resp;
          size_t respsize = sizeof(struct rndis_response_header);

          resp = rndis_prepare_response(priv, respsize, cmd_hdr);
          req  = (FAR struct rndis_set_msg *)dataout;

          if (!resp)
            {
              return -ENOMEM;
            }

          uinfo("RNDIS SET RID=%08x OID=%08x LEN=%d DAT=%08x",
                (unsigned)req->hdr.reqid, (unsigned)req->objid,
                (int)req->buflen, (unsigned)req->buffer[0]);

          if (req->objid == RNDIS_OID_GEN_CURRENT_PACKET_FILTER)
            {
              priv->rndis_packet_filter = req->buffer[0];

              if (req->buffer[0] == 0)
                {
                  priv->connected = false;
                  rndis_inited = false;
                }
              else
                {
                  uinfo("RNDIS is now connected");
                  priv->connected = true;
                  rndis_inited = true;
                }
            }
          else if (req->objid == RNDIS_OID_802_3_MULTICAST_LIST)
            {
              uinfo("RNDIS multicast list ignored");
            }
          else
            {
              uinfo("RNDIS unsupported set %08x", (unsigned)req->objid);
              resp->status = RNDIS_STATUS_NOT_SUPPORTED;
            }

          rndis_send_encapsulated_response(priv, respsize);
        }
        break;

      case RNDIS_RESET_MSG:
        {
          uinfo("RNDIS_RESET_MSG");
          FAR struct rndis_reset_cmplt *resp;
          size_t respsize = sizeof(struct rndis_reset_cmplt);

          priv->response_queue_words = 0;
          resp = rndis_prepare_response(priv, respsize, cmd_hdr);

          if (!resp)
            {
              return -ENOMEM;
            }

          resp->addreset  = 0;
          priv->connected = false;
          rndis_inited = false;
          rndis_send_encapsulated_response(priv, respsize);
        }
        break;

      case RNDIS_KEEPALIVE_MSG:
        {
          FAR struct rndis_response_header *resp;
          size_t respsize = sizeof(struct rndis_indicate_msg);
          if ((false == rndis_conn_stat_cur) && (true == rndis_conn_stat_expect))
          {
            uinfo("*****************");
            uinfo("*******wifi connected**********");
            uinfo("*****************");

            usbclass_setconfig(priv, RNDIS_CONFIGID);
            resp = rndis_prepare_indicate_status(priv, respsize, cmd_hdr, RNDIS_STATUS_MEDIA_CONNECT);
            if (!resp) {
                return -ENOMEM;
            }
            rndis_conn_stat_cur = rndis_conn_stat_expect;
            rndis_send_encapsulated_response(priv, respsize);
            break;
          }
          else if ((true == rndis_conn_stat_cur) && (false == rndis_conn_stat_expect))
          {
            uinfo("*****************");
            uinfo("*******wifi disconnected**********");
            uinfo("*****************");
            resp = rndis_prepare_indicate_status(priv, respsize, cmd_hdr, RNDIS_STATUS_MEDIA_DISCONNECT);
            if (!resp) {
                return -ENOMEM;
            }
            rndis_conn_stat_cur = rndis_conn_stat_expect;
            rndis_send_encapsulated_response(priv, respsize);
            break;
          }

          resp = rndis_prepare_response(priv, respsize, cmd_hdr);
          if (!resp)
            {
              return -ENOMEM;
            }

          rndis_send_encapsulated_response(priv, respsize);
        }
        break;

      default:
        uwarn("Unsupported RNDIS control message: %" PRIu32 "\n",
              cmd_hdr->msgtype);
    }

  return OK;
}

/****************************************************************************
 * Name: rndis_rdcomplete
 *
 * Description:
 *   Handle completion of read request on the bulk OUT endpoint.
 *
 ****************************************************************************/

static void rndis_rdcomplete(FAR struct usbdev_ep_s *ep,
                             FAR struct usbdev_req_s *req)
{
  FAR struct rndis_dev_s *priv;
  int ret;
  uinfo("rndis_debug enter");

  /* Sanity check */

#ifdef CONFIG_DEBUG_FEATURES
  if (!ep || !ep->priv || !req)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_INVALIDARG), 0);
      return;
    }
#endif

  /* Extract references to private data */

  priv = (FAR struct rndis_dev_s *)ep->priv;

  /* Process the received data unless this is some unusual condition */

  ret = OK;

  priv->rdreq_submitted = false;

  switch (req->result)
    {
    case 0: /* Normal completion */
      ret = rndis_recvpacket(priv, req->buf, req->xfrd);
      DEBUGASSERT(ret != -ENOMEM);
      break;

    case -ESHUTDOWN: /* Disconnection */
      uerr("ESHUTDOWN Disconnection");
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_RDSHUTDOWN), 0);
      return;

    default: /* Some other error occurred */
      uerr("Some other error occurred");
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_RDUNEXPECTED),
               (uint16_t)-req->result);
      break;
    };

  if (ret == OK && rndis_hasfreenetpkts(priv))
    {
      uinfo("call rndis_submit_rdreq");
      rndis_submit_rdreq(priv);
    }
  else
    {
      uinfo("ret=%d", ret);
    }

}

/****************************************************************************
 * Name: rndis_wrcomplete
 *
 * Description:
 *   Handle completion of write request.  This function probably executes
 *   in the context of an interrupt handler.
 *
 ****************************************************************************/

static void rndis_wrcomplete(FAR struct usbdev_ep_s *ep,
                             FAR struct usbdev_req_s *req)
{
  FAR struct rndis_dev_s *priv;
  FAR struct rndis_req_s *reqcontainer;

  uinfo("enter, epbulkin");
  /* Sanity check */

#ifdef CONFIG_DEBUG_FEATURES
  if (!ep || !ep->priv || !req || !req->priv)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_INVALIDARG), 0);
      return;
    }
#endif

  /* Extract references to our private data */

  priv         = (FAR struct rndis_dev_s *)ep->priv;
  reqcontainer = (FAR struct rndis_req_s *)req->priv;

  /* Return the write request to the free list */

  rndis_freewrreq(priv, reqcontainer);

  switch (req->result)
    {
    case OK: /* Normal completion */
      // uinfo("result is ok, epbulkin");
      priv->rndis_host_rx_count++;
      break;

    case -ESHUTDOWN: /* Disconnection */
      break;

    default: /* Some other error occurred */
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_WRUNEXPECTED),
               (uint16_t)-req->result);
      break;
    }
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
  FAR struct rndis_dev_s *priv;

  if (req->result || req->xfrd != req->len)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_REQRESULT),
               (uint16_t)-req->result);
    }
  else if (req->len > 0 && req->priv)
    {
      /* Get EP0 request private data  */

      priv = (FAR struct rndis_dev_s *)req->priv;

      /* This transfer was from the response queue,
       * subtract remaining byte count.
       */

      size_t len_words = req->len / sizeof(uint32_t);
      DEBUGASSERT(len_words * sizeof(uint32_t) == req->len);
      req->priv = 0;
      if (len_words >= priv->response_queue_words)
      {
        /* Queue now empty */

        priv->response_queue_words = 0;
      }
      else
      {
        /* Copy the remaining responses to beginning of buffer. */

        priv->response_queue_words -= len_words;
        memcpy(priv->response_queue, priv->response_queue + len_words,
               priv->response_queue_words * sizeof(uint32_t));
      }
    }
}

/****************************************************************************
 * Name: usbclass_epintin_complete
 *
 * Description:
 *   Handle completion of interrupt IN endpoint operations
 *
 ****************************************************************************/

static void usbclass_epintin_complete(FAR struct usbdev_ep_s *ep,
                                      FAR struct usbdev_req_s *req)
{
  if (req->result || req->xfrd != req->len)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_REQRESULT),
               (uint16_t)-req->result);
    }
}

/****************************************************************************
 * Name: usbclass_mkstrdesc
 *
 * Description:
 *   Construct a string descriptor
 *
 ****************************************************************************/

static int usbclass_mkstrdesc(uint8_t id, FAR struct usb_strdesc_s *strdesc)
{
  FAR uint8_t *data = (FAR uint8_t *)(strdesc + 1);
  FAR const char *str;
  int len;
  int ndata;
  int i;

  switch (id)
    {
#ifndef CONFIG_RNDIS_COMPOSITE
      case 0:
        {
          /* Descriptor 0 is the language id */

          strdesc->len  = 4;
          strdesc->type = USB_DESC_TYPE_STRING;
          data[0] = LSBYTE(RNDIS_STR_LANGUAGE);
          data[1] = MSBYTE(RNDIS_STR_LANGUAGE);
          return 4;
        }

      case RNDIS_MANUFACTURERSTRID:
        str = CONFIG_RNDIS_VENDORSTR;
        break;

      case RNDIS_PRODUCTSTRID:
        str = CONFIG_RNDIS_PRODUCTSTR;
        break;

      case RNDIS_SERIALSTRID:
#ifdef CONFIG_BOARD_USBDEV_SERIALSTR
        str = board_usbdev_serialstr();
#else
        str = CONFIG_RNDIS_SERIALSTR;
#endif
        break;
#endif

      default:
        return -EINVAL;
    }

  /* The string is utf16-le.  The poor man's utf-8 to utf16-le
   * conversion below will only handle 7-bit en-us ascii
   */

  len = strlen(str);
  if (len > (RNDIS_MAXSTRLEN / 2))
    {
      len = (RNDIS_MAXSTRLEN / 2);
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
  uint16_t totallen = 0;
  uint8_t epno;
  int ret;

  /* Check for switches between high and full speed */

  if (type == USB_DESC_TYPE_OTHERSPEEDCONFIG && speed < USB_SPEED_SUPER)
    {
      speed = speed == USB_SPEED_HIGH ? USB_SPEED_FULL : USB_SPEED_HIGH;
    }

  /* This is the total length of the configuration (not necessarily the
   * size that we will be sending now).
   */
#ifndef CONFIG_RNDIS_COMPOSITE
  if (buf != NULL)
    {
      /* Configuration descriptor.
       * If the USB serial device is configured as part of  composite device,
       * then the configuration descriptor will be provided by the
       * composite device logic.
       */

      FAR struct usb_cfgdesc_s *dest = (FAR struct usb_cfgdesc_s *)buf;
      int16_t size = usbclass_mkcfgdesc(NULL, NULL, speed, type);

      memcpy(buf, &g_rndis_cfgdesc, sizeof(struct usb_cfgdesc_s));
      dest->type = type;                                     /* Descriptor type */
      dest->totallen[0] = LSBYTE(size);                      /* LS Total length */
      dest->totallen[1] = MSBYTE(size);                      /* MS Total length */

      buf += sizeof(struct usb_cfgdesc_s);
    }

  totallen += sizeof(struct usb_cfgdesc_s);

#elif defined(CONFIG_COMPOSITE_IAD)
  /* Interface association descriptor */

  if (buf != NULL)
    {
      FAR struct usb_iaddesc_s *dest = (FAR struct usb_iaddesc_s *)buf;

      memcpy(dest, &g_rndis_assoc_desc, sizeof(struct usb_iaddesc_s));
      dest->firstif += devinfo->ifnobase;

      buf += sizeof(struct usb_iaddesc_s);
    }

  totallen += sizeof(struct usb_iaddesc_s);
#endif

  if (buf != NULL)
    {
      FAR struct usb_ifdesc_s *dest = (FAR struct usb_ifdesc_s *)buf;

      memcpy(dest, &g_rndis_comm_ifdesc, sizeof(struct usb_ifdesc_s));
#ifdef CONFIG_RNDIS_COMPOSITE
      dest->ifno += devinfo->ifnobase;
#endif
      buf += sizeof(struct usb_ifdesc_s);
    }

  totallen += sizeof(struct usb_ifdesc_s);

  epno = devinfo ? devinfo->epno[RNDIS_EP_INTIN_IDX] : 0;
  ret = usbdev_copy_epdesc((struct usb_epdesc_s *)buf,
                           epno,
                           speed,
                           &g_rndis_epintindesc);

  if (buf != NULL)
    {
      buf += ret;
    }

  totallen += ret;

  if (buf != NULL)
    {
      FAR struct usb_ifdesc_s *dest = (FAR struct usb_ifdesc_s *)buf;

      memcpy(dest, &g_rndis_data_ifdesc, sizeof(struct usb_ifdesc_s));
#ifdef CONFIG_RNDIS_COMPOSITE
      dest->ifno += devinfo->ifnobase;
#endif
      buf += sizeof(struct usb_ifdesc_s);
    }

  totallen += sizeof(struct usb_ifdesc_s);

  epno = devinfo ? devinfo->epno[RNDIS_EP_BULKIN_IDX] : 0;
  ret = usbdev_copy_epdesc((struct usb_epdesc_s *)buf,
                           epno,
                           speed,
                           &g_rndis_epbulkindesc);
  if (buf != NULL)
    {
      buf += ret;
    }

  totallen += ret;

  epno = devinfo ? devinfo->epno[RNDIS_EP_BULKOUT_IDX] : 0;
  ret = usbdev_copy_epdesc((struct usb_epdesc_s *)buf,
                           epno,
                           speed,
                           &g_rndis_epbulkoutdesc);
  if (buf != NULL)
    {
      buf += ret;
    }

  totallen += ret;

  return totallen;
}

/****************************************************************************
 * Name: usbclass_bind
 *
 * Description:
 *   Invoked when the driver is bound to a USB device driver
 *
 ****************************************************************************/

static int usbclass_bind(FAR struct usbdevclass_driver_s *driver,
                         FAR struct usbdev_s *dev)
{
  FAR struct rndis_dev_s *priv = ((FAR struct rndis_driver_s *)driver)->dev;
  FAR struct rndis_req_s *reqcontainer;
  size_t reqlen;
  int ret;
  int i;

  usbtrace(TRACE_CLASSBIND, 0);

  /* Bind the structures */

  priv->usbdev   = dev;

  /* Save the reference to our private data structure in EP0 so that it
   * can be recovered in ep0 completion events (Unless we are part of
   * a composite device and, in that case, the composite device owns
   * EP0).
   */

#ifndef CONFIG_RNDIS_COMPOSITE
  dev->ep0->priv = priv;
#endif

  /* Preallocate control request */

  priv->ctrlreq = usbdev_allocreq(dev->ep0, RNDIS_CTRLREQ_LEN);
  if (priv->ctrlreq == NULL)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_ALLOCCTRLREQ), 0);
      ret = -ENOMEM;
      goto errout;
    }

  priv->ctrlreq->callback = usbclass_ep0incomplete;

  /* Pre-allocate all endpoints... the endpoints will not be functional
   * until the SET CONFIGURATION request is processed in usbclass_setconfig.
   * This is done here because there may be calls to kmm_malloc and the SET
   * CONFIGURATION processing probably occurs within interrupt handling
   * logic where kmm_malloc calls will fail.
   */

  /* Pre-allocate the IN interrupt endpoint */

  priv->epintin = DEV_ALLOCEP(dev,
                    USB_EPIN(priv->devinfo.epno[RNDIS_EP_INTIN_IDX]),
                    true, USB_EP_ATTR_XFER_INT);
  if (!priv->epintin)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_EPINTINALLOCFAIL), 0);
      ret = -ENODEV;
      goto errout;
    }

  priv->epintin->priv = priv;

  priv->epintin_req =
    usbdev_allocreq(priv->epintin, sizeof(struct rndis_notification));
  if (priv->epintin_req == NULL)
  {
    usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_RDALLOCREQ), -ENOMEM);
    ret = -ENOMEM;
    goto errout;
  }

  priv->epintin_req->callback = usbclass_epintin_complete;

  /* Pre-allocate the IN bulk endpoint */

  priv->epbulkin = DEV_ALLOCEP(dev,
                      USB_EPIN(priv->devinfo.epno[RNDIS_EP_BULKIN_IDX]),
                      true, USB_EP_ATTR_XFER_BULK);
  if (!priv->epbulkin)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_EPBULKINALLOCFAIL), 0);
      ret = -ENODEV;
      goto errout;
    }

  priv->epbulkin->priv = priv;

  /* Pre-allocate the OUT bulk endpoint */

  priv->epbulkout =
    DEV_ALLOCEP(dev, USB_EPOUT(priv->devinfo.epno[RNDIS_EP_BULKOUT_IDX]),
                false, USB_EP_ATTR_XFER_BULK);
  if (!priv->epbulkout)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_EPBULKOUTALLOCFAIL), 0);
      ret = -ENODEV;
      goto errout;
    }

  priv->epbulkout->priv = priv;

  /* Pre-allocate read requests.  The buffer size is one full packet. */

#if defined(CONFIG_USBDEV_SUPERSPEED)
  if (dev->speed == USB_SPEED_SUPER ||
      dev->speed == USB_SPEED_SUPER_PLUS)
    {
      if (CONFIG_RNDIS_EPBULKOUT_MAXBURST < USB_SS_BULK_EP_MAXBURST)
        {
          reqlen = CONFIG_RNDIS_EPBULKOUT_SSSIZE *
                   (CONFIG_RNDIS_EPBULKOUT_MAXBURST + 1);
        }
      else
        {
          reqlen = CONFIG_RNDIS_EPBULKOUT_SSSIZE * USB_SS_BULK_EP_MAXBURST;
        }
    }
  else
#endif
#if defined(CONFIG_USBDEV_DUALSPEED)
  if (dev->speed == USB_SPEED_HIGH)
    {
      reqlen = CONFIG_RNDIS_EPBULKOUT_HSSIZE;
    }
  else
#endif
    {
      reqlen = CONFIG_RNDIS_EPBULKOUT_FSSIZE;
    }

  if (CONFIG_RNDIS_BULKOUT_REQLEN > reqlen)
    {
      reqlen = CONFIG_RNDIS_BULKOUT_REQLEN;
    }

  priv->rdreq = usbdev_allocreq(priv->epbulkout, reqlen);
  if (priv->rdreq == NULL)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_RDALLOCREQ), -ENOMEM);
      ret = -ENOMEM;
      goto errout;
    }

  priv->rdreq->callback = rndis_rdcomplete;

  /* Pre-allocate write request containers and put in a free list.
   * The buffer size should be larger than a full packet.  Otherwise,
   * we will send a bogus null packet at the end of each packet.
   *
   * Pick the larger of the max packet size and the configured request
   * size.
   */

  if (CONFIG_IOB_BUFSIZE >= CONFIG_RNDIS_BULKIN_REQLEN)
    {
      reqlen = 0;
    }
  else
    {
#if defined(CONFIG_USBDEV_SUPERSPEED)
      if (dev->speed == USB_SPEED_SUPER ||
          dev->speed == USB_SPEED_SUPER_PLUS)
        {
          if (CONFIG_RNDIS_EPBULKIN_MAXBURST < USB_SS_BULK_EP_MAXBURST)
            {
              reqlen = CONFIG_RNDIS_EPBULKIN_SSSIZE *
                       (CONFIG_RNDIS_EPBULKIN_MAXBURST + 1);
            }
          else
            {
              reqlen = CONFIG_RNDIS_EPBULKIN_SSSIZE *
                       USB_SS_BULK_EP_MAXBURST;
            }
        }
      else
    #endif
    #if defined(CONFIG_USBDEV_DUALSPEED)
      if (dev->speed == USB_SPEED_HIGH)
        {
          reqlen = CONFIG_RNDIS_EPBULKIN_HSSIZE;
        }
      else
    #endif
        {
          reqlen = CONFIG_RNDIS_EPBULKIN_FSSIZE;
        }
    }

  if (CONFIG_RNDIS_BULKIN_REQLEN > reqlen)
    {
      reqlen = CONFIG_RNDIS_BULKIN_REQLEN;
    }

  for (i = 0; i < CONFIG_RNDIS_NWRREQS; i++)
    {
      reqcontainer      = &priv->wrreqs[i];
      reqcontainer->req = usbdev_allocreq(priv->epbulkin, reqlen);

      if (reqcontainer->req == NULL)
        {
          usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_WRALLOCREQ), -ENOMEM);
          ret = -ENOMEM;
          goto errout;
        }

      reqcontainer->buf           = reqcontainer->req->buf;
      reqcontainer->req->priv     = reqcontainer;
      reqcontainer->req->callback = rndis_wrcomplete;

      while (nxmutex_lock(&priv->lock) < 0);
      sq_addlast((FAR sq_entry_t *)reqcontainer, &priv->reqlist);
      nxmutex_unlock(&priv->lock);
    }

  /* Initialize response queue to empty */

  priv->response_queue_words = 0;

  /* Report if we are selfpowered */

#ifndef CONFIG_RNDIS_COMPOSITE
#ifdef CONFIG_USBDEV_SELFPOWERED
  DEV_SETSELFPOWERED(dev);
#endif

  /* And pull-up the data line for the soft connect function */

  DEV_CONNECT(dev);
#endif

  return OK;

errout:
  usbclass_unbind(driver, dev);
  return ret;
}

/****************************************************************************
 * Name: usbclass_unbind
 *
 * Description:
 *    Invoked when the driver is unbound from a USB device driver
 *
 ****************************************************************************/

static void usbclass_unbind(FAR struct usbdevclass_driver_s *driver,
                            FAR struct usbdev_s *dev)
{
  FAR struct rndis_dev_s *priv;
  FAR struct rndis_req_s *reqcontainer;

  usbtrace(TRACE_CLASSUNBIND, 0);

#ifdef CONFIG_DEBUG_FEATURES
  if (!driver || !dev || !dev->ep0)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_INVALIDARG), 0);
      return;
    }
#endif

  /* Extract reference to private data */

  priv = ((FAR struct rndis_driver_s *)driver)->dev;

#ifdef CONFIG_DEBUG_FEATURES
  if (!priv)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_EP0NOTBOUND), 0);
      return;
    }
#endif

  /* Make sure that we are not already unbound */

  if (priv != NULL)
    {
      /* Make sure that the endpoints have been unconfigured.  If
       * we were terminated gracefully, then the configuration should
       * already have been reset.  If not, then calling usbclass_resetconfig
       * should cause the endpoints to immediately terminate all
       * transfers and return the requests to us (with result == -ESHUTDOWN)
       */

      usbclass_resetconfig(priv);
      up_mdelay(50);

      /* Free the pre-allocated control request */

      if (priv->ctrlreq != NULL)
        {
          usbdev_freereq(dev->ep0, priv->ctrlreq);
          priv->ctrlreq = NULL;
        }

      if (priv->epintin_req != NULL)
        {
          usbdev_freereq(priv->epintin, priv->epintin_req);
          priv->epintin_req = NULL;
        }

      /* Free pre-allocated read requests (which should all have
       * been returned to the free list at this time -- we don't check)
       */

      if (priv->rdreq)
        {
          usbdev_freereq(priv->epbulkout, priv->rdreq);
        }

      /* Free write requests that are not in use (which should be all
       * of them
       */

      while (nxmutex_lock(&priv->lock) < 0);
      while (!sq_empty(&priv->reqlist))
        {
          reqcontainer = (struct rndis_req_s *)sq_remfirst(&priv->reqlist);
          if (reqcontainer->req != NULL)
            {
              reqcontainer->req->buf = reqcontainer->buf;
              usbdev_freereq(priv->epbulkin, reqcontainer->req);
            }
        }

      nxmutex_unlock(&priv->lock);

      /* Free the interrupt IN endpoint */

      if (priv->epintin)
        {
          DEV_FREEEP(dev, priv->epintin);
          priv->epintin = NULL;
        }

      /* Free the bulk IN endpoint */

      if (priv->epbulkin)
        {
          DEV_FREEEP(dev, priv->epbulkin);
          priv->epbulkin = NULL;
        }

      /* Free the bulk OUT endpoint */

      if (priv->epbulkout)
        {
          DEV_FREEEP(dev, priv->epbulkout);
          priv->epbulkout = NULL;
        }
    }
}

/****************************************************************************
 * Name: usbclass_setup
 *
 * Description:
 *   Invoked for ep0 control requests.  This function probably executes
 *   in the context of an interrupt handler.
 *
 ****************************************************************************/

static int usbclass_setup(FAR struct usbdevclass_driver_s *driver,
                          FAR struct usbdev_s *dev,
                          FAR const struct usb_ctrlreq_s *ctrl,
                          FAR uint8_t *dataout, size_t outlen)
{
  FAR struct rndis_dev_s *priv;
  FAR struct usbdev_req_s *ctrlreq;
  uint16_t value;
  uint16_t len;
  int ret = -EOPNOTSUPP;

#ifdef CONFIG_DEBUG_FEATURES
  if (!driver || !dev || !dev->ep0 || !ctrl)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_INVALIDARG), 0);
      return -EIO;
    }
#endif

  /* Extract reference to private data */

  usbtrace(TRACE_CLASSSETUP, ctrl->req);
  priv = ((FAR struct rndis_driver_s *)driver)->dev;

#ifdef CONFIG_DEBUG_FEATURES
  if (!priv || !priv->ctrlreq)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_EP0NOTBOUND), 0);
      return -ENODEV;
    }
#endif
  ctrlreq = priv->ctrlreq;
  ctrlreq->priv = 0;

  /* Extract the little-endian 16-bit values to host order */

  value = GETUINT16(ctrl->value);
  len   = GETUINT16(ctrl->len);

  uinfo("type=%02x req=%02x value=%04x len=%04x\n",
        ctrl->type, ctrl->req, value, len);

  switch (ctrl->type & USB_REQ_TYPE_MASK)
    {
     /***********************************************************************
      * Standard Requests
      ***********************************************************************/

    case USB_REQ_TYPE_STANDARD:
      {
        switch (ctrl->req)
          {
          case USB_REQ_GETDESCRIPTOR:
            {
              /* The value field specifies the descriptor type in the MS byte
               * and the descriptor index in the LS byte (order is little
               * endian)
               */

              switch (ctrl->value[1])
                {
#ifndef CONFIG_RNDIS_COMPOSITE
                case USB_DESC_TYPE_DEVICE:
                  {
                    ret = usbdev_copy_devdesc(ctrlreq->buf,
                                              &g_devdesc,
                                              dev->speed);
                  }
                  break;
#endif

                /* If the serial device is used in as part of a composite
                 * device, then the configuration descriptor is provided by
                 * logic in the composite device implementation.
                 */

#ifndef CONFIG_RNDIS_COMPOSITE
#  ifdef CONFIG_USBDEV_DUALSPEED
                case USB_DESC_TYPE_OTHERSPEEDCONFIG:
#  endif /* CONFIG_USBDEV_DUALSPEED */
                case USB_DESC_TYPE_CONFIG:
                  {
                    ret = usbclass_mkcfgdesc(ctrlreq->buf, &priv->devinfo,
                                             dev->speed, ctrl->value[1]);
                  }
                  break;
#endif

#ifndef CONFIG_RNDIS_COMPOSITE
                case USB_DESC_TYPE_STRING:
                  {
                    /* index == language code. */

                    ret = usbclass_mkstrdesc(ctrl->value[0],
                                  (FAR struct usb_strdesc_s *)ctrlreq->buf);
                  }
                  break;
#endif

                default:
                  {
                    usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_GETUNKNOWNDESC),
                             value);
                  }
                  break;
                }
            }
            break;

          case USB_REQ_SETCONFIGURATION:
            {
              if (ctrl->type == 0)
                {
                  ret = usbclass_setconfig(priv, value);
                }
            }
            break;

          /* If the serial device is used in as part of a composite device,
           * then the overall composite class configuration is managed by
           * logic in the composite device implementation.
           */

#ifndef CONFIG_RNDIS_COMPOSITE
          case USB_REQ_GETCONFIGURATION:
            {
              if (ctrl->type == USB_DIR_IN)
                {
                  *(FAR uint8_t *)ctrlreq->buf = priv->config;
                  ret = 1;
                }
            }
            break;
#endif

          default:
            usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_UNSUPPORTEDSTDREQ),
                     ctrl->req);
            break;
          }
      }
      break;

    /* Class requests */

    case USB_REQ_TYPE_CLASS:
      {
        if ((ctrl->type & USB_REQ_RECIPIENT_MASK) ==
            USB_REQ_RECIPIENT_INTERFACE)
          {
            if (ctrl->req == RNDIS_SEND_ENCAPSULATED_COMMAND)
              {
                ret = rndis_handle_control_message(priv, dataout, outlen);
              }
            else if (ctrl->req == RNDIS_GET_ENCAPSULATED_RESPONSE)
              {
                if (priv->response_queue_words == 0)
                  {
                    /* No reply available is indicated with a single
                     * 0x00 byte.
                     */

                    ret = 1;
                    ctrlreq->buf[0] = 0;
                  }
                else
                  {
                    /* Retrieve a single reply from the response queue to
                     * control request buffer.
                     */

                    FAR struct rndis_response_header *hdr =
                      (struct rndis_response_header *)priv->response_queue;
                    memcpy(ctrlreq->buf, hdr, hdr->msglen);
                    ctrlreq->priv = priv;
                    ret = hdr->msglen;
                  }
              }
          }
      }
      break;

    default:
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_UNSUPPORTEDTYPE), ctrl->type);
      break;
    }

  /* Respond to the setup command if data was returned.  On an error return
   * value (ret < 0), the USB driver will stall.
   */

  if (ret >= 0)
    {
      /* Configure the response */

      ctrlreq->len   = MIN(len, ret);
      ctrlreq->flags = USBDEV_REQFLAGS_NULLPKT;

      /* Send the response -- either directly to the USB controller or
       * indirectly in the case where this class is a member of a composite
       * device.
       */

#ifndef CONFIG_RNDIS_COMPOSITE
      ret = EP_SUBMIT(dev->ep0, ctrlreq);
#else
      ret = composite_ep0submit(driver, dev, ctrlreq, ctrl);
#endif
      if (ret < 0)
        {
          usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_EPRESPQ), (uint16_t)-ret);
          ctrlreq->result = OK;
          usbclass_ep0incomplete(dev->ep0, ctrlreq);
        }
    }

  return ret;
}

/****************************************************************************
 * Name: usbclass_disconnect
 *
 * Description:
 *   Invoked after all transfers have been stopped, when the host is
 *   disconnected.  This function is probably called from the context of an
 *   interrupt handler.
 *
 ****************************************************************************/

static void usbclass_disconnect(FAR struct usbdevclass_driver_s *driver,
                                FAR struct usbdev_s *dev)
{
  FAR struct rndis_dev_s *priv;

  usbtrace(TRACE_CLASSDISCONNECT, 0);

#ifdef CONFIG_DEBUG_FEATURES
  if (!driver || !dev || !dev->ep0)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_INVALIDARG), 0);
      return;
    }
#endif

  /* Extract reference to private data */

  priv = ((FAR struct rndis_driver_s *)driver)->dev;

#ifdef CONFIG_DEBUG_FEATURES
  if (!priv)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_EP0NOTBOUND), 0);
      return;
    }
#endif

  /* Inform the "upper half" network driver that we have lost the USB
   * connection.
   */

  priv->netdev.d_ifdown(&priv->netdev);

  while (nxmutex_lock(&priv->lock) < 0);

  /* Reset the configuration */

  usbclass_resetconfig(priv);

  nxmutex_unlock(&priv->lock);

  /* Perform the soft connect function so that we will we can be
   * re-enumerated (unless we are part of a composite device)
   */

#ifndef CONFIG_RNDIS_COMPOSITE
  DEV_CONNECT(dev);
#endif
}

/****************************************************************************
 * Name: usbclass_resetconfig
 *
 * Description:
 *   Mark the device as not configured and disable all endpoints.
 *
 ****************************************************************************/

static void usbclass_resetconfig(FAR struct rndis_dev_s *priv)
{
  /* Are we configured? */

  if (priv->config != RNDIS_CONFIGIDNONE)
    {
      /* Yes.. but not anymore */

      priv->config = RNDIS_CONFIGIDNONE;

      priv->netdev.d_ifdown(&priv->netdev);

      /* Disable endpoints.  This should force completion of all pending
       * transfers.
       */

      EP_DISABLE(priv->epintin);
      EP_DISABLE(priv->epbulkin);
      EP_DISABLE(priv->epbulkout);
    }
}

/****************************************************************************
 * Name: usbclass_setconfig
 *
 * Description:
 *   Set the device configuration by allocating and configuring endpoints and
 *   by allocating and queue read and write requests.
 *
 ****************************************************************************/

static int usbclass_setconfig(FAR struct rndis_dev_s *priv, uint8_t config)
{
  struct usb_ss_epdesc_s epdesc;
  int ret = 0;

#ifdef CONFIG_DEBUG_FEATURES
  if (priv == NULL)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_INVALIDARG), 0);
      return -EIO;
    }
#endif

  if (config == priv->config)
    {
      /* Already configured -- Do nothing */

      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_ALREADYCONFIGURED), 0);
      return 0;
    }

  /* Discard the previous configuration data */

  usbclass_resetconfig(priv);

  /* Was this a request to simply discard the current configuration? */

  if (config == RNDIS_CONFIGIDNONE)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_CONFIGNONE), 0);
      return 0;
    }

  /* We only accept one configuration */

  if (config != RNDIS_CONFIGID)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_CONFIGIDBAD), 0);
      return -EINVAL;
    }

  /* Configure the IN interrupt endpoint */

  usbdev_copy_epdesc(&epdesc.epdesc,
                     priv->devinfo.epno[RNDIS_EP_INTIN_IDX],
                     priv->usbdev->speed,
                     &g_rndis_epintindesc);
  ret = EP_CONFIGURE(priv->epintin, &epdesc.epdesc, false);
  if (ret < 0)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_EPINTINCONFIGFAIL), 0);
      goto errout;
    }

  priv->epintin->priv = priv;

  /* Configure the IN bulk endpoint */

  usbdev_copy_epdesc(&epdesc.epdesc,
                     priv->devinfo.epno[RNDIS_EP_BULKIN_IDX],
                     priv->usbdev->speed,
                     &g_rndis_epbulkindesc);
  ret = EP_CONFIGURE(priv->epbulkin, &epdesc.epdesc, false);
  if (ret < 0)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_EPBULKINCONFIGFAIL), 0);
      goto errout;
    }

  priv->epbulkin->priv = priv;

  /* Configure the OUT bulk endpoint */

  usbdev_copy_epdesc(&epdesc.epdesc,
                     priv->devinfo.epno[RNDIS_EP_BULKOUT_IDX],
                     priv->usbdev->speed,
                     &g_rndis_epbulkoutdesc);
  ret = EP_CONFIGURE(priv->epbulkout, &epdesc.epdesc, true);
  if (ret < 0)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_EPBULKOUTCONFIGFAIL), 0);
      goto errout;
    }

  priv->epbulkout->priv = priv;

  /* Queue read requests in the bulk OUT endpoint */

  while (nxmutex_lock(&priv->lock) < 0);
  priv->rdreq->callback = rndis_rdcomplete;
  ret = rndis_submit_rdreq(priv);
  nxmutex_unlock(&priv->lock);
  if (ret != OK)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_RDSUBMIT), (uint16_t)-ret);
      goto errout;
    }

  /* We are successfully configured */

  priv->config = config;
  if (priv->netdev.d_ifup(&priv->netdev) == OK)
    {
      priv->netdev.d_flags |= IFF_UP;
    }

  netdev_carrier_on(&priv->netdev);

  return OK;

errout:
  usbclass_resetconfig(priv);
  return ret;
}

/****************************************************************************
 * Name: usbclass_classobject
 *
 * Description:
 *   Allocate memory for the RNDIS driver class object
 *
 * Returned Value:
 *   0 on success, negative error code on failure.
 *
 ****************************************************************************/

static int usbclass_classobject(int minor,
                                FAR struct usbdev_devinfo_s *devinfo,
                                FAR struct usbdevclass_driver_s **classdev)
{
  FAR struct rndis_alloc_s *alloc;
  FAR struct rndis_dev_s *priv;
  FAR struct rndis_driver_s *drvr;
  int ret = 0;

  /* Allocate the structures needed */

  alloc = kmm_zalloc(sizeof(struct rndis_alloc_s));
  if (!alloc)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_ALLOCDEVSTRUCT), 0);
      return -ENOMEM;
    }

  /* Convenience pointers into the allocated blob */

  priv = &alloc->dev;
  drvr = &alloc->drvr;
  *classdev = &drvr->drvr;

  nxmutex_init(&priv->lock);

  /* Get device info */

  memcpy(&priv->devinfo, devinfo, sizeof(struct usbdev_devinfo_s));

  /* Initialize the USB ethernet driver structure */

  sq_init(&priv->reqlist);
  memcpy(priv->host_mac_address, g_rndis_default_mac_addr, 6);
  priv->netdev.d_private = priv;
  priv->netdev.d_ifup = &rndis_ifup;
  priv->netdev.d_ifdown = &rndis_ifdown;
  // priv->netdev.d_txavail = &rndis_txavail;

  g_rndis_netdev = &priv->netdev;

  /* MAC address filtering is purposefully left out of this driver. Since
   * in the RNDIS USB scenario there are only two devices in the network
   * (host and us), there shouldn't be any packets received that don't
   * belong to us.
   */

  /* Initialize the USB class driver structure */

#if defined(CONFIG_USBDEV_SUPERSPEED)
  drvr->drvr.speed         = USB_SPEED_SUPER;
#elif defined(CONFIG_USBDEV_DUALSPEED)
  drvr->drvr.speed         = USB_SPEED_HIGH;
#else
  drvr->drvr.speed         = USB_SPEED_FULL;
#endif

  drvr->drvr.ops           = &g_driverops;
  drvr->dev                = priv;

  // ret = netdev_register(&priv->netdev, NET_LL_ETHERNET);
  // if (ret)
  //   {
  //     uerr("Failed to register net device");
  //   }

  return ret;
}

static void usbclass_uninitialize(FAR struct usbdevclass_driver_s *classdev)
{
  FAR struct rndis_driver_s *drvr = (FAR struct rndis_driver_s *)classdev;
  FAR struct rndis_alloc_s *alloc = (FAR struct rndis_alloc_s *)drvr->dev;

//   netdev_unregister(&drvr->dev->netdev);
  nxmutex_destroy(&drvr->dev->lock);
  kmm_free(alloc);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: usbdev_rndis_initialize
 *
 * Description:
 *   Initialize the RNDIS USB device driver.
 *
 * Input Parameters:
 *   mac_address: pointer to an array of six octets which is the MAC address
 *                of the host side of the interface. May be NULL to use the
 *                default MAC address.
 *
 * Returned Value:
 *   0 on success, -errno on failure
 *
 ****************************************************************************/

#ifndef CONFIG_RNDIS_COMPOSITE
int usbdev_rndis_initialize(FAR const uint8_t *mac_address)
{
  int ret;
  FAR struct usbdevclass_driver_s *classdev;
  FAR struct rndis_driver_s *drvr;
  struct usbdev_devinfo_s devinfo;

  memset(&devinfo, 0, sizeof(struct usbdev_devinfo_s));
  devinfo.ninterfaces                = RNDIS_NINTERFACES;
  devinfo.nstrings                   = RNDIS_NSTRIDS;
  devinfo.nendpoints                 = RNDIS_NUM_EPS;
  devinfo.epno[RNDIS_EP_INTIN_IDX]   = CONFIG_RNDIS_EPINTIN;
  devinfo.epno[RNDIS_EP_BULKIN_IDX]  = CONFIG_RNDIS_EPBULKIN;
  devinfo.epno[RNDIS_EP_BULKOUT_IDX] = CONFIG_RNDIS_EPBULKOUT;

  ret = usbclass_classobject(0, &devinfo, &classdev);
  if (ret)
    {
      nerr("usbclass_classobject failed: %d\n", ret);
      return ret;
    }

  drvr = (FAR struct rndis_driver_s *)classdev;

  if (mac_address)
    {
      memcpy(drvr->dev->host_mac_address, mac_address, 6);
    }

  ret = usbdev_register(&drvr->drvr);
  if (ret)
    {
      nerr("usbdev_register failed: %d\n", ret);
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_DEVREGISTER), (uint16_t)-ret);
      usbclass_uninitialize(classdev);
      return ret;
    }

  return OK;
}
#endif

/****************************************************************************
 * Name: usbdev_rndis_set_host_mac_addr
 *
 * Description:
 *   Set host MAC address. Mainly for use with composite devices where
 *   the MAC cannot be given directly to usbdev_rndis_initialize().
 *
 * Input Parameters:
 *   netdev:      pointer to the network interface. Can be obtained from
 *                e.g. netdev_findbyname().
 *
 *   mac_address: pointer to an array of six octets which is the MAC address
 *                of the host side of the interface. May be NULL to use the
 *                default MAC address.
 *
 * Returned Value:
 *   0 on success, -errno on failure
 *
 ****************************************************************************/

int usbdev_rndis_set_host_mac_addr(FAR struct net_driver_s *netdev,
                                   FAR const uint8_t *mac_address)
{
  FAR struct rndis_dev_s *dev = (FAR struct rndis_dev_s *)netdev;

  if (mac_address)
    {
      memcpy(dev->host_mac_address, mac_address, 6);
    }
  else
    {
      memcpy(dev->host_mac_address, g_rndis_default_mac_addr, 6);
    }

  return OK;
}

/****************************************************************************
 * Name: usbdev_rndis_get_composite_devdesc
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

#ifdef CONFIG_RNDIS_COMPOSITE
void usbdev_rndis_get_composite_devdesc(struct composite_devdesc_s *dev)
{
  memset(dev, 0, sizeof(struct composite_devdesc_s));

  /* The callback functions for the RNDIS class.
   *
   * classobject() and uninitialize() must be provided by board-specific
   * logic
   */

  dev->mkconfdesc          = usbclass_mkcfgdesc;
  dev->mkstrdesc           = usbclass_mkstrdesc;
  dev->classobject         = usbclass_classobject;
  dev->uninitialize        = usbclass_uninitialize;

  dev->nconfigs            = RNDIS_NCONFIGS; /* Number of configurations supported  */
  dev->configid            = RNDIS_CONFIGID; /* The only supported configuration ID */

  /* Let the construction function calculate the size of config descriptor */

  dev->cfgdescsize         = usbclass_mkcfgdesc(NULL, NULL,
                                                USB_SPEED_UNKNOWN, 0);

  /* Board-specific logic must provide the device minor */

  /* Interfaces.
   *
   * ifnobase must be provided by board-specific logic
   */

  dev->devinfo.ninterfaces = RNDIS_NINTERFACES;

  /* Strings.
   *
   * strbase must be provided by board-specific logic
   */

  dev->devinfo.nstrings    = 0;

  /* Endpoints.
   *
   * Endpoint numbers must be provided by board-specific logic.
   */

  dev->devinfo.nendpoints  = RNDIS_NUM_EPS;

  FAR struct rndis_netpkt_s *packetcontainer;

  sq_init(&rndis_free_netpkt_lst);
  sq_init(&rndis_pending_netpkt_lst);

  for (int i = 0; i < CONFIG_RNDIS_NWRREQS; i++)
    {
      packetcontainer = &rndis_netpackets[i];

      irqstate_t flags = enter_critical_section();
      sq_addlast((FAR sq_entry_t *)packetcontainer, &rndis_free_netpkt_lst);
      leave_critical_section(flags);
    }

  int pid = task_create("rndis_rxdispatch", 128, CONFIG_DEFAULT_TASK_STACKSIZE, rndis_rxdispatch_task_run, NULL);
  if (pid < 0)
  {
    uerr("ERROR: Failed to start the rndis_rxdispatch task.\n");
  }

}
#endif
