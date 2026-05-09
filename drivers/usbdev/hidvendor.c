/****************************************************************************
 * drivers/usbdev/hidvendor.c
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
#include <nuttx/usb/hidvendor.h>
#include <nuttx/kmalloc.h>
#include <nuttx/wqueue.h>
#include <nuttx/fs/fs.h>
#include <poll.h>
#include <debug.h>
#include <errno.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

/****************************************************************************
 * Pre-processor definitions
 ****************************************************************************/

#define USBHID_CHARDEV_PATH         "/dev/usbhid"

#define HID_CLASS_VERSION           (0x0111)

#define HID_OUT_REPORTID            (0x01)
#define HID_IN_REPORTID             (0x02)

/* Buffer big enough for any of our descriptors (the config descriptor is the
 * biggest).
 */

#define HID_DATA_LENGTH             (CONFIG_USBHID_EPINT_SIZE - 1)

#define RX_DATA_LEN_MAX             (CONFIG_USBHID_EPINT_SIZE * 2)

#define USBHID_MXDESCLEN            (128)
#define USBHID_MAXSTRLEN            (USBHID_MXDESCLEN - 2)


#define USBHID_INTERFACESTRID       (1)
#define USBHID_NSTRIDS              (1)

#define USBHID_NCONFIGS             (1)

#ifndef CONFIG_USBHID_NRDREQS
#define CONFIG_USBHID_NRDREQS       (1)
#endif

#ifndef CONFIG_USBHID_NWRREQS
#define CONFIG_USBHID_NWRREQS       (1)
#endif

/* Container to support a list of requests */

struct hid_wrreq_s
{
    FAR struct hid_wrreq_s *flink;          /* Implements a singly linked list */
    FAR struct usbdev_req_s *req;           /* The contained request */
};

struct hid_rdreq_s
{
    FAR struct hid_rdreq_s *flink;          /* Implements a singly linked list */
    FAR struct usbdev_req_s *req;           /* The contained request */
};

struct hid_report_s
{
  uint8_t report_id;                        /* reportid */
  uint8_t payload[HID_DATA_LENGTH];     /* payload length */
};

struct hidvendor_driver_s
{
    struct usbdevclass_driver_s drvr;
    struct usbdev_devinfo_s  devinfo;
    struct work_s work;
    uint8_t nwrq;                           /* Number of queue write requests (in txfree) */

    bool linked;                            /* Indicates if the driver has been linked */
    uint8_t crefs;                          /* Count of opened instances */
    mutex_t lock;                           /* Enforces device exclusive access */

    FAR struct usbdev_req_s *ctrlreq;       /* Pointer to preallocated control request */
    FAR struct usbdev_ep_s *epintin;        /* Interrupt IN endpoint structure */
    FAR struct usbdev_ep_s *epintout;       /* Interrupt OUT endpoint structure */
    struct sq_queue_s txfree;               /* Available write request containers */
    struct sq_queue_s rxdata;               /* Received data queue */

    /* Pre-allocated write request containers.  The write requests will
     * be linked in a free list (txfree), and used to send requests to
     * EPINTIN.
     */

    struct hid_wrreq_s wrreqs[CONFIG_USBHID_NWRREQS];
    FAR struct hid_wrreq_s *wrcontainer;

    struct hid_rdreq_s rdreqs[CONFIG_USBHID_NRDREQS];
    FAR struct hid_rdreq_s *rdcontainer;

    /* Received data buffer cache*/
    uint8_t rxbuffer[RX_DATA_LEN_MAX];
    size_t rxlen;
    bool rxpending;

    /* Poll support */
    FAR struct pollfd *poll_fds;
    bool poll_waiting;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Completion event handlers ************************************************/

static void usbclass_ep0incomplete(FAR struct usbdev_ep_s *ep,
                 FAR struct usbdev_req_s *req);
static void hidep_wrcomplete(FAR struct usbdev_ep_s *ep,
                 FAR struct usbdev_req_s *req);
static void hidep_rdcomplete(FAR struct usbdev_ep_s *ep,
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
static void usbclass_suspend(FAR struct usbdevclass_driver_s *driver,
                                FAR struct usbdev_s *dev);
static void usbclass_resume(FAR struct usbdevclass_driver_s *driver,
                                FAR struct usbdev_s *dev);

/* Char device Operations ***************************************************/

static int usbdev_fs_open(FAR struct file *filep);
static int usbdev_fs_close(FAR struct file *filep);
static ssize_t usbdev_fs_read(FAR struct file *filep, FAR char *buffer,
                              size_t len);
static ssize_t usbdev_fs_write(FAR struct file *filep,
                               FAR const char *buffer, size_t len);
static int usbdev_fs_poll(FAR struct file *filep, FAR struct pollfd *fds,
                          bool setup);

/* Transfer helpers *********************************************************/

static int hidvendor_sndpacket(FAR struct hidvendor_driver_s *priv, FAR const uint8_t *buffer);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* USB driver operations */

const static struct usbdevclass_driverops_s g_hidvendor_driverops =
{
    &usbclass_bind,
    &usbclass_unbind,
    &usbclass_setup,
    &usbclass_disconnect,
    &usbclass_suspend,
    &usbclass_resume
};

/* Char device **************************************************************/

static const struct file_operations g_hidvendor_fs_fops =
{
    usbdev_fs_open,         /* open */
    usbdev_fs_close,        /* close */
    usbdev_fs_read,         /* read */
    usbdev_fs_write,        /* write */
    NULL,                   /* seek */
    NULL,                   /* ioctl */
    NULL,                   /* mmap */
    NULL,                   /* truncate */
    usbdev_fs_poll          /* poll */
};

/* HID repost descriptor ***********************************************************/
static const uint8_t hidvendor_report_descriptor[] = {
    0x06,0x00,0xFF,         // USAGE PAGE  (Vendor 0xFF07)
    0x0A,0x22,0x02,         // USAGE (0x0222)
    0xA1,0x01,              // COLLECTION (Application)
    0x85,HID_OUT_REPORTID,  // REPORT ID (OUTPUT 0x1)
    0x75,0x08,              // REPORT SIZE (8)
    0x95,HID_DATA_LENGTH,   // REPORT COUNT (63)
    0x15,0x00,              // LOGICAL_MINIMUM (00)
    0x25,0xFF,              // LOGICAL_MAXIMUM (FF)
    0x09,0x01,              // USAGE (Generic Value)
    0x91,0x02,              // OUTPUT (Data,Var,Abs)
    0x85,HID_IN_REPORTID,   // REPORT ID (INPUT 0x2)
    0x95,HID_DATA_LENGTH,   // REPORT COUNT (63)
    0x09,0x02,              // USAGE (Generic Value)
    0x81,0x02,              //INPUT (Data Variable Absolute)
    0xC0,                   // END_COLLECTION (Application)
};

static struct usb_iaddesc_s g_iaddesc =
{
    .len      = USB_SIZEOF_IADDESC,
    .type     = USB_DESC_TYPE_INTERFACEASSOCIATION,
    .firstif  = 0,
    .nifs     = 1,
    .classid  = USB_CLASS_HID,    // 0x03，HID
    .subclass = 0x00,             // 0x00
    .protocol = 0x00,             // 0x00
    .ifunction = 0
};

static const struct usb_ifdesc_s g_hid_ifdesc =
{
    .len      = USB_SIZEOF_IFDESC,
    .type     = USB_DESC_TYPE_INTERFACE,
    .ifno     = 0,
    .alt      = 0,
    .neps     = 2,                /* 2 ep */
    .classid  = USB_CLASS_HID,    // 0x03，HID
    .subclass = 0x00,             // 0x00
    .protocol = 0x00,             // 0x00
    .iif      = 0
};

static const struct usbhid_descriptor_s g_hiddesc =
{
    .len          = sizeof(struct usbhid_descriptor_s),
    .type         = USBHID_DESCTYPE_HID,
    .hid[0]       = LSBYTE(HID_CLASS_VERSION),
    .hid[1]       = MSBYTE(HID_CLASS_VERSION),
    .country      = 0x00,
    .ndesc        = 0x01,
    .classdesc    = USBHID_DESCTYPE_REPORT,
    .desclen[0]   = LSBYTE(sizeof(hidvendor_report_descriptor)),
    .desclen[1]   = MSBYTE(sizeof(hidvendor_report_descriptor))
};

static const struct usbdev_epinfo_s g_epintin =
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
        .interval  = CONFIG_USBHID_FSEPINT_INTERVAL,
#else
        .interval  = CONFIG_USBHID_HSEPINT_INTERVAL,
#endif
    },
    .reqnum        = CONFIG_USBHID_NWRREQS,
    .fssize        = CONFIG_USBHID_EPINT_SIZE,
#ifdef CONFIG_USBDEV_DUALSPEED
    .hssize        = CONFIG_USBHID_EPINT_SIZE,
#endif
};

static const struct usbdev_epinfo_s g_epintout =
{
    .desc =
    {
        .len       = USB_SIZEOF_EPDESC,
        .type      = USB_DESC_TYPE_ENDPOINT,
        .addr      = USB_DIR_OUT,
        .attr      = USB_EP_ATTR_XFER_INT |
                     USB_EP_ATTR_NO_SYNC   |
                     USB_EP_ATTR_USAGE_DATA,
#ifdef CONFIG_USBDEV_DUALSPEED
        .interval  = CONFIG_USBHID_FSEPINT_INTERVAL,
#else
        .interval  = CONFIG_USBHID_HSEPINT_INTERVAL,
#endif
    },
    .reqnum        = CONFIG_USBHID_NRDREQS,
    .fssize        = CONFIG_USBHID_EPINT_SIZE,
#ifdef CONFIG_USBDEV_DUALSPEED
    .hssize        = CONFIG_USBHID_EPINT_SIZE,
#endif
};

static const FAR struct usbdev_epinfo_s *g_hidvendor_epinfos[USBHID_VENDOR_NUM_EPS] =
{
    &g_epintin,
    &g_epintout,
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
    FAR struct hidvendor_driver_s *priv = inode->i_private;
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
    FAR struct hidvendor_driver_s *priv = inode->i_private;
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
 *   Read usbdev fs device - (HOST->DEVICE)
 *
 ****************************************************************************/

static ssize_t usbdev_fs_read(FAR struct file *filep, FAR char *buffer,
                              size_t len)
{
    FAR struct inode *inode = filep->f_inode;
    FAR struct hidvendor_driver_s *priv = inode->i_private;
    size_t copylen;
    int ret;

    uinfo("usbdev_fs_read: entry, len=%zu, rxlen=%zu, rxpending=%d\n",
        len, priv->rxlen, priv->rxpending);

    ret = nxmutex_lock(&priv->lock);
    if (ret < 0)
    {
        uerr("usbdev_fs_read: nxmutex_lock failed: %d\n", ret);
        return ret;
    }

    /* Check if there is any data to read */
    if (priv->rxlen == 0)
    {
        uinfo("usbdev_fs_read: no data available, rxlen=0\n");
        nxmutex_unlock(&priv->lock);
        return 0;  /* No data to read */
    }

    /* Copy data to user buffer */
    copylen = (len < priv->rxlen) ? len : priv->rxlen;
    memcpy(buffer, priv->rxbuffer, copylen);

    uinfo("usbdev_fs_read: copied %zu bytes to user buffer\n", copylen);

    /* If there is remaining data in the buffer, move it to the front */
    if (copylen < priv->rxlen)
    {
        memmove(priv->rxbuffer, priv->rxbuffer + copylen, priv->rxlen - copylen);
        priv->rxlen -= copylen;
        priv->rxpending = true; /* Still data pending */
        uinfo("usbdev_fs_read: %zu bytes remaining in buffer\n", priv->rxlen);
    }
    else
    {
        /* All data has been read */
        priv->rxlen = 0;
        priv->rxpending = false;
        uinfo("usbdev_fs_read: buffer is now empty\n");
    }

    nxmutex_unlock(&priv->lock);

    uinfo("usbdev_fs_read: returning %zu bytes\n", copylen);
    return copylen;
}

/****************************************************************************
 * Name: usbdev_fs_write
 *
 * Description:
 *   Write usbdev fs device. (DEVICE -> HOST)
 *
 ****************************************************************************/

static ssize_t usbdev_fs_write(FAR struct file *filep,
                               FAR const char *buffer, size_t len)
{
    FAR struct inode *inode = filep->f_inode;
    FAR struct hidvendor_driver_s *priv = inode->i_private;
    int ret;

    ret = nxmutex_lock(&priv->lock);
    if (ret < 0)
    {
        goto errout;
    }

    if (len > (CONFIG_USBHID_EPINT_SIZE - 1))
    {
        len = CONFIG_USBHID_EPINT_SIZE - 1;
    }

    len = hidvendor_sndpacket(priv, (FAR const uint8_t *)buffer);

    uinfo("wrote %u bytes, nwrq %d\n", len, priv->nwrq);


errout:
    nxmutex_unlock(&priv->lock);

    return len;
}

/****************************************************************************
 * Name: usbdev_fs_poll
 *
 * Description:
 *   Poll for data availability. This function is used to determine if
 *   there is data available for reading or if the device is ready for
 *   writing.
 *
 ****************************************************************************/

static int usbdev_fs_poll(FAR struct file *filep, FAR struct pollfd *fds,
                          bool setup)
{
    FAR struct inode *inode = filep->f_inode;
    FAR struct hidvendor_driver_s *priv = inode->i_private;
    pollevent_t eventset = 0;
    int ret;;

    uinfo("usbdev_fs_poll: entry, setup=%d\n", setup);

    ret = nxmutex_lock(&priv->lock);
    if (ret < 0)
    {
        uerr("usbdev_fs_poll: nxmutex_lock failed: %d\n", ret);
        return ret;
    }

    if (!priv->linked)
    {
        uinfo("usbdev_fs_poll: device not connected\n");
        nxmutex_unlock(&priv->lock);
        return setup ? -ENOTCONN : OK;
    }

    if (setup)
    {
        /* Setup the poll structure */

        /* Save the poll file descriptor for later notification */
        priv->poll_fds = fds;
        priv->poll_waiting = true;

        /* Check if data is available for reading */
        if (fds->events & POLLIN)
        {
            if (priv->rxpending && priv->rxlen > 0)
            {
                /* Data is available for immediate reading */
                eventset |= POLLIN;
                uinfo("usbdev_fs_poll: data available for reading, rxlen=%zu\n",
                    priv->rxlen);
            }
            else
            {
                /* No data available, need to wait */
                uinfo("usbdev_fs_poll: no data available, waiting for notification\n");
            }
        }

        /* Check if device is ready for writing */
        if (fds->events & POLLOUT)
        {
            /* Check if we have free write requests available */
            if (priv->nwrq > 0)
            {
                /* Device is ready for writing */
                eventset |= POLLOUT;
                uinfo("usbdev_fs_poll: device ready for writing, nwrq=%d\n",
                    priv->nwrq);
            }
            else
            {
                uinfo("usbdev_fs_poll: device not ready for writing, nwrq=%d\n",
                    priv->nwrq);
            }
        }

        /* Check for hangup/error conditions */
        if (!priv->linked)
        {
            eventset |= POLLHUP;
        }

        if (eventset != 0)
        {
            /* Immediately notify the caller if any events are ready */
            poll_notify(&fds, 1, eventset);
            syslog(0, "usbdev_fs_poll: immediate notification, eventset=0x%x\n",
                 eventset);
            /* Clear waiting state since we're notifying immediately */
            priv->poll_waiting = false;
        }
    }
    else
    {
        /* Teardown the poll */
        uinfo("usbdev_fs_poll: teardown\n");
        priv->poll_fds = NULL;
        priv->poll_waiting = false;
    }

    nxmutex_unlock(&priv->lock);

    uinfo("usbdev_fs_poll: returning, eventset=0x%x\n", eventset);
    return setup ? OK : eventset;
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
 * Name: hidvendor_sndpacket
 *
 * Description:
 *   This function obtains write requests, transfers the TX data into the
 *   request, and submits the requests to the USB controller.  This
 *   continues until either (1) there are no further packets available, or
 *   (2) there is no further data to send.
 *
 ****************************************************************************/

static int hidvendor_sndpacket(FAR struct hidvendor_driver_s *priv, FAR const uint8_t *buffer)
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
    report               = (FAR struct hid_report_s *)req->buf;

    report->report_id    = HID_IN_REPORTID;
    memcpy(report->payload, buffer, (CONFIG_USBHID_EPINT_SIZE - 1));

    /* Then submit the request to the endpoint */

    req->len             = CONFIG_USBHID_EPINT_SIZE;
    req->priv            = wrcontainer;
    req->flags           = USBDEV_REQFLAGS_NULLPKT;
    ret                  = EP_SUBMIT(ep, req);

    uinfo("report_id:%d report->payload:%0x %0x ...\n", report->report_id, buffer[0], buffer[1]);

    if (ret < 0)
    {
        usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_SUBMITFAIL), (uint16_t)-ret);
    }
    else
    {
        ret = req->len - 1; // report ID 1 byte
    }

errout_with_flags:

    leave_critical_section(flags);
    uinfo("ep_submit ret %d \n", ret);
    return ret;
}

/****************************************************************************
 * Name: hidep_rdcomplete
 *
 * Description:
 *   Handle completion of read request (OUT endpoint). This function
 *   processes data received from the host.
 *
 ****************************************************************************/

static void hidep_rdcomplete(FAR struct usbdev_ep_s *ep,
                              FAR struct usbdev_req_s *req)
{
    FAR struct hidvendor_driver_s *priv;
    FAR struct hid_report_s *report;
    int ret;

    /* Extract references to our private data */

    priv = (FAR struct hidvendor_driver_s *)ep->priv;

    uinfo("rdcomplete: len=%d, result=%d\n", req->xfrd, req->result);
    syslog(0, "rdcomplete: len=%d, result=%d\n", req->xfrd, req->result);

    switch (req->result)
    {
        case OK: /* Normal completion */
        {
            if (req->xfrd > 0)
            {
                report = (FAR struct hid_report_s *)req->buf;
                size_t new_data_len = req->xfrd > 0 ? req->xfrd - 1 : 0; /* -1 for report ID */

                /* Check if there is space in the buffer to append new data */
                if (priv->rxlen + new_data_len <= RX_DATA_LEN_MAX)
                {
                    /* Append new data to the buffer */
                    memcpy(priv->rxbuffer + priv->rxlen, report->payload, new_data_len);
                    priv->rxlen += new_data_len;
                    priv->rxpending = true;

                    uinfo("Received %d bytes, buffer now has %d bytes\n", (int)new_data_len, (int)priv->rxlen);

                    /* Notify any waiting poll() callers that data is available */
                    if (priv->poll_waiting && priv->poll_fds != NULL)
                    {
                        if (priv->poll_fds->events & POLLIN)
                        {
                            poll_notify(&priv->poll_fds, 1, POLLIN);
                            uinfo("hidep_rdcomplete: Notified poll() caller about incoming data\n");
                            priv->poll_waiting = false; /* Clear waiting state after notification */
                        }
                    }
                }
                else
                {
                    uerr("RX buffer full, dropping %d bytes\n", (int)new_data_len);
                }
            }

            /* Resubmit the read request */
            req->len = CONFIG_USBHID_EPINT_SIZE;
            req->result = OK;  /* Clear any error status */
            ret = EP_SUBMIT(ep, req);
            if (ret < 0)
            {
                uerr("hidep_rdcomplete: Failed to resubmit OUT request, ret=%d\n", ret);
                usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_RDSUBMIT), (uint16_t)-ret);
            }
            usbtrace(TRACE_CLASSRDCOMPLETE, priv->nwrq);
        }
        break;

        case -ESHUTDOWN: /* Disconnection */
        {
            usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_RDSHUTDOWN), 0);
        }
        break;

        default: /* Some other error occurred */
        {
            usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_RDUNEXPECTED),
                     (uint16_t)-req->result);
            syslog(0, "hidep_rdcomplete: Unexpected error %d\n", req->result);
        }
        break;
    }
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
    FAR struct hidvendor_driver_s *priv;
    FAR struct hid_wrreq_s *wrcontainer;
    irqstate_t flags;

    /* Extract references to our private data */

    priv        = (FAR struct hidvendor_driver_s *)ep->priv;
    wrcontainer = (FAR struct hid_wrreq_s *)req->priv;

    /* Return the write request to the free list */

    flags = enter_critical_section();
    sq_addlast((FAR sq_entry_t *)wrcontainer, &priv->txfree);
    priv->nwrq++;
    leave_critical_section(flags);

    uinfo("nwrq %d", priv->nwrq);

    /* Notify any waiting poll() callers that device is ready for writing */
    if (priv->poll_waiting && priv->poll_fds != NULL)
    {
        if (priv->poll_fds->events & POLLOUT)
        {
            poll_notify(&priv->poll_fds, 1, POLLOUT);
            uinfo("hidep_wrcomplete: Notified poll() caller about write readiness\n");
            priv->poll_waiting = false; /* Clear waiting state after notification */
        }
    }

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

    g_iaddesc.firstif = devinfo->ifnobase;
    memcpy(iaddesc, &g_iaddesc, sizeof(g_iaddesc));
    totallen += sizeof(struct usb_iaddesc_s);

    memcpy(itfdesc, &g_hid_ifdesc, sizeof(g_hid_ifdesc));
    totallen += sizeof(struct usb_ifdesc_s);

    memcpy(hiddesc, &g_hiddesc, sizeof(g_hiddesc));
    totallen += sizeof(struct usbhid_descriptor_s);

    ret = usbdev_copy_epdesc((FAR struct usb_epdesc_s *)epdesc,
                             devinfo->epno[USBVENDOR_EP_INTIN_IDX],
                             speed, &g_epintin);
    totallen += ret;
    epdesc += ret;

    ret = usbdev_copy_epdesc((FAR struct usb_epdesc_s *)epdesc,
                             devinfo->epno[USBVENDOR_EP_INTOUT_IDX],
                             speed, &g_epintout);
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

        case USBHID_INTERFACESTRID:
            str = CONFIG_USBHID_INTERFACESTR;
            break;

        default:
            return -EINVAL;
    }

    /* The string is utf16-le.  The poor man's utf-8 to utf16-le
     * conversion below will only handle 7-bit en-us ascii
     */

    len = strlen(str);
    if (len > (USBHID_MAXSTRLEN / 2))
    {
        len = (USBHID_MAXSTRLEN / 2);
    }

    for (i = 0, ndata = 0; i < len; i++, ndata += 2)
    {
        data[ndata] = str[i];
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
    FAR struct hidvendor_driver_s *alloc;

    alloc = kmm_zalloc(sizeof(struct hidvendor_driver_s));
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
    alloc->drvr.ops = &g_hidvendor_driverops;

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
 *   Satrt OUT rx
 *
 ****************************************************************************/

static void usbdev_register_driver(FAR void *arg)
{
    FAR struct hidvendor_driver_s *priv = arg;
    FAR struct hid_rdreq_s *rdcontainer;
    FAR struct usbdev_req_s *req;
    int ret;

    syslog(0, "hid: Registering device driver at %s\n", USBHID_CHARDEV_PATH);
    ret = register_driver(USBHID_CHARDEV_PATH, &g_hidvendor_fs_fops, 0666, priv);
    if (ret < 0)
    {
        syslog(0, "hid: ERROR: Failed to register driver, ret=%d\n", ret);
        return;
    }
    syslog(0, "hid: Device driver registered successfully\n");

    priv->rxlen = 0;
    priv->rxpending = false;

    priv->poll_fds = NULL;
    priv->poll_waiting = false;

    if (priv->epintout && CONFIG_USBHID_NRDREQS > 0)
    {
        for (int i = 0; i < CONFIG_USBHID_NRDREQS; i++)
        {
            rdcontainer = &priv->rdreqs[i];
            req = rdcontainer->req;
            if (req != NULL)
            {
                req->len = CONFIG_USBHID_EPINT_SIZE;
                req->priv = priv;
                ret = EP_SUBMIT(priv->epintout, req);
                if (ret < 0)
                {
                    uerr("Failed to submit OUT request %d, ret=%d\n", i, ret);
                    break;
                }
                uinfo("OUT endpoint request %d ready for receiving\n", i);
            }
        }
    }

    priv->linked = true;
}

static int  usbclass_setup(FAR struct usbdevclass_driver_s *driver,
                           FAR struct usbdev_s *dev,
                           FAR const struct usb_ctrlreq_s *ctrl,
                           FAR uint8_t *dataout, size_t outlen)
{
    FAR struct hidvendor_driver_s *priv = (FAR struct hidvendor_driver_s *)driver;
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

    /* Log detailed setup request information */
    syslog(0, "hid: Setup request - type=0x%02x, req=0x%02x, value=0x%04x, index=0x%04x, len=%d\n",
           ctrl->type, ctrl->req, value, index, len);

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
                    ret = sizeof(hidvendor_report_descriptor);
                    memcpy(ctrlreq->buf, &hidvendor_report_descriptor, ret);
                }
            }
            break;

            case USB_REQ_SETCONFIGURATION:
            {
                /* activate IN endpoint */
                usbdev_copy_epdesc(&epdesc.epdesc, devinfo->epno[USBVENDOR_EP_INTIN_IDX],
                             priv->drvr.speed, &g_epintin);
                EP_CONFIGURE(priv->epintin, &epdesc.epdesc, false);

                /* activate OUT endpoint for receiving data from host */
                usbdev_copy_epdesc(&epdesc.epdesc, devinfo->epno[USBVENDOR_EP_INTOUT_IDX],
                             priv->drvr.speed, &g_epintout);
                EP_CONFIGURE(priv->epintout, &epdesc.epdesc, false);

                /* Queue device registration work */
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
    FAR struct hidvendor_driver_s *priv = (FAR struct hidvendor_driver_s *)driver;
    FAR struct hid_wrreq_s *wrcontainer;
    irqstate_t flags;
    size_t reqlen;
    int ret;
    int i;

    syslog(0, "hid: usbclass_bind enter, speed=%d\n",priv->drvr.speed);

    /* Initialize lock */

    nxmutex_init(&priv->lock);

    /* Preallocate control request */
    priv->ctrlreq = usbdev_allocreq(dev->ep0, USBHID_MAXSTRLEN);
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

    syslog(0, "hid: Allocating IN endpoint %d (0x%02x)\n",
           priv->devinfo.epno[USBVENDOR_EP_INTIN_IDX],
           HIDVENDOR_MKEPINTIN(&priv->devinfo));
    priv->epintin = DEV_ALLOCEP(dev, HIDVENDOR_MKEPINTIN(&priv->devinfo),
                                true, USB_EP_ATTR_XFER_INT);
    if (!priv->epintin)
    {
        syslog(0, "hid: ERROR: Failed to allocate IN endpoint %d (0x%02x), attr=0x%02x\n",
               priv->devinfo.epno[USBVENDOR_EP_INTIN_IDX],
               HIDVENDOR_MKEPINTIN(&priv->devinfo),
               USB_EP_ATTR_XFER_INT);
        usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_EPINTINALLOCFAIL), 0);
        ret = -ENODEV;
        goto errout;
    }
    syslog(0, "hid: IN endpoint %d allocated successfully\n",
           priv->devinfo.epno[USBVENDOR_EP_INTIN_IDX]);

    priv->epintin->priv = priv;

    /* Pre-allocate the OUT interrupt endpoint */

    syslog(0, "hid: Allocating OUT endpoint %d (0x%02x)\n",
           priv->devinfo.epno[USBVENDOR_EP_INTOUT_IDX],
           HIDVENDOR_MKEPINTOUT(&priv->devinfo));
    priv->epintout = DEV_ALLOCEP(dev, HIDVENDOR_MKEPINTOUT(&priv->devinfo),
                                 false, USB_EP_ATTR_XFER_INT);
    if (!priv->epintout)
    {
        syslog(0, "hid: ERROR: Failed to allocate OUT endpoint %d (0x%02x), attr=0x%02x\n",
               priv->devinfo.epno[USBVENDOR_EP_INTOUT_IDX],
               HIDVENDOR_MKEPINTOUT(&priv->devinfo),
               USB_EP_ATTR_XFER_INT);
        //usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_EPINTOUTALLOCFAIL), 0);
        ret = -ENODEV;
        goto errout;
    }
    syslog(0, "hid: OUT endpoint %d allocated successfully\n",
           priv->devinfo.epno[USBVENDOR_EP_INTOUT_IDX]);

    priv->epintout->priv = priv;

    /* Pre-allocate write request containers and put in a free list.  The
     * buffer size should be larger than a full build IN packet.  Otherwise,
     * we will send a bogus null packet at the end of each packet.
     *
     * Pick the larger of the max packet size and the configured request size.
     *
     * NOTE: These write requests are sized for the bulk IN endpoint but are
     * shared with interrupt IN endpoint which does not need a large buffer.
     */


    reqlen = CONFIG_USBHID_EPINT_SIZE;

    for (i = 0; i < CONFIG_USBHID_NWRREQS; i++)
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

    /* Pre-allocate read request containers for OUT endpoint */
    for (i = 0; i < CONFIG_USBHID_NRDREQS; i++)
    {
        FAR struct hid_rdreq_s *rdcontainer = &priv->rdreqs[i];
        rdcontainer->req = usbdev_allocreq(priv->epintout, reqlen);
        if (rdcontainer->req == NULL)
        {
            usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_RDALLOCREQ), -ENOMEM);
            ret = -ENOMEM;
            goto errout;
        }

        rdcontainer->req->priv     = rdcontainer;
        rdcontainer->req->callback = hidep_rdcomplete;
    }

    syslog(0, "hid: usbclass_bind completed successfully\n");
    return OK;

errout:
    syslog(0, "hid: usbclass_bind failed with error %d\n", ret);
    usbclass_unbind(driver, dev);
    return ret;
}

static void usbclass_unbind(FAR struct usbdevclass_driver_s *driver,
                            FAR struct usbdev_s *dev)
{
    FAR struct hidvendor_driver_s *priv = (FAR struct hidvendor_driver_s *)driver;
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
    /* Note: In case of bind failure, nwrq might not equal CONFIG_USBHID_NWRREQS */
    syslog(0, "hid: unbind - nwrq=%d, expected=%d\n", priv->nwrq, CONFIG_USBHID_NWRREQS);

    while (!sq_empty(&priv->txfree))
    {
        wrcontainer = (struct hid_wrreq_s *)sq_remfirst(&priv->txfree);
        if (wrcontainer->req != NULL)
        {
            usbdev_freereq(priv->epintin, wrcontainer->req);
            priv->nwrq--;     /* Number of write requests queued */
        }
    }

    if (priv->nwrq != 0)
    {
        syslog(0, "lnv_hid: WARNING - nwrq=%d after cleanup, expected 0\n", priv->nwrq);
    }
    leave_critical_section(flags);

    /* Free read requests */

    for (i = 0; i < CONFIG_USBHID_NRDREQS; i++)
    {
        FAR struct hid_rdreq_s *rdcontainer = &priv->rdreqs[i];
        if (rdcontainer->req != NULL)
        {
            usbdev_freereq(priv->epintout, rdcontainer->req);
            rdcontainer->req = NULL;
        }
    }

    priv->linked = false;

    nxmutex_destroy(&priv->lock);

    if (priv->epintin)
    {
        DEV_FREEEP(dev, priv->epintin);
        priv->epintin = NULL;
    }

    if (priv->epintout)
    {
        DEV_FREEEP(dev, priv->epintout);
        priv->epintout = NULL;
    }
}

static void usbclass_disconnect(FAR struct usbdevclass_driver_s *driver,
                                FAR struct usbdev_s *dev)
{
    FAR struct hidvendor_driver_s *priv = (FAR struct hidvendor_driver_s *)driver;

    priv->linked = false;

    /* Notify any waiting poll() callers about disconnection */
    if (priv->poll_waiting && priv->poll_fds != NULL)
    {
        poll_notify(&priv->poll_fds, 1, POLLHUP);
        uinfo("usbclass_disconnect: Notified poll() caller about disconnection\n");
        priv->poll_waiting = false;
        priv->poll_fds = NULL;
    }
}

static void usbclass_suspend(FAR struct usbdevclass_driver_s *driver,
                                FAR struct usbdev_s *dev)
{
    FAR struct hidvendor_driver_s *priv = (FAR struct hidvendor_driver_s *)driver;

    syslog(0, "hid: usbclass_suspend\n");
    priv->linked = false;

    /* Notify any waiting poll() callers about disconnection */
    if (priv->poll_waiting && priv->poll_fds != NULL)
    {
        poll_notify(&priv->poll_fds, 1, POLLHUP);
        uinfo("usbclass_suspend: Notified poll() caller about suspend\n");
        priv->poll_waiting = false;
        priv->poll_fds = NULL;
    }
}

static void usbclass_resume(FAR struct usbdevclass_driver_s *driver,
                                FAR struct usbdev_s *dev)
{
    FAR struct hidvendor_driver_s *priv = (FAR struct hidvendor_driver_s *)driver;

    syslog(0, "lnv_hid: usbclass_resume\n");
    priv->linked = true;
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

static void usbclass_uninitialize(FAR struct usbdevclass_driver_s *classdev)
{
    FAR struct hidvendor_driver_s *drvr = (FAR struct hidvendor_driver_s *)classdev;
    int ret;

    /* Un-register the device */

    ret = unregister_driver(USBHID_CHARDEV_PATH);
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
 * Name: usbdev_hidvendor_get_composite_devdesc
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

void usbdev_hidvendor_get_composite_devdesc(FAR struct composite_devdesc_s *dev)
{
    memset(dev, 0, sizeof(struct composite_devdesc_s));

    dev->mkconfdesc          = usbclass_mkcfgdesc;
    dev->mkstrdesc           = usbclass_mkstrdesc;
    dev->classobject         = usbclass_classobject;
    dev->uninitialize        = usbclass_uninitialize;
    dev->nconfigs            = USBHID_NCONFIGS;
    dev->configid            = 1;
    dev->cfgdescsize         = sizeof(g_iaddesc)
                                + sizeof(g_hid_ifdesc)
                                + sizeof(g_hiddesc)
                                + 2 * USB_SIZEOF_EPDESC;
    dev->devinfo.ninterfaces = 1;
    dev->devinfo.nstrings    = USBHID_NSTRIDS;
    dev->devinfo.nendpoints  = USBHID_VENDOR_NUM_EPS;
    dev->devinfo.epinfos     = g_hidvendor_epinfos;
    dev->devinfo.name        = USBHID_CHARDEV_PATH;
}
