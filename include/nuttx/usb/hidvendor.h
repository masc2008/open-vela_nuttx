/****************************************************************************
 * include/nuttx/usb/hidvendor.h
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

#ifndef __INCLUDE_NUTTX_USB_VENDOR_H
#define __INCLUDE_NUTTX_USB_VENDOR_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/usb/usbdev.h>

/****************************************************************************
 * Preprocessor definitions
 ****************************************************************************/

/* Indexes for devinfo.epno[] array.
 * Used for composite device configuration.
 */

#define USBHID_VENDOR_NUM_EPS           (2)

#define USBVENDOR_EP_INTIN_IDX          (0)
#define USBVENDOR_EP_INTOUT_IDX         (1)

/* Endpoint configuration ***************************************************/

#define HIDVENDOR_MKEPINTIN(desc)       (USB_DIR_IN | (desc)->epno[USBVENDOR_EP_INTIN_IDX])
#define HIDVENDOR_MKEPINTOUT(desc)      (USB_DIR_OUT | (desc)->epno[USBVENDOR_EP_INTOUT_IDX])
#define HIDVENDOR_EPINTIN_ATTR          (USB_EP_ATTR_XFER_INT)

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#  define EXTERN extern "C"
extern "C"
{
#else
#  define EXTERN extern
#endif

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

void usbdev_hidvendor_get_composite_devdesc(FAR struct composite_devdesc_s *dev);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __INCLUDE_NUTTX_USB_VENDOR_H */