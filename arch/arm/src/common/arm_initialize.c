/****************************************************************************
 * arch/arm/src/common/arm_initialize.c
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

#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <arch/board/board.h>

#include "arm_internal.h"

extern void hal_uart_printf(const char *fmt, ...);

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* g_interrupt_context store irq status */

#undef g_interrupt_context
DEFINE_PER_CPU_BSS(volatile bool, g_interrupt_context);

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_initialize
 *
 * Description:
 *   up_initialize will be called once during OS initialization after the
 *   basic OS services have been initialized.  The architecture specific
 *   details of initializing the OS will be handled here.  Such things as
 *   setting up interrupt service routines, starting the clock, and
 *   registering device drivers are some of the things that are different
 *   for each processor and hardware platform.
 *
 *   up_initialize is called after the OS initialized but before the user
 *   initialization logic has been started and before the libraries have
 *   been initialized.  OS services and driver services are available.
 *
 ****************************************************************************/

void up_initialize(void)
{
  /* Add any extra memory fragments to the memory manager */

  hal_uart_printf("xxx up_initialize: before arm_addregion\n");
  arm_addregion();
  hal_uart_printf("xxx up_initialize: after arm_addregion\n");

#ifdef CONFIG_PM
  /* Initialize the power management subsystem.  This MCU-specific function
   * must be called *very* early in the initialization sequence *before* any
   * other device drivers are initialized (since they may attempt to register
   * with the power management subsystem).
   */

  hal_uart_printf("xxx up_initialize: before arm_pminitialize\n");
  arm_pminitialize();
  hal_uart_printf("xxx up_initialize: after arm_pminitialize\n");
#endif

#ifdef CONFIG_ARCH_DMA
  /* Initialize the DMA subsystem if the weak function arm_dma_initialize has
   * been brought into the build
   */

#ifdef CONFIG_HAVE_WEAKFUNCTIONS
  if (arm_dma_initialize)
#endif
    {
      hal_uart_printf("xxx up_initialize: before arm_dma_initialize\n");
      arm_dma_initialize();
      hal_uart_printf("xxx up_initialize: after arm_dma_initialize\n");
    }
#endif

  /* Initialize the serial device driver */

#ifdef USE_SERIALDRIVER
  hal_uart_printf("xxx up_initialize: before arm_serialinit\n");
  arm_serialinit();
  hal_uart_printf("xxx up_initialize: after arm_serialinit\n");
#endif

  /* Initialize the network */

  hal_uart_printf("xxx up_initialize: before arm_netinitialize\n");
  arm_netinitialize();
  hal_uart_printf("xxx up_initialize: after arm_netinitialize\n");

#if defined(CONFIG_USBDEV) || defined(CONFIG_USBHOST)
  /* Initialize USB -- device and/or host */

  hal_uart_printf("xxx up_initialize: before arm_usbinitialize\n");
  arm_usbinitialize();
  hal_uart_printf("xxx up_initialize: after arm_usbinitialize\n");
#endif

#ifdef CONFIG_ARM_COREDUMP_REGION
  hal_uart_printf("xxx up_initialize: before arm_coredump_add_region\n");
  arm_coredump_add_region();
  hal_uart_printf("xxx up_initialize: after arm_coredump_add_region\n");
#endif

  /* Initialize the L2 cache if present and selected */

  hal_uart_printf("xxx up_initialize: before arm_l2ccinitialize\n");
  arm_l2ccinitialize();
  hal_uart_printf("xxx up_initialize: after arm_l2ccinitialize\n");

#ifdef CONFIG_ARCH_HAVE_DEBUG
  hal_uart_printf("xxx up_initialize: before arm_enable_dbgmonitor\n");
  arm_enable_dbgmonitor();
  hal_uart_printf("xxx up_initialize: after arm_enable_dbgmonitor\n");
#endif

  hal_uart_printf("xxx up_initialize: before board_autoled_on\n");
  board_autoled_on(LED_IRQSENABLED);
  hal_uart_printf("xxx up_initialize: after board_autoled_on\n");
}
