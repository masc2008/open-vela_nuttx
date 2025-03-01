/****************************************************************************
 * arch/arm64/src/common/arm64_arch_timer.h
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

#ifndef __ARCH_ARM64_SRC_COMMON_ARM64_ARCH_TIMER_H
#define __ARCH_ARM64_SRC_COMMON_ARM64_ARCH_TIMER_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/timers/oneshot.h>
#define CONFIG_ARM_TIMER_SECURE_IRQ         (GIC_PPI_INT_BASE + 13)
#define CONFIG_ARM_TIMER_NON_SECURE_IRQ     (GIC_PPI_INT_BASE + 14)
#define CONFIG_ARM_TIMER_VIRTUAL_IRQ        (GIC_PPI_INT_BASE + 11)
#define CONFIG_ARM_TIMER_HYP_IRQ            (GIC_PPI_INT_BASE + 10)

#define ARM_ARCH_TIMER_IRQ                  CONFIG_ARM_TIMER_VIRTUAL_IRQ
#define ARM_ARCH_TIMER_PRIO                 IRQ_DEFAULT_PRIORITY
#define ARM_ARCH_TIMER_FLAGS                IRQ_TYPE_LEVEL

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef CONFIG_ONESHOT
/****************************************************************************
 * Name: arm64_oneshot_initialize
 *
 * Description:
 *   This function initialize generic timer hardware module
 *   and return an instance of a "lower half" timer interface.
 *
 * Returned Value:
 *   On success, a non-NULL oneshot_lowerhalf_s is returned to the caller.
 *   In the event of any failure, a NULL value is returned.
 *
 ****************************************************************************/

struct oneshot_lowerhalf_s *arm64_oneshot_initialize(void);
#endif

#endif /* __ARCH_ARM64_SRC_COMMON_ARM64_ARCH_TIMER_H */
