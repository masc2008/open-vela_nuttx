/****************************************************************************
 * drivers/usbdev/bes_rndis_router.c
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
#include <nuttx/config.h>          /* NuttX configuration */
#include <debug.h>                 /* For nerr, info */
#include <nuttx/compiler.h>        /* For CONFIG_CPP_HAVE_WARNING */

#include <sys/socket.h>
#include <sys/ioctl.h>

#include <inttypes.h>
#include <sched.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
#include <errno.h>

#include <nuttx/net/net.h>
#include <nuttx/net/netdev.h>

#define RNDIS_ROUTER_DHCPD_ROUTERIP     0xc0a82b01
#define RNDIS_ROUTER_DHCPD_NETMASK      0xffffff00
#define RNDIS_ROUTER_DHCPD_STARTIP      0xc0a82b02
#define RNDIS_ROUTER_DHCPD_DNSIP        0x72727272
#define RNDIS_ROUTER_IPV6_ADDR0         0xfc00
#define RNDIS_ROUTER_IPV6_ADDR1         0x0000
#define RNDIS_ROUTER_IPV6_ADDR2         0x0000
#define RNDIS_ROUTER_IPV6_ADDR3         0x0000
#define RNDIS_ROUTER_IPV6_ADDR4         0x0000
#define RNDIS_ROUTER_IPV6_ADDR5         0x0000
#define RNDIS_ROUTER_IPV6_ADDR6         0x0000
#define RNDIS_ROUTER_IPV6_ADDR7         0x0001
#define RNDIS_ROUTER_IPV6_PREFLEN       64

void rndis_router_setup(struct net_driver_s *dev)
{
  FAR const char *devname = dev->d_ifname;
  struct in_addr addr;
  struct in6_addr in6_addr;

  /* Set up our host address */
  addr.s_addr = HTONL(RNDIS_ROUTER_DHCPD_ROUTERIP);
  netlib_set_ipv4addr(devname, &addr);

  /* Set up the default router address */
  addr.s_addr = HTONL(RNDIS_ROUTER_DHCPD_ROUTERIP);
  netlib_set_dripv4addr(devname, &addr);

  /* Setup the subnet mask */
  addr.s_addr = HTONL(RNDIS_ROUTER_DHCPD_NETMASK);
  netlib_set_ipv4netmask(devname, &addr);

  netlib_obtain_ipv6addr(devname);

  in6_addr.in6_u.u6_addr16[0] = HTONS(RNDIS_ROUTER_IPV6_ADDR0);
  in6_addr.in6_u.u6_addr16[1] = HTONS(RNDIS_ROUTER_IPV6_ADDR1);
  in6_addr.in6_u.u6_addr16[2] = HTONS(RNDIS_ROUTER_IPV6_ADDR2);
  in6_addr.in6_u.u6_addr16[3] = HTONS(RNDIS_ROUTER_IPV6_ADDR3);
  in6_addr.in6_u.u6_addr16[4] = HTONS(RNDIS_ROUTER_IPV6_ADDR4);
  in6_addr.in6_u.u6_addr16[5] = HTONS(RNDIS_ROUTER_IPV6_ADDR5);
  in6_addr.in6_u.u6_addr16[6] = HTONS(RNDIS_ROUTER_IPV6_ADDR6);
  in6_addr.in6_u.u6_addr16[7] = HTONS(RNDIS_ROUTER_IPV6_ADDR7);
  netlib_add_ipv6addr(devname, &in6_addr, RNDIS_ROUTER_IPV6_PREFLEN);
  netlib_set_dripv6addr(devname, &in6_addr);

  /* Set start ip */
  addr.s_addr = RNDIS_ROUTER_DHCPD_STARTIP;
  dhcpd_set_startip(addr.s_addr);

  addr.s_addr = RNDIS_ROUTER_DHCPD_ROUTERIP;
  dhcpd_set_routerip(addr.s_addr);

  addr.s_addr = RNDIS_ROUTER_DHCPD_NETMASK;
  dhcpd_set_netmask(addr.s_addr);

  addr.s_addr = RNDIS_ROUTER_DHCPD_DNSIP;
  dhcpd_set_dnsip(addr.s_addr);

  dhcpd_start(devname);

  ninfo("%s dhcpd started\n", __func__);
}

void rndis_router_teardown(struct net_driver_s *dev)
{
    dhcpd_stop();
    ninfo("%s dhcpd stoped\n", __func__);
}