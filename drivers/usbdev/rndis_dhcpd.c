/****************************************************************************
 * drivers/usbdev/rndis_dhcpd.c
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

#include <nuttx/net/netdev.h>
#include <nuttx/net/udp.h>
#include <net/if.h>
#include <netinet/in.h>
// #include <netinet/udp.h>
#include <arpa/inet.h>

// #include "netutils/netlib.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

#define DHCP_SERVER_PORT         67
#define DHCP_CLIENT_PORT         68

/* Option codes understood in this file
 *                              Code    Data   Description
 *                                      Length
 */
#define DHCP_OPTION_PAD           0  /*  1     Pad                          */
#define DHCP_OPTION_SUBNET_MASK   1  /*  1     Subnet Mask                  */
#define DHCP_OPTION_ROUTER        3  /*  4     Router                       */
#define DHCP_OPTION_DNS_SERVER    6  /*  4N    DNS                          */
#define DHCP_OPTION_REQ_IPADDR   50  /*  4     Requested IP Address         */
#define DHCP_OPTION_LEASE_TIME   51  /*  4     IP address lease time        */
#define DHCP_OPTION_OVERLOAD     52  /*  1     Option overload              */
#define DHCP_OPTION_MSG_TYPE     53  /*  1     DHCP message type            */
#define DHCP_OPTION_SERVER_ID    54  /*  4     Server identifier            */
#define DHCP_OPTION_END         255  /*  0     End                          */

/* Values for the dhcp msg 'op' field */

#define DHCP_REQUEST              1
#define DHCP_REPLY                2

/* DHCP message types understood in this file */

#define DHCPDISCOVER              1  /* Received from client only */
#define DHCPOFFER                 2  /* Sent from server only */
#define DHCPREQUEST               3  /* Received from client only */
#define DHCPDECLINE               4  /* Received from client only */
#define DHCPACK                   5  /* Sent from server only */
#define DHCPNAK                   6  /* Sent from server only */
#define DHCPRELEASE               7  /* Received from client only */
#define DHCPINFORM                8  /* Not used */

/* The form of an option is:
 *   code   - 1 byte
 *   length - 1 byte
 *   data   - variable number of bytes
 */

#define DHCPD_OPTION_CODE         0
#define DHCPD_OPTION_LENGTH       1
#define DHCPD_OPTION_DATA         2

/* Size of options in DHCP message */

#define DHCPD_OPTIONS_SIZE        312

/* Values for htype and hlen field */

#define DHCP_HTYPE_ETHERNET       1
#define DHCP_HLEN_ETHERNET        6

/* Values for flags field */

#define BOOTP_BROADCAST           0x8000

/* Legal values for this option are:
 *
 *   1     the 'file' field is used to hold options
 *   2     the 'sname' field is used to hold options
 *   3     both fields are used to hold options
 */

#define DHCPD_OPTION_FIELD        0
#define DHCPD_FILE_FIELD          1
#define DHCPD_SNAME_FIELD         2

#ifndef CONFIG_NETUTILS_DHCPD_LEASETIME
#  define CONFIG_NETUTILS_DHCPD_LEASETIME (60*60*24*10) /* 10 days */
#  undef CONFIG_NETUTILS_DHCPD_MINLEASETIME
#  undef CONFIG_NETUTILS_DHCPD_MAXLEASETIME
#endif

#ifndef CONFIG_NETUTILS_DHCPD_MINLEASETIME
#  define CONFIG_NETUTILS_DHCPD_MINLEASETIME (60*60*24*1) /* 1 days */
#endif

#ifndef CONFIG_NETUTILS_DHCPD_MAXLEASETIME
#  define CONFIG_NETUTILS_DHCPD_MAXLEASETIME (60*60*24*30) /* 30 days */
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct dhcpmsg_s
{
  uint8_t  op;
  uint8_t  htype;
  uint8_t  hlen;
  uint8_t  hops;
  uint8_t  xid[4];
  uint16_t secs;
  uint16_t flags;
  uint8_t  ciaddr[4];
  uint8_t  yiaddr[4];
  uint8_t  siaddr[4];
  uint8_t  giaddr[4];
  uint8_t  chaddr[16];
  uint8_t  sname[64];
  uint8_t  file[128];
  uint8_t  options[312];
};

struct dhcpd_state_s
{
  /* Server configuration */
  in_addr_t        ds_serverip;     /* The server IP address */
  /* Message buffers */
  struct dhcpmsg_s *ds_inpacket;     /* Holds the incoming DHCP client message */
  struct dhcpmsg_s *ds_outpacket;    /* Holds the outgoing DHCP server message */
  /* Parsed options from the incoming DHCP client message */
  uint8_t          ds_optmsgtype;   /* Incoming DHCP message type */
  in_addr_t        ds_optreqip;     /* Requested IP address (host order) */
  in_addr_t        ds_optserverip;  /* Serverip IP address (host order) */
  time_t           ds_optleasetime; /* Requested lease time (host order) */
  /* End option pointer for outgoing DHCP server message */
  uint8_t          *ds_optend;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const uint8_t g_magiccookie[4] = {99, 130, 83, 99};
FAR struct dhcpd_state_s g_ds_data = {0};
in_addr_t g_dnsip = 0;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: dhcpd_parseoptions
 ****************************************************************************/

static inline bool dhcpd_parseoptions(void)
{
  uint32_t tmp = 0;
  uint8_t *ptr = NULL;
  int optlen = 0;
  int remaining = 0;

  /* Verify that the option field starts with a valid magic number */

  ptr = g_ds_data.ds_inpacket->options;
  if (memcmp(ptr, g_magiccookie, 4) != 0)
  {
    /* Bad magic number... skip g_ds_data.ds_outpacket */
    nerr("ERROR: Bad magic: %d,%d,%d,%d\n", ptr[0], ptr[1], ptr[2], ptr[3]);
    return false;
  }

  /* Set up to parse the options */
  ptr       += 4;
  remaining  = DHCPD_OPTIONS_SIZE - 4;

  /* Set all options to the default value */
  g_ds_data.ds_optmsgtype   = 0;    /* Incoming DHCP message type */
  g_ds_data.ds_optreqip     = 0;    /* Requested IP address (host order) */
  g_ds_data.ds_optserverip  = 0;    /* Serverip IP address (host order) */
  g_ds_data.ds_optleasetime = 0;    /* Requested lease time (host order) */
  g_ds_data.ds_optend       = NULL;

  do
  {
    /* The form of an option is:
     *   code   - 1 byte
     *   length - 1 byte
     *   data   - variable number of bytes
     */

    switch (ptr[DHCPD_OPTION_CODE])
    {
    case DHCP_OPTION_PAD: /* Skip over any padding bytes */
      optlen = 1;
      break;

    case DHCP_OPTION_END:
      return true;

    case DHCP_OPTION_MSG_TYPE: /* DHCP message type */
      optlen = ptr[DHCPD_OPTION_LENGTH] + 2;
      if (optlen >= 3 && optlen < remaining)
        {
          g_ds_data.ds_optmsgtype = ptr[DHCPD_OPTION_DATA];
        }
      break;

    case DHCP_OPTION_REQ_IPADDR: /* Requested IP Address */
      optlen = ptr[DHCPD_OPTION_LENGTH] + 2;
      if (optlen >= 6 && optlen < remaining)
        {
          memcpy(&tmp, &ptr[DHCPD_OPTION_DATA], 4);
          g_ds_data.ds_optreqip = (in_addr_t)ntohl(tmp);
        }
      break;

    case DHCP_OPTION_SERVER_ID: /* Server identifier */
      optlen = ptr[DHCPD_OPTION_LENGTH] + 2;
      if (optlen >= 6 && optlen < remaining)
        {
          memcpy(&tmp, &ptr[DHCPD_OPTION_DATA], 4);
          g_ds_data.ds_optserverip = (in_addr_t)ntohl(tmp);
        }
      break;

    case DHCP_OPTION_LEASE_TIME: /* IP address lease time */
        optlen = ptr[DHCPD_OPTION_LENGTH] + 2;
      if (optlen >= 6 && optlen < remaining)
        {
          memcpy(&tmp, &ptr[DHCPD_OPTION_DATA], 4);
          g_ds_data.ds_optleasetime = (time_t)ntohl(tmp);
        }
      break;

    default: /* Skip over unsupported options */
      optlen = ptr[DHCPD_OPTION_LENGTH] + 2;
      break;
    }

    /* Advance to the next option */
    ptr       += optlen;
    remaining -= optlen;
  } while (remaining > 0);

  return false;
}

/****************************************************************************
 * Name: dhcpd_verifyreqleasetime
 ****************************************************************************/

static inline bool dhcpd_verifyreqleasetime(uint32_t *leasetime)
{
  uint32_t tmp = g_ds_data.ds_optleasetime;

  /* Did the client request a specific lease time? */

  if (tmp != 0)
    {
      /* Yes..  Verify that the requested lease time is within a
       * valid range
       */

      if (tmp > CONFIG_NETUTILS_DHCPD_MAXLEASETIME)
        {
          tmp = CONFIG_NETUTILS_DHCPD_MAXLEASETIME;
        }
      else if (tmp < CONFIG_NETUTILS_DHCPD_MINLEASETIME)
        {
          tmp = CONFIG_NETUTILS_DHCPD_MINLEASETIME;
        }

      /* Return the clipped lease time */

      *leasetime = tmp;
      return true;
    }

  return false;
}

/****************************************************************************
 * Name: dhcpd_addoption
 ****************************************************************************/

static int dhcpd_addoption(uint8_t *option)
{
  int offset;
  int len = 4;

  if (g_ds_data.ds_optend)
    {
      offset = g_ds_data.ds_optend - g_ds_data.ds_outpacket->options;
      len    = option[DHCPD_OPTION_LENGTH] + 2;

      /* Check if the option will fit into the options array */

      if (offset + len + 1 < DHCPD_OPTIONS_SIZE)
        {
          /* Copy the option into the option array */

          memcpy(g_ds_data.ds_optend, option, len);
          g_ds_data.ds_optend += len;
          *g_ds_data.ds_optend = DHCP_OPTION_END;
        }
    }

  return len;
}

/****************************************************************************
 * Name: dhcpd_addoption8
 ****************************************************************************/

static int dhcpd_addoption8(uint8_t code, uint8_t value)
{
  uint8_t option[3];

  /* Construct the option sequence */

  option[DHCPD_OPTION_CODE]   = code;
  option[DHCPD_OPTION_LENGTH] = 1;
  option[DHCPD_OPTION_DATA]   = value;

  /* Add the option sequence to the response */

  return dhcpd_addoption(option);
}

/****************************************************************************
 * Name: dhcpd_addoption32
 ****************************************************************************/

static int dhcpd_addoption32(uint8_t code, uint32_t value)
{
  uint8_t option[6];

  /* Construct the option sequence */

  option[DHCPD_OPTION_CODE]   = code;
  option[DHCPD_OPTION_LENGTH] = 4;
  memcpy(&option[DHCPD_OPTION_DATA], &value, 4);

  /* Add the option sequence to the response */

  return dhcpd_addoption(option);
}

/****************************************************************************
 * Name: dhcp_addoption32p
 ****************************************************************************/

static int dhcp_addoption32p(uint8_t code, FAR uint8_t *value)
{
  uint8_t option[6];

  /* Construct the option sequence */

  option[DHCPD_OPTION_CODE]   = code;
  option[DHCPD_OPTION_LENGTH] = 4;
  memcpy(&option[DHCPD_OPTION_DATA], value, 4);

  /* Add the option sequence to the response */

  return dhcpd_addoption(option);
}

/****************************************************************************
 * Name: dhcpd_initpacket
 ****************************************************************************/

static void dhcpd_initpacket(uint8_t mtype)
{
  memset(g_ds_data.ds_outpacket, 0, sizeof(struct dhcpmsg_s));

  g_ds_data.ds_outpacket->op         = DHCP_REPLY;
  g_ds_data.ds_outpacket->htype      = g_ds_data.ds_inpacket->htype;
  g_ds_data.ds_outpacket->hlen       = g_ds_data.ds_inpacket->hlen;

  memcpy(&g_ds_data.ds_outpacket->xid, &g_ds_data.ds_inpacket->xid, 4);

  g_ds_data.ds_outpacket->flags  = g_ds_data.ds_inpacket->flags;

  memcpy(g_ds_data.ds_outpacket->chaddr, g_ds_data.ds_inpacket->chaddr, 16);

  /* Add the generic options */
  memcpy(g_ds_data.ds_outpacket->options, g_magiccookie, 4);
  g_ds_data.ds_optend = &g_ds_data.ds_outpacket->options[4];
  *g_ds_data.ds_optend = DHCP_OPTION_END;
  dhcpd_addoption8(DHCP_OPTION_MSG_TYPE, mtype);
  dhcpd_addoption32(DHCP_OPTION_SERVER_ID, g_ds_data.ds_serverip);
}

/****************************************************************************
 * Name: dhcpd_request
 ****************************************************************************/

static inline uint16_t dhcpd_request(FAR struct net_driver_s *netdev)
{
  in_addr_t ipaddr = 0;
  uint32_t  dnsaddr = 0;
  uint32_t  leasetime = CONFIG_NETUTILS_DHCPD_LEASETIME;
  uint8_t   response = 0;
  uint16_t  len = 0;

  if (NULL == netdev)
  {
    return 0;
  }

  ipaddr = netdev->d_ipaddr;
  uinfo("ipaddr: %08" PRIx32 " Server IP: %08" PRIx32 " Requested IP: %08" PRIx32 "\n",
        (uint32_t)ipaddr, (uint32_t)g_ds_data.ds_optserverip, (uint32_t)g_ds_data.ds_optreqip);

  uinfo("d_draddr: %08lx, ipaddr: %08lx\n", ntohl(netdev->d_draddr), ntohl(ipaddr));

  if (0 == g_ds_data.ds_optserverip && 0 == g_ds_data.ds_optreqip)
  {
    uint32_t tmp = htonl(ipaddr);
    if (memcmp(&tmp, g_ds_data.ds_inpacket->ciaddr, 4) == 0)
    {
      uinfo("ciaddr matched\n");
      response = DHCPACK;
    }
    else
    {
      uinfo("ciaddr not matched\n");
      response = DHCPNAK;
    }
  }
  else if ((0 == g_ds_data.ds_optserverip || g_ds_data.ds_optserverip == ntohl(netdev->d_draddr)) &&
           (0 == g_ds_data.ds_optreqip || g_ds_data.ds_optreqip == ntohl(ipaddr)))
  {
    uinfo("matched\n");
    response = DHCPACK;
  }
  else
  {
    uinfo("wrong request\n");
    response = DHCPNAK;
  }

  if (response == DHCPNAK)
  {
    uinfo("NAK IP %08lx\n", (long)ipaddr);
    dhcpd_initpacket(DHCPNAK);
    memcpy(g_ds_data.ds_outpacket->ciaddr, g_ds_data.ds_inpacket->ciaddr, 4);
  }
  else if (response == DHCPACK)
  {
    uinfo("ACK IP %08lx\n", (long)ipaddr);

    dnsaddr = g_dnsip;

    /* Initialize the ACK response */
    dhcpd_initpacket(DHCPACK);

    memcpy(g_ds_data.ds_outpacket->ciaddr, g_ds_data.ds_inpacket->ciaddr, 4);

    /* Add the IP address assigned to the client */
    memcpy(g_ds_data.ds_outpacket->yiaddr, &ipaddr, 4);

    /* Did the client request a specific lease time? */
    dhcpd_verifyreqleasetime(&leasetime);

    dhcpd_addoption32(DHCP_OPTION_LEASE_TIME,   htonl(leasetime));
    dhcpd_addoption32(DHCP_OPTION_SUBNET_MASK,  netdev->d_netmask);
    dhcpd_addoption32(DHCP_OPTION_ROUTER,       netdev->d_draddr);
    dhcp_addoption32p(DHCP_OPTION_DNS_SERVER,   (FAR uint8_t *)&dnsaddr);
  }
  else
  {
    uinfo("Remaining silent IP %08lx\n", (long)ipaddr);
  }

  len = (g_ds_data.ds_optend - (FAR uint8_t *)g_ds_data.ds_outpacket) + 1;
  return len;
}

static inline uint16_t dhcpd_discover(FAR struct net_driver_s *netdev)
{
  in_addr_t ipaddr;
  uint32_t  dnsaddr;
  uint32_t  leasetime = CONFIG_NETUTILS_DHCPD_LEASETIME;
  uint16_t  len = 0;

  ipaddr = netdev->d_ipaddr;
  uinfo("ipaddr = %08lx\n", (long)ipaddr);

  /* dns address is in host order */
  dnsaddr = g_dnsip;

  /* Check if the client has requested a specific lease time */
  dhcpd_verifyreqleasetime(&leasetime);

  /* Initialize the outgoing packet */
  dhcpd_initpacket(DHCPOFFER);

  memcpy(g_ds_data.ds_outpacket->yiaddr, &ipaddr, 4);

  dhcpd_addoption32(DHCP_OPTION_LEASE_TIME,   htonl(leasetime));
  dhcpd_addoption32(DHCP_OPTION_SUBNET_MASK,  netdev->d_netmask);
  dhcpd_addoption32(DHCP_OPTION_ROUTER,       netdev->d_draddr);
  dhcp_addoption32p(DHCP_OPTION_DNS_SERVER,   (FAR uint8_t *)&dnsaddr);

  len = (g_ds_data.ds_optend - (FAR uint8_t *)g_ds_data.ds_outpacket) + 1;
  return len;
}

static int rndis_obtain_dns(FAR void *arg, FAR struct sockaddr *addr, socklen_t addrlen)
{
  FAR struct sockaddr_in *in_addr = (FAR struct sockaddr_in *)addr;

  if (AF_INET == addr->sa_family)
  {
    g_dnsip = in_addr->sin_addr.s_addr;
  }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: dhcpd_run
 ****************************************************************************/

uint16_t rndis_dhcpd_dhcp(FAR struct net_driver_s *netdev, uint8_t *input_buf, uint16_t input_len, uint8_t *output_buf, uint16_t output_len)
{
  uint16_t len = 0;

  uinfo("enter\n");

  g_ds_data.ds_serverip = netdev->d_draddr;

  g_ds_data.ds_inpacket  = (struct dhcpmsg_s*)input_buf;
  g_ds_data.ds_outpacket = (struct dhcpmsg_s*)output_buf;

  if (!dhcpd_parseoptions())
  {
    nerr("ERROR: No msg type\n");
    return 0;
  }

  if (0 == g_dnsip)
  {
    int ret = 0;
    ret = dns_foreach_nameserver(rndis_obtain_dns, &g_dnsip);
    uinfo("rndis_obtain_dns result=%d, g_dnsip=%08x", ret, g_dnsip);
  }

  switch (g_ds_data.ds_optmsgtype)
  {
  case DHCPDISCOVER:
    uinfo("DHCPDISCOVER\n");
    len = dhcpd_discover(netdev);
    break;

  case DHCPREQUEST:
    uinfo("DHCPREQUEST\n");
    len = dhcpd_request(netdev);
    break;

  default:
    uerr("ERROR: Unsupported message type: %d\n", g_ds_data.ds_optmsgtype);
    break;
  }

  return len;
}

#define UDP_HDRLEN (8)
uint16_t rndis_dhcpd_udp(FAR struct net_driver_s *netdev, uint8_t *input_buf, uint16_t input_len, uint8_t *output_buf, uint16_t output_len)
{
  FAR struct udp_hdr_s  *input_udp_hdr   = NULL;
  FAR struct udp_hdr_s  *output_udp_hdr  = NULL;
  uint16_t len = 0;
  uint16_t chksum = 0;

  uinfo("enter\n");

  if (NULL == netdev || NULL == input_buf || UDP_HDRLEN > input_len || NULL == output_buf || 0 == output_len )
  {
    uinfo("param error\n");
    return 0;
  }

  input_udp_hdr = (FAR struct udp_hdr_s *)input_buf;
  if (htons(DHCP_SERVER_PORT) != input_udp_hdr->destport)
  {
    uinfo("rndis_debug %d, destport=%d\n", __LINE__, input_udp_hdr->destport);
    return 0;
  }

  len = rndis_dhcpd_dhcp(netdev, &input_buf[UDP_HDRLEN], input_len - UDP_HDRLEN, output_buf + UDP_HDRLEN, output_len - UDP_HDRLEN);
  if (0 == len)
  {
    return 0;
  }

  len += UDP_HDRLEN;

  output_udp_hdr = (FAR struct udp_hdr_s *)output_buf;
  output_udp_hdr->srcport   = htons(DHCP_SERVER_PORT);
  output_udp_hdr->destport  = htons(DHCP_CLIENT_PORT);
  output_udp_hdr->udplen    = htons(len);
  output_udp_hdr->udpchksum = 0;

  return len;
}

uint16_t rndis_dhcpd_ipv4(FAR struct net_driver_s *netdev, uint8_t *input_buf, uint16_t input_len, uint8_t *output_buf, uint16_t output_len)
{
  FAR struct ipv4_hdr_s *input_ipv4_hdr  = NULL;
  FAR struct ipv4_hdr_s *output_ipv4_hdr = NULL;
  uint16_t len = 0;

  uinfo("enter\n");

  if (NULL == netdev || NULL == input_buf || IPv4_HDRLEN > input_len || NULL == output_buf || 0 == output_len )
  {
    uinfo("param error\n");
    return 0;
  }

  input_ipv4_hdr = (FAR struct ipv4_hdr_s *)input_buf;
  if (IP_PROTO_UDP != input_ipv4_hdr->proto)
  {
    uinfo("rndis_debug %d, ipv4 proto type=0x%02x\n", __LINE__, input_ipv4_hdr->proto);
    return 0;
  }

  len = rndis_dhcpd_udp(netdev, &input_buf[IPv4_HDRLEN], input_len - IPv4_HDRLEN, output_buf + IPv4_HDRLEN, output_len - IPv4_HDRLEN);
  if (0 == len)
  {
    return 0;
  }

  len += IPv4_HDRLEN;

  output_ipv4_hdr               = (FAR struct output_buf *)output_buf;
  output_ipv4_hdr->vhl          = input_ipv4_hdr->vhl;
  output_ipv4_hdr->tos          = input_ipv4_hdr->tos;
  output_ipv4_hdr->len[0]       = (len >> 8);
  output_ipv4_hdr->len[1]       = (len & 0xff);
  output_ipv4_hdr->ipid[0]      = 0;
  output_ipv4_hdr->ipid[1]      = 0;
  output_ipv4_hdr->ipoffset[0]  = 0;
  output_ipv4_hdr->ipoffset[1]  = 0;
  output_ipv4_hdr->ttl          = 64;
  output_ipv4_hdr->proto        = IP_PROTO_UDP;
  net_ipv4addr_hdrcopy(output_ipv4_hdr->srcipaddr, &netdev->d_draddr);
  net_ipv4addr_hdrcopy(output_ipv4_hdr->destipaddr, &netdev->d_ipaddr);

  output_ipv4_hdr->ipchksum    = 0;
  output_ipv4_hdr->ipchksum    = ~ipv4_chksum(output_ipv4_hdr);

  return len;
}
