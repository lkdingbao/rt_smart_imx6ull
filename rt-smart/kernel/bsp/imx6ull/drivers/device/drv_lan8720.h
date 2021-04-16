/*
 * LAN8720A
 *   LAN8720A driver file
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-01-30     Lyons        first version
 */

#ifndef __DRV_LAN8720_H__
#define __DRV_LAN8720_H__

#include "skt.h"

#ifdef RT_USING_LAN8720

#include "fsl_enet.h"
#include <netif/ethernetif.h>

#define ENET_RXBD_NUM               (4)
#define ENET_TXBD_NUM               (4)

/* Fixed value, not edit! */
#define ENET_CONTROL_PIN_NUM        (11)

#define ENET_MAC_ADDR_LENGTH        (6)

struct skt_netdev
{
    struct eth_device               parent;

    const char                      *name;
    struct skt_periph               periph;
    rt_uint32_t                     irqno;

    struct skt_gpio                 gpio[ENET_CONTROL_PIN_NUM];
    uint32_t                        rst_pin;

    uint8_t                         phy_addr;
    uint8_t                         mac_addr[ENET_MAC_ADDR_LENGTH];
    enet_handle_t                   handle;

    rt_bool_t                       tx_wait_flag;
    struct rt_semaphore             tx_wait;

    rt_bool_t                       link_up_flag;
    rt_uint8_t                      link_up_status; //high-4 is speed, low-4 is duplex

    rt_uint32_t                     flag;
};

#endif //#ifdef RT_USING_LAN8720
#endif //#ifndef __DRV_LAN8720_H__

