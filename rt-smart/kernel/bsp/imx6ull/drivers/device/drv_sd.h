/*
 * LAN8720A
 *   LAN8720A driver file
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-01-30     Lyons        first version
 */

#ifndef __DRV_SDCARD_H__
#define __DRV_SDCARD_H__

#include "skt.h"

#ifdef RT_USING_SDCARD

/* Fixed value, not edit! */
#define SDCARD_CONTROL_PIN_NUM      (7)

struct skt_sddev
{
    struct rt_device                parent;

    const char                      *name;
    struct skt_periph               periph;
    rt_uint32_t                     irqno;

    struct skt_gpio                 gpio[SDCARD_CONTROL_PIN_NUM];
    uint32_t                        cd_pin;

    rt_uint32_t                     flag;
};

#endif //#ifdef RT_USING_SDCARD
#endif //#ifndef __DRV_SDCARD_H__

