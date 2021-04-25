/*
 * IMX6ULL
 *   imx6ull sdhc driver file
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-04-23     Lyons        first version
 */

#ifndef __DRV_SDHC_H__
#define __DRV_SDHC_H__

#include "skt.h"

#ifdef BSP_USING_SDHC

/* Fixed value, not edit! */
#define SDCARD_CONTROL_PIN_NUM      (6)

struct skt_sddev
{
    struct rt_device                parent;

    const char                      *name;
    struct skt_periph               periph;
    rt_uint32_t                     irqno;

    struct skt_gpio                 gpio[SDCARD_CONTROL_PIN_NUM];

    rt_uint32_t                     flag;
};

#endif //#ifdef BSP_USING_SDHC
#endif //#ifndef __DRV_SDHC_H__

