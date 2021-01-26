/*
 * IMX6ULL
 *   imx6ull pin driver file
 *
 * Change Logs:
 * Date           Author       Notes
 * 2020-01-13     Lyons        edit and remove irq setting
 */

#ifndef __DRV_PIN_H__
#define __DRV_PIN_H__

#include "skt.h"

/* 
 * p, GPIOx, count fron 0, 0~4 is valid
 * n, PINx, count fron 0, depend on the pin mask table, max 31
 */
#define GET_PIN(p, n)       (rt_uint8_t)((((p)&0x7)<<5) | ((n) & 0x1F))

#endif //#ifndef __DRV_PIN_H__

