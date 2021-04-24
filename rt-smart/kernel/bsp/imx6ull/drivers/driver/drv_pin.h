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
 * p, GPIOx, count fron 1, 1~5 is valid
 * n, PINx, count fron 0, depend on the pin mask table, max 31
 */
#define _PORT_MASK(p)       ( (p-1) & 0x07 )
#define _PIN_MASK(n)        ( (n) & 0x1F )
#define GET_PIN(p, n)       (rt_uint8_t)( (_PORT_MASK(p)<<5) | _PIN_MASK(n) )

#endif //#ifndef __DRV_PIN_H__

