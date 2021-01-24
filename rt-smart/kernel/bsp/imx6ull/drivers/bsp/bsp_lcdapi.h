/*
 * IMX6ULL
 *   imx6ull rgb lcd driver file
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-01-11     Lyons        first version
 */

#ifndef __BSP_LCDAPI_H__
#define __BSP_LCDAPI_H__

#include <rtdef.h>
#include "skt.h"

extern struct skt_lcd_info _g_lcd_info;

void lcd_show_char( rt_uint16_t x, rt_uint16_t y, rt_uint8_t sz, char c );
void lcd_show_string( rt_uint16_t x, rt_uint16_t y, rt_uint8_t sz, char *p );

#endif //#ifndef __BSP_LCDAPI_H__

