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

typedef union {
    struct {
        uint8_t blue;
        uint8_t green;
        uint8_t red;
        uint8_t alpha;
    } ch;
    uint32_t full;
} lcd_color32_t;

extern struct skt_lcd_info _g_lcd_info;

void lcd_fill(uint32_t *src, uint32_t *dest, uint32_t num);
void lcd_show_char( rt_uint16_t x, rt_uint16_t y, rt_uint8_t sz, char c );
void lcd_show_string( rt_uint16_t x, rt_uint16_t y, rt_uint8_t sz, char *p );

#endif //#ifndef __BSP_LCDAPI_H__

