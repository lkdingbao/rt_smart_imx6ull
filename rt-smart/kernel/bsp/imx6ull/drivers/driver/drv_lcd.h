/*
 * IMX6ULL
 *   imx6ull rgb lcd driver file
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-01-11     Lyons        first version
 */

#ifndef __DRV_LCD_H__
#define __DRV_LCD_H__

#include "rt_lcd.h"
#include "skt.h"

#ifdef RT_USING_RGBLCD

#define RGB_COLOR_BLUE                      0x000000FF
#define RGB_COLOR_GREEN                     0x0000FF00
#define RGB_COLOR_RED                       0x00FF0000
#define RGB_COLOR_CYAN                      0x0000FFFF
#define RGB_COLOR_MAGENTA                   0x00FF00FF
#define RGB_COLOR_YELLOW                    0x00FFFF00
#define RGB_COLOR_LIGHTBLUE                 0x008080FF
#define RGB_COLOR_LIGHTGREEN                0x0080FF80
#define RGB_COLOR_LIGHTRED                  0x00FF8080
#define RGB_COLOR_LIGHTCYAN                 0x0080FFFF
#define RGB_COLOR_LIGHTMAGENTA              0x00FF80FF
#define RGB_COLOR_LIGHTYELLOW               0x00FFFF80
#define RGB_COLOR_DARKBLUE                  0x00000080
#define RGB_COLOR_DARKGREEN                 0x00008000
#define RGB_COLOR_DARKRED                   0x00800000
#define RGB_COLOR_DARKCYAN                  0x00008080
#define RGB_COLOR_DARKMAGENTA               0x00800080
#define RGB_COLOR_DARKYELLOW                0x00808000
#define RGB_COLOR_WHITE                     0x00FFFFFF
#define RGB_COLOR_LIGHTGRAY                 0x00D3D3D3
#define RGB_COLOR_GRAY                      0x00808080
#define RGB_COLOR_DARKGRAY                  0x00404040
#define RGB_COLOR_BLACK                     0x00000000
#define RGB_COLOR_BROWN                     0x00A52A2A
#define RGB_COLOR_ORANGE                    0x00FFA500
#define RGB_COLOR_TRANSPARENT               0x00000000
#define RGB_COLOR_PURPLE                    0x00800080

#endif //#ifdef RT_USING_RGBLCD
#endif //#ifndef __DRV_LCD_H__

