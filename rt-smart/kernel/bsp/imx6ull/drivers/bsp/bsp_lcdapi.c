/*
 * IMX6ULL
 *   imx6ull rgb lcd driver file
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-01-11     Lyons        first version
 */

#include <rtdef.h>

#include "font.h"
#include "skt.h"

struct skt_lcd_info _g_lcd_info = 
{
    .fb_virt = RT_NULL, //must be 0 after system init!
};

rt_inline void _lcd_draw_point( rt_uint16_t x, rt_uint16_t y, rt_uint32_t c )
{
    rt_uint32_t *framebuffer = (rt_uint32_t*)_g_lcd_info.fb_virt;
    framebuffer[y*_g_lcd_info.width + x] = c;
}

void lcd_show_char( rt_uint16_t x, rt_uint16_t y, rt_uint8_t sz, rt_uint8_t c )
{
    rt_uint16_t ybak, fontsz, t;
    rt_uint8_t dummy;

    fontsz = ((sz/8) + ((sz%8)?1:0)) * (sz/2);

    ybak = y; //save the y as the start pos
    c -= 0x20; //calc offset in table

    for (t=0; t<fontsz; t++)
    {
        switch (sz)
        {
            case 12:
                dummy = _k_ascii_1206_tbl[c][t];
                break;
            case 16:
                dummy = _k_ascii_1608_tbl[c][t];
                break;
            case 24:
                dummy = _k_ascii_2412_tbl[c][t];
                break;
            case 32:
                dummy = _k_ascii_3216_tbl[c][t];
                break;
            default:
                return;
        }

        for (int i=0; i<8; i++)
        {
            if (dummy & 0x80) //msb first
                _lcd_draw_point(x, y, _g_lcd_info.pen_color);

            y++;
            if (y > _g_lcd_info.height)
                return;

            if ((y - ybak) == sz)
            {
                y = ybak;

                x++;
                if (x > _g_lcd_info.width)
                    return;

                break; //height has reached the max
            }

            dummy <<= 1;
        }
    }
}

void lcd_show_string( rt_uint16_t x, rt_uint16_t y, rt_uint8_t sz, char *p )
{
    RT_ASSERT(RT_NULL != _g_lcd_info.fb_virt);

    while ((' ' <= *p) && (*p <= '~'))
    {
        lcd_show_char(x, y, sz, *p);
        x += sz/2;
        p++;
    }
}
