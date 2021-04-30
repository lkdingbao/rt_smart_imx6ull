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

rt_inline void _lcd_draw_point( uint16_t x, uint16_t y, uint32_t c )
{
    rt_uint32_t *framebuffer = (rt_uint32_t*)_g_lcd_info.fb;
    framebuffer[y*_g_lcd_info.width + x] = c;
}

void lcd_show_char( uint16_t x, uint16_t y, uint8_t sz, uint8_t c )
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

void lcd_show_string( uint16_t x, uint16_t y, uint16_t sz, char *p )
{
    RT_ASSERT(RT_NULL != _g_lcd_info.fb_virt);

    while ((' ' <= *p) && (*p <= '~'))
    {
        lcd_show_char(x, y, sz, *p);
        x += sz/2;
        p++;
    }
}

void lcd_draw_point( uint16_t x, uint16_t y, uint32_t color )
{
    _lcd_draw_point(x, y, color);
}

void lcd_draw_line( uint16_t xs, uint16_t ys, uint16_t xe, uint16_t ye, uint32_t color )
{
    uint16_t start, end;
    int16_t x_width, y_height;
    int16_t rem;
    int8_t x_inc, y_inc;

    if (ys == ye) //horizon line
    {
        if (xs > xe)
        {
            start = xe;
            end = xs;
        } else {
            start = xs;
            end = xe;
        }

        for (uint16_t i=start; i<=end; i++) {
            _lcd_draw_point(i, ys, color);
        }
    }
    else if (xs == xe) //vertical line
    {
        if (ys > ye)
        {
            start = ye;
            end = ys;
        } else {
            start = ys;
            end = ye;
        }

        for (uint16_t i=start; i<=end; i++) {
            _lcd_draw_point(xs, i, color);
        }
    }
    else //other line
    {
        x_width = xe - xs;
        y_height = ye - ys;

        if (x_width < 0) {
            x_width = 0 - x_width;
        }
        if (y_height < 0) {
            y_height = 0 - y_height;
        }

        x_inc = (xe > xs) ? 1 : -1;
        y_inc = (ye > ys) ? 1 : -1;

        if (x_width >= y_height)
        {
            rem = x_width/2;
            for (; xs!=xe; xs+=x_inc)
            {
                _lcd_draw_point(xs, ys, color);

                rem += y_height;
                if(rem >= x_width)
                {
                    rem -= x_width;
                    ys += y_inc;
                }
            }
        }
        else
        {
            rem = y_height/2;
            for (; ys!=ye; ys+=y_inc)
            {
                _lcd_draw_point(xs, ys, color);

                rem += x_width;
                if(rem >= y_height)
                {
                    rem -= y_height;
                    xs += x_inc;
                }
            }
        }
    }
}

