/*
 * IMX6ULL
 *   imx6ull arch timer driver file
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-01-13     Lyons        first version
 */

#ifndef __DRV_SKT_H__
#define __DRV_SKT_H__

#ifdef cplusplus
extern "C" {
#endif

#include <rtdevice.h>

struct skt_periph
{
    rt_uint32_t                 paddr;
    rt_uint32_t                 vaddr;
};

struct skt_gpio
{
    rt_uint32_t                 muxRegister;
    rt_uint32_t                 muxMode;
    rt_uint32_t                 inputRegister;
    rt_uint32_t                 inputDaisy;
    rt_uint32_t                 configRegister;

    rt_uint32_t                 inputOnfield;
    rt_uint32_t                 configValue;
};

struct skt_uart
{
    struct rt_serial_device     parent;

    const char                  *name;
    struct skt_periph           periph;
    rt_uint32_t                 irqno;

    //[0]-tx [1]-rx
    struct skt_gpio             gpio[2];

    rt_uint32_t                 flag;
    struct serial_configure     param;
};

struct skt_spi_cs
{
    rt_uint32_t                 gpio_port;
    rt_uint16_t                 gpio_pin;
    rt_uint8_t                  reserved[2];
    rt_uint32_t                 cs_delay;
    rt_uint32_t                 data_delay;
};

struct skt_spi
{
    struct rt_spi_bus           parent;

    const char                  *name;
    struct skt_periph           periph;

    //[0]-clk [1]-miso [2]-mosi
    struct skt_gpio             gpio[3];

    rt_uint32_t                 flag;
};

struct skt_i2c
{
    struct rt_i2c_bus_device    parent;

    const char                  *name;
    struct skt_periph           periph;

    //[0]-sck [1]-sda
    struct skt_gpio             gpio[2];

    rt_uint32_t                 flag;
};

struct skt_lcd_info
{
    rt_uint16_t                 width;
    rt_uint16_t                 height;

    rt_uint8_t                  pxsz;
    rt_uint8_t                  reserved[3];

    rt_uint32_t                 fb_phys;
    rt_uint32_t                 fb_virt;

    rt_uint16_t                 vsw;
    rt_uint16_t                 vbp;
    rt_uint16_t                 vfp;
    rt_uint16_t                 hsw;
    rt_uint16_t                 hbp;
    rt_uint16_t                 hfp;

    rt_uint32_t                 pen_color;
    rt_uint32_t                 panel_color;
};

struct skt_lcd
{
    struct rt_device            parent;

    const char                  *name;
    struct skt_periph           periph;

    struct skt_lcd_info         info;

    rt_uint32_t                 flag;
};

struct skt_lcd_disp
{
    rt_uint8_t                  *buf;

    rt_uint16_t                 deep;
    rt_uint16_t                 cnt;

    rt_uint32_t                 color;

    struct skt_lcd_disp         *next;
};

#ifdef RT_LCD_CONSOLE_PARSER
struct skt_parser
{
    rt_uint8_t                  buf[8];
    rt_uint8_t                  deep;
    rt_uint8_t                  cnt;
    rt_uint8_t                  flag;
};
#endif

struct skt_lcd_console
{
    struct rt_serial_device     parent;

    const char                  *name;

    rt_uint16_t                 xn;
    rt_uint16_t                 yn;

    rt_uint32_t                 pen_color;
    rt_uint32_t                 panel_color;
    rt_uint16_t                 font_size;

    struct skt_lcd_disp         disp_list;

#ifdef RT_LCD_CONSOLE_PARSER
    struct skt_parser           parser;
#endif

    rt_uint32_t                 flag;
};

#ifdef cplusplus
}
#endif

#endif //#ifndef __DRV_SKT_H__

