/*
 * IMX6ULL
 *   imx6ull rgb lcd driver file
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-01-11     Lyons        first version
 */

#include <rtconfig.h>
#include <rthw.h>
#include <rtdevice.h>

#ifdef BSP_USING_RGBLCD

#include <board.h>
#ifdef RT_USING_LWP
#include <lwp.h>
#include <lwp_user_mm.h>
#endif

#include "__def.h"
#include "realview.h"
#include "bsp_gpio.h"
#include "bsp_lcdapi.h"
#include "drv_lcd.h"
#include "skt.h"

#define DBG_TAG "drv.lcd"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

#define _LCD_WIDTH                  BSP_LCD_WIDTH
#define _LCD_HEIGHT                 BSP_LCD_HEIGHT
#define _LCD_PIXEL_SIZE             4

#if ( defined (BSP_LCD_CONSOLE_DEBUG)  \
   && defined (BSP_LCD_CONSOLE_PARSER) )
#define _PARSER_FLAG_ANALYSE        (1 << 0)
#define _PARSER_FLAG_FINISH         (1 << 1)
#endif

_internal_rw struct skt_lcd _s_lcd =
{
    .name = "lcd",
    .periph = {
        {.paddr = REALVIEW_CLCD_BASE,},
        {.paddr = REALVIEW_SDMA_BASE,},
    },
    .info = {
        .width  = _LCD_WIDTH,
        .height = _LCD_HEIGHT,
        .pxsz   = _LCD_PIXEL_SIZE,

        .hsw  = 3,
        .hbp  = 88,
        .hfp  = 40,
        .vsw  = 3,
        .vbp  = 32,
        .vfp  = 13,

        .fb_virt = KERNEL_VADDR_START + 0x0f000000, //defined at board.c!
        .fb_phys = 0,

        .pen_color   = RGB_COLOR_BLACK,
        .panel_color = RGB_COLOR_WHITE,
    },
    .flag = 0,
};

_internal_rw rt_thread_t _s_lcd_disp_tid = RT_NULL;

_internal_rw struct skt_gpio _s_gpio_info[] = 
{
    {IOMUXC_LCD_DATA00_LCDIF_DATA00, 0, 0x79},
    {IOMUXC_LCD_DATA01_LCDIF_DATA01, 0, 0x79},
    {IOMUXC_LCD_DATA02_LCDIF_DATA02, 0, 0x79},
    {IOMUXC_LCD_DATA03_LCDIF_DATA03, 0, 0x79},
    {IOMUXC_LCD_DATA04_LCDIF_DATA04, 0, 0x79},
    {IOMUXC_LCD_DATA05_LCDIF_DATA05, 0, 0x79},
    {IOMUXC_LCD_DATA06_LCDIF_DATA06, 0, 0x79},
    {IOMUXC_LCD_DATA07_LCDIF_DATA07, 0, 0x79},
    {IOMUXC_LCD_DATA08_LCDIF_DATA08, 0, 0x79},
    {IOMUXC_LCD_DATA09_LCDIF_DATA09, 0, 0x79},
    {IOMUXC_LCD_DATA10_LCDIF_DATA10, 0, 0x79},
    {IOMUXC_LCD_DATA11_LCDIF_DATA11, 0, 0x79},
    {IOMUXC_LCD_DATA12_LCDIF_DATA12, 0, 0x79},
    {IOMUXC_LCD_DATA13_LCDIF_DATA13, 0, 0x79},
    {IOMUXC_LCD_DATA14_LCDIF_DATA14, 0, 0x79},
    {IOMUXC_LCD_DATA15_LCDIF_DATA15, 0, 0x79},
    {IOMUXC_LCD_DATA16_LCDIF_DATA16, 0, 0x79},
    {IOMUXC_LCD_DATA17_LCDIF_DATA17, 0, 0x79},
    {IOMUXC_LCD_DATA18_LCDIF_DATA18, 0, 0x79},
    {IOMUXC_LCD_DATA19_LCDIF_DATA19, 0, 0x79},
    {IOMUXC_LCD_DATA20_LCDIF_DATA20, 0, 0x79},
    {IOMUXC_LCD_DATA21_LCDIF_DATA21, 0, 0x79},
    {IOMUXC_LCD_DATA22_LCDIF_DATA22, 0, 0x79},
    {IOMUXC_LCD_DATA23_LCDIF_DATA23, 0, 0x79},
    
    {IOMUXC_LCD_CLK_LCDIF_CLK,       0, 0x79},
    {IOMUXC_LCD_ENABLE_LCDIF_ENABLE, 0, 0x79},
    {IOMUXC_LCD_HSYNC_LCDIF_HSYNC,   0, 0x79},
    {IOMUXC_LCD_VSYNC_LCDIF_VSYNC,   0, 0x79},
};

#ifdef BSP_LCD_CONSOLE_DEBUG
_internal_rw struct skt_lcd_console _s_lcd_console = 
{
    .name = "clcd",
    .font_size = 16, //only support 1608!
    .flag = (RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX),
};
#endif

#if ( defined (BSP_LCD_CONSOLE_DEBUG)  \
   && defined (BSP_LCD_CONSOLE_PARSER) )
_internal_ro rt_uint32_t _k_console_color_tbl[] = 
{
    RGB_COLOR_BLACK,    //30
    RGB_COLOR_RED,      //31
    RGB_COLOR_GREEN,    //32
    RGB_COLOR_YELLOW,   //33
    RGB_COLOR_BLUE,     //34
    RGB_COLOR_PURPLE,   //35
    RGB_COLOR_CYAN,     //36
    RGB_COLOR_WHITE,    //37
};
#endif

static void _lcd_clear( register rt_uint32_t color )
{
    rt_uint32_t *fbpoint = (rt_uint32_t*)_s_lcd.info.fb;
    rt_uint32_t num = _s_lcd.info.width * _s_lcd.info.height;

    while (num--)
    {
        *fbpoint++ = color;
    }
}

void lcd_clear( register rt_uint32_t color )
{
    _lcd_clear(color);
}

static void _lcd_gpio_init( void )
{
    for (int i=0; i<GET_ARRAY_NUM(_s_gpio_info); i++)
    {
        gpio_set_iomux(&_s_gpio_info[i]);
    }
}

static void _lcd_clock_init( rt_uint8_t loopDiv,
                             rt_uint8_t postDiv,
                             rt_uint8_t preDiv,
                             rt_uint8_t div )
{
    CLOCK_DeinitVideoPll();

    CLOCK_DisableClock(kCLOCK_Lcd);
    CLOCK_DisableClock(kCLOCK_Lcdif1);

    /*
     * Initialize the Video PLL.
     * Video PLL output clock is OSC24M * (loopDivider + (denominator / numerator)) / postDivider.
     */
    clock_video_pll_config_t config = {
        .loopDivider = loopDiv, .postDivider = postDiv, .numerator = 0, .denominator = 0,
    };

    CLOCK_InitVideoPll(&config);

    /*
     * 000 derive clock from PLL2
     * 001 derive clock from PLL3 PFD3
     * 010 derive clock from PLL5
     * 011 derive clock from PLL2 PFD0
     * 100 derive clock from PLL2 PFD1
     * 101 derive clock from PLL3 PFD1
     */
    CLOCK_SetMux(kCLOCK_Lcdif1PreMux, 0x2);

    CLOCK_SetDiv(kCLOCK_Lcdif1PreDiv, (preDiv-1));
    CLOCK_SetDiv(kCLOCK_Lcdif1Div, (div-1));

    /*
     * 000 derive clock from divided pre-muxed lcdif1 clock
     * 001 derive clock from ipp_di0_clk
     * 010 derive clock from ipp_di1_clk
     * 011 derive clock from ldb_di0_clk
     * 100 derive clock from ldb_di1_clk
     */
    CLOCK_SetMux(kCLOCK_Lcdif1Mux, 0x0);
}

static void _lcd_periph_init( struct skt_lcd *device )
{
    LCDIF_Type *periph = RT_NULL;
    elcdif_rgb_mode_config_t config;

    RT_ASSERT(RT_NULL != device);
    periph = (LCDIF_Type*)device->periph[0].vaddr;

    CLOCK_EnableClock(kCLOCK_Lcd);
    CLOCK_EnableClock(kCLOCK_Lcdif1);

    ELCDIF_RgbModeGetDefaultConfig(&config);

    config.panelHeight = device->info.height;
    config.panelWidth = device->info.width;

    config.hsw = device->info.hsw;
    config.hfp = device->info.hfp;
    config.hbp = device->info.hbp;
    config.vsw = device->info.vsw;
    config.vfp = device->info.vfp;
    config.vbp = device->info.vbp;

    config.polarityFlags = kELCDIF_VsyncActiveLow
                         | kELCDIF_HsyncActiveLow
                         | kELCDIF_DataEnableActiveHigh
                         | kELCDIF_DriveDataOnFallingClkEdge;
    config.pixelFormat = kELCDIF_PixelFormatRGB888;
    config.dataBus = kELCDIF_DataBus24Bit;

    config.bufferAddr = device->info.fb_phys; //use physical addr!

    ELCDIF_RgbModeInit(periph, &config);
    ELCDIF_RgbModeStart(periph);
}

static rt_err_t _lcd_ops_open( rt_device_t dev,
                               rt_uint16_t oflag )
{
    RT_ASSERT(RT_NULL != dev);

    if (!(dev->flag & RT_DEVICE_FLAG_RDWR))
    {
        LOG_W("only support rd/wr option!");
        return -RT_ERROR;
    }

    dev->ref_count++;

    return RT_EOK;
}

static rt_err_t _lcd_ops_close(rt_device_t dev)
{
    RT_ASSERT(RT_NULL != dev);

    dev->ref_count = (0 == dev->ref_count) ? 0 : (dev->ref_count - 1);

    return RT_EOK;
}

static rt_err_t _lcd_ops_control( rt_device_t dev,
                                  int cmd,
                                  void *args )
{
    struct skt_lcd *lcd = RT_NULL;
    rt_err_t result = RT_EOK;;

    RT_ASSERT(RT_NULL != dev);
    lcd = (struct skt_lcd*)dev;

    switch (cmd)
    {
        case FBIOGET_VSCREENINFO:
            if (!args)
            {
                result = -RT_EINVAL;
                goto _lcd_ops_control_exit;
            }
            rt_memcpy(args, &lcd->info, sizeof(struct skt_lcd_info));
            break;

        case FBIOBLANK:
            _lcd_clear(lcd->info.panel_color);
            break;

        default:
            result = -RT_ERROR;
            break;
    }

_lcd_ops_control_exit:
    return result;
}

#if ( defined (BSP_LCD_CONSOLE_DEBUG)  \
   && defined (BSP_LCD_CONSOLE_PARSER) )
static void _lcd_console_parser_get_color( const struct skt_parser *parser, rt_uint32_t *pcolor )
{
    rt_uint32_t color_code = 0;
    const rt_uint8_t *pdata = parser->buf;

    if (('\033' != pdata[0]) || ('[' != pdata[1]))
    {
        return;
    }

    for (rt_uint8_t i=2; i<parser->cnt; i++)
    {
        if ('m' == pdata[i])
        {
            break;
        }
        else
        {
            color_code *= 10;
            color_code += (pdata[i] - '0');
        }
    }

    if ( !(parser->flag & _PARSER_FLAG_FINISH) )
    {
        color_code = ((30<=color_code)&&(color_code<=37)) ? (color_code-30) : 0;
        *pcolor = _k_console_color_tbl[color_code];
    }
}
#endif //#ifdef BSP_LCD_CONSOLE_PARSER

#ifdef BSP_LCD_CONSOLE_DEBUG
static rt_err_t _lcd_console_ops_control( struct rt_serial_device *dev,
                                          int cmd,
                                          void *arg )
{
    return RT_EOK;
}

static int _lcd_console_ops_putc( struct rt_serial_device *dev, 
                                  char ch )
{
    struct skt_lcd_console *lcd_console = RT_NULL;
    struct skt_lcd_info *lcd_info = RT_NULL;
    uint32_t src_buf;
    uint32_t dst_buf;
    uint32_t buf_size;
    int flush_flag = 0;
    int start, end;

    RT_ASSERT(RT_NULL != dev);
    RT_ASSERT(RT_NULL != dev->parent.user_data);

    lcd_console = (struct skt_lcd_console*)dev;
    lcd_info = (struct skt_lcd_info*)dev->parent.user_data;

    start = 0;
    end = 1;

    switch (ch)
    {
        case '\0':
            end = 0;
            break;
        case '\b':
            lcd_console->xn = (0 == lcd_console->xn) ? 0 : (lcd_console->xn - 1);
            ch = ' ';
            break;
        case '\r':
            end = 0;
            lcd_console->xn = 0;
            break;
        case '\n':
            end = 0;
            flush_flag = 1;
            break;
        case '\t':
            end = 4;
            ch = ' ';
            break;
        case '\033':
#ifdef BSP_LCD_CONSOLE_PARSER
            lcd_console->parser.flag |= _PARSER_FLAG_ANALYSE;
            lcd_console->parser.cnt = 0;
#else
            ch = ' ';
#endif
            break;
        default:
            ch &= 0x7f;
            if ( !(ch>=32 && ch<127) ) { //check printable
                return 1;
            }
            break;
    }

#ifdef BSP_LCD_CONSOLE_PARSER
    if (lcd_console->parser.flag & _PARSER_FLAG_ANALYSE)
    {
        lcd_console->parser.buf[lcd_console->parser.cnt ++] = ch; //store the row char!
        if ('m' == ch)
        {
            _lcd_console_parser_get_color(&lcd_console->parser, &lcd_info->pen_color);

            lcd_console->parser.flag &= ~_PARSER_FLAG_ANALYSE;
            lcd_console->parser.flag ^=  _PARSER_FLAG_FINISH;

            rt_memset(&lcd_console->parser.buf, 0, 8);
        } 
        else if (lcd_console->parser.deep == lcd_console->parser.cnt)
        {
            lcd_console->parser.flag = 0;
        }
    }
    else
#endif
    {
        for (; start<end; start++)
        {
            if ((lcd_console->xn + 1) >= lcd_console->xnc)
            {
                flush_flag = 1;
            }

            lcd_show_char(lcd_console->xn*lcd_console->font_size/2, lcd_console->yn*lcd_console->font_size, lcd_console->font_size, ch);
            lcd_console->xn ++;

            if ( (0 != flush_flag)
              && ((lcd_console->yn + 1) < lcd_console->ync) )
            {
                flush_flag = 0;
                lcd_console->xn = 0;
                lcd_console->yn ++;
            }
        }

        if (flush_flag)
        {
            if ((lcd_console->yn + 1) >= lcd_console->ync)
            {
                buf_size = _s_lcd.info.width
                         * (_s_lcd.info.height - _s_lcd_console.font_size)
                         * _s_lcd.info.pxsz;

                src_buf = _s_lcd.info.fb;
                dst_buf = _s_lcd.info.fb
                        + _s_lcd.info.width * _s_lcd_console.font_size * _s_lcd.info.pxsz;

                rt_memcpy((uint8_t*)src_buf, (uint8_t*)dst_buf, buf_size);
                rt_memset((uint8_t*)(src_buf+buf_size),
                          0xFF,
                          _s_lcd.info.width * _s_lcd_console.font_size * _s_lcd.info.pxsz);
            } else {
                lcd_console->yn ++;
                lcd_info->pen_color = RGB_COLOR_BLACK;
            }

            lcd_console->xn = 0;
        }

        lcd_info->flush_flag = RT_TRUE;
    }

    return 1;
}

int lcd_console_putc(char ch)
{
    return _lcd_console_ops_putc((struct rt_serial_device*)&_s_lcd_console, ch);
}

_internal_ro struct rt_uart_ops _k_lcd_console_ops =
{
    RT_NULL,                    /* configure */
    _lcd_console_ops_control,   /* control */
    _lcd_console_ops_putc,      /* putc */
    RT_NULL,                    /* getc */
    RT_NULL,                    /* dma_transmit */
};
#endif //#ifdef BSP_LCD_CONSOLE_DEBUG

#ifdef RT_USING_DEVICE_OPS
_internal_ro struct rt_device_ops _k_lcd_ops =
{
    RT_NULL,                    /* init */
    _lcd_ops_open,              /* open */
    _lcd_ops_close,             /* close */
    RT_NULL,                    /* read */
    RT_NULL,                    /* write */
    _lcd_ops_control,           /* control */
};
#endif

void lcd_fill(uint32_t *src, uint32_t *dest, uint32_t num)
{
    rt_memcpy(dest, src, num);
}

int rt_hw_lcd_init(void)
{
    struct rt_device *device = RT_NULL;
    rt_uint32_t fb_size;

    device = &_s_lcd.parent;

    device->type    = RT_Device_Class_Graphic;
#ifdef RT_USING_DEVICE_OPS
    device->ops     = &_k_lcd_ops;
#else
    device->init    = RT_NULL;
    device->open    = _lcd_ops_open;
    device->close   = _lcd_ops_close;
    device->read    = RT_NULL;
    device->write   = RT_NULL;
    device->control = _lcd_ops_control;
#endif
    device->user_data = RT_NULL;

    rt_device_register(device, _s_lcd.name, RT_DEVICE_FLAG_RDWR);

    for (int i=0; i<GET_ARRAY_NUM(_s_lcd.periph); i++) {
        _s_lcd.periph[i].vaddr = platform_get_periph_vaddr(_s_lcd.periph[i].paddr);
    }

    fb_size = _s_lcd.info.width * _s_lcd.info.height * _s_lcd.info.pxsz;
    _s_lcd.info.fb = (rt_uint32_t)rt_malloc(fb_size);
#ifdef RT_USING_USERSPACE
    _s_lcd.info.fb_phys = PV_OFFSET + _s_lcd.info.fb_virt;
#else
    _s_lcd.info.fb_virt = (rt_uint32_t)rt_malloc(fb_size);
    _s_lcd.info.fb_phys = _s_lcd.info.fb_virt;
#endif

    if (RT_NULL == _s_lcd.info.fb_virt)
    {
        return -RT_ERROR;
    }

    rt_memset((rt_uint8_t*)_s_lcd.info.fb, 0xFF, fb_size);
    rt_memset((rt_uint8_t*)_s_lcd.info.fb_virt, 0xFF, fb_size);

    _s_lcd.info.flush_flag = RT_FALSE;

    _lcd_gpio_init();
    _lcd_clock_init(42, 1, 4, 8);
    _lcd_periph_init(&_s_lcd);

#ifdef BSP_LCD_CONSOLE_DEBUG
    _s_lcd_console.parent.ops = &_k_lcd_console_ops;

    rt_hw_serial_register( &_s_lcd_console.parent,
                           _s_lcd_console.name,
                           _s_lcd_console.flag,
                           &_g_lcd_info );

    rt_memcpy(&_g_lcd_info, &_s_lcd.info, sizeof(struct skt_lcd_info));

    _s_lcd_console.xn = 0;
    _s_lcd_console.yn = 0;
    _s_lcd_console.xnc = _g_lcd_info.width / _s_lcd_console.font_size * 2;
    _s_lcd_console.ync = _g_lcd_info.height / _s_lcd_console.font_size;

#ifdef BSP_LCD_CONSOLE_PARSER
    rt_memset(&_s_lcd_console.parser, 0x00, sizeof(struct skt_parser));
    _s_lcd_console.parser.deep = GET_ARRAY_NUM(_s_lcd_console.parser.buf);
#endif //#ifdef BSP_LCD_CONSOLE_PARSER
#endif //#ifdef BSP_LCD_CONSOLE_DEBUG

    return RT_EOK;
}
INIT_BOARD_EXPORT(rt_hw_lcd_init);

static void lcd_flush_thread_entry( void *param )
{
    uint32_t fb_size = _s_lcd.info.width * _s_lcd.info.height
                     * _s_lcd.info.pxsz;

    while (1)
    {
        if (RT_TRUE == _g_lcd_info.flush_flag)
        {
            _g_lcd_info.flush_flag = RT_FALSE;
            lcd_fill((uint32_t*)_s_lcd.info.fb, (uint32_t*)_s_lcd.info.fb_virt, fb_size);
        }
        rt_thread_mdelay(10);
    }
}

int rt_hw_lcd_disp_init(void)
{
    /* start lcd display flush thread in the end. */
    _s_lcd_disp_tid = rt_thread_create( "lcd_disp",
                                        lcd_flush_thread_entry,
                                        RT_NULL,
                                        512,
                                        RT_THREAD_PRIORITY_MAX - 2,
                                        2 );

    if (RT_NULL != _s_lcd_disp_tid)
    {
        rt_thread_startup(_s_lcd_disp_tid);
    } else {
        LOG_E("phy monitor start failed!");
    }

    return RT_EOK;
}
INIT_DEVICE_EXPORT(rt_hw_lcd_disp_init);

#endif //#ifdef BSP_USING_RGBLCD

