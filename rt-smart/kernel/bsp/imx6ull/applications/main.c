/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2020/10/7      bernard      the first version
 */

#include <rtthread.h>
#ifdef RT_USING_LWP
#include <lwp.h>
#endif

#include "__def.h"
#include "bsp_gpio.h"
#include "bsp_lcdapi.h"
#include "rt_lcd.h"
#include "drv_pin.h"
#include "drv_gt9147.h"
#ifdef RT_USING_LVGL
#include "lvgl.h"
#include "lv_examples.h"
#endif

#define DBG_TAG "main"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

#define LED_PIN                 GET_PIN(0,3)

struct skt_touch_data _g_touch_data;

#ifdef RT_FUNC_SELF_TEST
static void _self_test(void);
#endif

int main(void)
{
#ifdef RT_LCD_CONSOLE_DEBUG
    rt_console_set_device("clcd");
    rt_show_version();

    LOG_D("board: i.mx 6ull");
    LOG_D("build: %s %s", __DATE__, __TIME__);

    /*
     * redirect the console will cause a error.
     * e.g. redirect from uart to clcd, and final to uart.
     *      the second uart will not recv any input.
     *      but this is only occured under rt-thread,
     *      while rt-smart is no-error.
     */
    rt_console_set_device(RT_CONSOLE_DEVICE_NAME);
#endif

    LOG_D("board: i.mx 6ull");
    LOG_D("build: %s %s", __DATE__, __TIME__);

#ifdef RT_FUNC_SELF_TEST
    _self_test();
#endif

    return RT_EOK;
}

#ifdef RT_FUNC_SELF_TEST
void display_entry( void * parameter )
{
    rt_device_t display_dev;
    rt_device_t touch_dev;

    rt_memset(&_g_touch_data, 0x00, sizeof(struct skt_touch_data));

    display_dev= rt_device_find("lcd");
    if (!display_dev)
    {
        LOG_W("not find device[%s]", "lcd");
        return;
    }
    rt_device_open(display_dev, RT_DEVICE_OFLAG_RDWR);
    rt_device_control(display_dev, FBIOGET_VSCREENINFO, &_g_lcd_info);

    touch_dev= rt_device_find("gt9147");
    if (!touch_dev)
    {
        LOG_W("not find device[%s]", "gt9147");
        return;
    }
    rt_device_open(touch_dev, RT_DEVICE_FLAG_RDONLY);

#ifdef RT_USING_LVGL
    lv_init();
    lv_port_disp_init();
    lv_port_indev_init();

    lv_demo_widgets();
#endif

    while (1)
    {
#ifdef RT_USING_GT7147
        if (_g_gt9147_flag & GT_FLAG_NEW_DATA)
        {
            rt_device_read(touch_dev, 0, &_g_touch_data, sizeof(struct skt_touch_data)); //1ms/2ms/5ms
            _g_gt9147_flag &= ~GT_FLAG_NEW_DATA;
        }
#endif
#ifdef RT_USING_LVGL
        lv_task_handler();
#endif

        rt_thread_mdelay(1000/RT_TICK_PER_SECOND); //used to active schedule!
    }

    /* Never Run to Here! */
    rt_device_close(display_dev);
    rt_device_close(touch_dev);
}

static void _self_test( void )
{
    rt_device_t icm20608_dev;
    rt_device_t pcf8574x_dev;
    rt_device_t pin_dev;
    rt_thread_t display_thread;

    rt_uint8_t dummy_data[16];

    UNUSED(dummy_data);
    for (int i=0; i<GET_ARRAY_NUM(dummy_data); i++)
        dummy_data[i] = 0x30 + i;

#ifdef RT_FUNC_SELF_TEST_SPI_DEV
    icm20608_dev= rt_device_find("icm20608");
    rt_device_open(icm20608_dev, RT_DEVICE_OFLAG_RDWR);
    rt_device_write(icm20608_dev, 0, dummy_data, GET_ARRAY_NUM(dummy_data));
#endif

#ifdef RT_FUNC_SELF_TEST_I2C_DEV
    pcf8574x_dev= rt_device_find("pcf8574x");
    rt_device_open(pcf8574x_dev, RT_DEVICE_FLAG_WRONLY);
    rt_device_write(pcf8574x_dev, 0, "hello-world", strlen("hello-world"));
    rt_device_write(pcf8574x_dev, 16, "2020-01-17", strlen("2020-01-17"));
#endif

#ifdef RT_FUNC_SELF_TEST_PIN_DEV
    struct rt_device_pin_mode pin_mode;
    struct rt_device_pin_status pin_status;

    pin_mode.pin = LED_PIN;
    pin_mode.mode = PIN_MODE_OUTPUT;
    pin_status.pin = pin_mode.pin;

    pin_dev= rt_device_find("pin");
    rt_device_open(pin_dev, RT_DEVICE_OFLAG_RDWR);
    rt_device_control(pin_dev, 0, &pin_mode);
#endif

#ifdef RT_FUNC_SELF_TEST_LVGL_DEV
    display_thread = rt_thread_create( "display", 
                                       display_entry, 
                                       RT_NULL,
                                       4096,
                                       10, 100 );
    if (RT_NULL != display_thread ){
        rt_thread_startup(display_thread);
    }
#endif

    while (1)
    {
        pin_status.status = 0;
        rt_device_write(pin_dev, 0, &pin_status, sizeof(pin_status));
        rt_thread_mdelay(500);

        pin_status.status = 1;
        rt_device_write(pin_dev, 0, &pin_status, sizeof(pin_status));
        rt_thread_mdelay(500);
    }

    /* Never Run to Here! */
    rt_device_close(icm20608_dev);
    rt_device_close(pcf8574x_dev);
    rt_device_close(pin_dev);
    if (RT_NULL != display_thread ){
        rt_thread_delete(display_thread);
    }
}
#endif //#ifdef RT_FUNC_SELF_TEST

