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
#include <lwp.h>

#include "__def.h"
#include "drv_pin.h"
#include "drv_gt9147.h"

#define DBG_TAG "main"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

#define LED_PIN                 GET_PIN(0,3)

#ifdef RT_FUNC_SELF_TEST
static void _self_test(void);
#endif

int main(void)
{
#ifdef RT_LCD_CONSOLE_DEBUG
    rt_console_set_device("clcd");
    rt_show_version();

    LOG_D("rt-smart on imx6ull");
    LOG_D("build %s %s", __DATE__, __TIME__);
#endif

    rt_console_set_device(RT_CONSOLE_DEVICE_NAME);

    LOG_D("rt-smart on imx6ull");
    LOG_D("build %s %s", __DATE__, __TIME__);

#ifdef RT_FUNC_SELF_TEST
    _self_test();
#endif

    return RT_EOK;
}

#ifdef RT_FUNC_SELF_TEST
static void touch_entry( void * parameter )
{
    rt_device_t gt9147_dev;

    gt9147_dev= rt_device_find("gt9147");
    rt_device_open(gt9147_dev, RT_DEVICE_FLAG_RDONLY);

    while (1)
    {
        if (_g_gt9147_flag & GT_FLAG_NEW_DATA) {
            rt_device_read(gt9147_dev, 0, RT_NULL, 0);
        }
        rt_thread_mdelay(100);
    }

    /* Never Run to Here! */
    rt_device_close(gt9147_dev);
}

static void _self_test( void )
{
    rt_device_t icm20608_dev;
    rt_device_t pcf8574x_dev;
    rt_device_t lcd_dev;
    rt_device_t pin_dev;
    rt_thread_t touch_thread;

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

#ifdef RT_FUNC_SELF_TEST_LCD_DEV
    lcd_dev= rt_device_find("clcd");
    rt_device_open(lcd_dev, RT_DEVICE_OFLAG_RDWR);
    rt_device_control(lcd_dev, FBIOGET_VSCREENINFO, &_g_lcd_info);
    rt_device_write(lcd_dev, 0, "hello-world\n", strlen("hello-world\n"));
    rt_device_write(lcd_dev, 16, "2020-01-17\n", strlen("2020-01-17\n"));
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

#ifdef RT_FUNC_SELF_TEST_TOUCH_DEV
    touch_thread = rt_thread_create( "touch", 
                                     touch_entry, 
                                     RT_NULL,
                                     1024,
                                     10, 20 );
    if (RT_NULL != touch_thread ){
        rt_thread_startup(touch_thread);
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
    rt_device_close(lcd_dev);
    rt_device_close(pin_dev);
    if (RT_NULL != touch_thread ){
        rt_thread_delete(touch_thread);
    }
}
#endif //#ifdef RT_FUNC_SELF_TEST

