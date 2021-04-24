/*
 * Main
 *   main file
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-04-24     Lyons        first version
 */

#include <rtthread.h>
#ifdef RT_USING_LWP
#include <lwp.h>
#endif

#include "__def.h"
#include "bsp_gpio.h"
#include "drv_pin.h"

#define DBG_TAG "main"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

#define LED_PIN GET_PIN(1,3)

static void main_loop(void);

int main(void)
{
    LOG_D("board: i.mx 6ull");
    LOG_D("build: %s %s", __DATE__, __TIME__);

    main_loop();

    /* Never Run to Here! */
    return RT_EOK;
}

static void main_loop( void )
{
#ifdef RT_FUNC_SELF_TEST_PIN_DEV
    rt_device_t pin_dev;
    struct rt_device_pin_mode pin_mode;
    struct rt_device_pin_status pin_status;
#endif

#ifdef RT_FUNC_SELF_TEST_PIN_DEV
    pin_dev= rt_device_find("pin");

    pin_mode.pin = LED_PIN;
    pin_mode.mode = PIN_MODE_OUTPUT;

    rt_device_open(pin_dev, RT_DEVICE_OFLAG_RDWR);
    rt_device_control(pin_dev, 0, &pin_mode);

    pin_status.pin = pin_mode.pin;
    pin_status.status = 0;
#endif

    while (1)
    {
#ifdef RT_FUNC_SELF_TEST_PIN_DEV
        pin_status.status = ~pin_status.status;
        rt_device_write(pin_dev, 0, &pin_status, sizeof(pin_status));
#endif

        rt_thread_mdelay(500);
    }

    /* Never Run to Here! */
#ifdef RT_FUNC_SELF_TEST_PIN_DEV
    UNUSED(pin_dev);
#endif
}

