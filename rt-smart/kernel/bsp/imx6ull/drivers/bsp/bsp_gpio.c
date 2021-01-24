/*
 * IMX6ULL
 *   imx6ull gpio bsp file
 *
 * Change Logs:
 * Date           Author       Notes
 * 2020-12-20     Lyons        first version
 */

#include <rtconfig.h>
#include <rthw.h>

#include <board.h>

#include "__def.h"
#include "realview.h"
#include "bsp_gpio.h"

#define _P2V(pa)                    platform_get_periph_vaddr((pa))

void gpio_set_iomux(const struct skt_gpio *gpio)
{
    rt_uint32_t v[3];

    v[0] = gpio->muxRegister ? _P2V(gpio->muxRegister) : gpio->muxRegister;
    v[1] = gpio->inputRegister ? _P2V(gpio->inputRegister) : gpio->inputRegister;
    v[2] = gpio->configRegister ? _P2V(gpio->configRegister) : gpio->configRegister;

    IOMUXC_SetPinMux(v[0], gpio->muxMode, v[1], gpio->inputDaisy, v[2], gpio->inputOnfield);
    IOMUXC_SetPinConfig(v[0], gpio->muxMode, v[1], gpio->inputDaisy, v[2], gpio->configValue);
}

void gpio_set_mode(GPIO_Type *port, rt_uint32_t pin, const gpio_pin_config_t* cfg)
{
    /* Register reset to default value */
    port->IMR &= ~(1U << pin);

    /* Configure GPIO pin direction */
    if (cfg->direction == kGPIO_DigitalInput)
    {
        port->GDIR &= ~(1U << pin);
    }
    else
    {
        gpio_write(port, pin, cfg->outputLogic);
        port->GDIR |= (1U << pin);
    }
}

void gpio_write(GPIO_Type *port, rt_uint32_t pin, rt_uint32_t lvl)
{
    RT_ASSERT(pin < 32);

    if (0 == lvl)
    {
        port->DR &= ~(1 << pin);
    }else {
        port->DR |=  (1 << pin);
    }
}

rt_uint32_t gpio_read(GPIO_Type *port, rt_uint32_t pin)
{
    RT_ASSERT(pin < 32);

    return ((port->PSR >> pin) & 0x1);
}

