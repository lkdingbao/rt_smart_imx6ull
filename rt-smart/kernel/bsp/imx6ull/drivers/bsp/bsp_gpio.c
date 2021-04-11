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

_internal_rw rt_uint32_t _s_gpio_last_vaddr = 0;
_internal_rw rt_uint32_t _s_gpio_last_pin = 0;

void gpio_set_iomux(const struct skt_gpio *gpio)
{
    rt_uint32_t v[3];

    v[0] = gpio->muxRegister ? _P2V(gpio->muxRegister) : gpio->muxRegister;
    v[1] = gpio->inputRegister ? _P2V(gpio->inputRegister) : gpio->inputRegister;
    v[2] = gpio->configRegister ? _P2V(gpio->configRegister) : gpio->configRegister;

    IOMUXC_SetPinMux(v[0], gpio->muxMode, v[1], gpio->inputDaisy, v[2], gpio->inputOnfield);
    IOMUXC_SetPinConfig(v[0], gpio->muxMode, v[1], gpio->inputDaisy, v[2], gpio->configValue);
}

void gpio_set_int_mode(GPIO_Type *port, rt_uint32_t pin, const gpio_interrupt_mode_t pinInterruptMode)
{
    volatile rt_uint32_t *icr;
    rt_uint32_t icrShift;

    icrShift = pin;

    /* Register reset to default value */
    port->EDGE_SEL &= ~(1U << pin);

    if(pin < 16)
    {
        icr = (volatile rt_uint32_t*)&(port->ICR1);
    }
    else
    {
        icr = (volatile rt_uint32_t*)&(port->ICR2);
        icrShift -= 16;
    }
    switch(pinInterruptMode)
    {
        case(kGPIO_IntLowLevel):
            *icr &= ~(3U << (2 * icrShift));
            break;
        case(kGPIO_IntHighLevel):
            *icr = (*icr & (~(3U << (2 * icrShift)))) | (1U << (2 * icrShift));
            break;
        case(kGPIO_IntRisingEdge):
            *icr = (*icr & (~(3U << (2 * icrShift)))) | (2U << (2 * icrShift));
            break;
        case(kGPIO_IntFallingEdge):
            *icr |= (3U << (2 * icrShift));
            break;
        case(kGPIO_IntRisingOrFallingEdge):
            port->EDGE_SEL |= (1U << pin);
            break;
        default:
            break;
    }
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

    /* Configure GPIO pin interrupt mode */
    gpio_set_int_mode(port, pin, cfg->interruptMode);
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

void gpio_config(rt_uint32_t pn, gpio_pin_config_t *config)
{
    rt_uint32_t paddr, vaddr;
    rt_uint8_t port_num, pin_num;

    RT_ASSERT(RT_NULL != config);

    port_num = GET_PORT_FIELD(pn);
    pin_num = GET_PIN_FIELD(pn);

    paddr = GET_GPIO_BASE_ADDR(port_num);
    vaddr = platform_get_periph_vaddr((rt_uint32_t)paddr);

    gpio_set_mode((GPIO_Type*)vaddr, pin_num, config);
}

void gpio_output(rt_uint32_t pn, rt_uint32_t lvl)
{
    rt_uint32_t paddr, vaddr;
    rt_uint8_t port_num, pin_num;

    port_num = GET_PORT_FIELD(pn);
    pin_num = GET_PIN_FIELD(pn);

    paddr = GET_GPIO_BASE_ADDR(port_num);
    vaddr = platform_get_periph_vaddr((rt_uint32_t)paddr);

    gpio_write((GPIO_Type*)vaddr, pin_num, lvl&0x1);
}

void gpio_easy_set_output_mode(rt_uint32_t pin)
{
    gpio_pin_config_t config;
    rt_uint32_t paddr, vaddr;
    rt_uint8_t port_num, pin_num;

    port_num = GET_PORT_FIELD(pin);
    pin_num = GET_PIN_FIELD(pin);

    config.direction = kGPIO_DigitalOutput;
    paddr = GET_GPIO_BASE_ADDR(port_num);
    vaddr = platform_get_periph_vaddr((rt_uint32_t)paddr);
    gpio_set_mode((GPIO_Type*)vaddr, pin_num, &config);

    _s_gpio_last_vaddr = vaddr;
    _s_gpio_last_pin = pin_num;
}

void gpio_easy_write(rt_uint32_t lvl)
{
    /* no need to check! is only for test! */
    gpio_write((GPIO_Type*)_s_gpio_last_vaddr, _s_gpio_last_pin, lvl&0x1);
}

