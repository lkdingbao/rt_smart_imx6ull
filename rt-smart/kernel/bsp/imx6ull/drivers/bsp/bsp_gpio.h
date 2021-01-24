/*
 * IMX6ULL
 *   imx6ull gpio bsp file
 *
 * Change Logs:
 * Date           Author       Notes
 * 2020-12-20     Lyons        first version
 */

#ifndef __BSP_GPIO_H__
#define __BSP_GPIO_H__
 
#include <board.h>
#include "skt.h"

#define GET_GPIO_BASE_ADDR(n)       (rt_uint32_t)(REALVIEW_GPIO1_BASE + (n) * REALVIEW_PERIPH_SIZE)

#define GET_PORT_FIELD(d)           (rt_uint8_t)(((d) >> 5) & 0x07)
#define GET_PIN_FIELD(d)            (rt_uint8_t)( (d) & 0x1F)

void gpio_set_iomux(const struct skt_gpio *gpio);
void gpio_set_mode(GPIO_Type *port, rt_uint32_t pin, const gpio_pin_config_t* cfg);
void gpio_write(GPIO_Type *port, rt_uint32_t pin, rt_uint32_t lvl);
rt_uint32_t gpio_read(GPIO_Type *port, rt_uint32_t pin);

#endif //#ifndef __BSP_GPIO_H__
 
