/*
 * IMX6ULL
 *   imx6ull uart driver file
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-01-09     Lyons        first version
 */

#ifndef __DRV_UART_H__
#define __DRV_UART_H__

#include "skt.h"

#ifdef RT_USING_UART

#ifndef HW_UART_BUS_CLOCK
#define HW_UART_BUS_CLOCK                   80000000
#endif

#define RT_SERIAL_CONFIG_115200N81          \
{                                           \
    BAUD_RATE_115200,                       \
    DATA_BITS_8,                            \
    STOP_BITS_1,                            \
    PARITY_NONE,                            \
    BIT_ORDER_LSB,                          \
    NRZ_NORMAL,                             \
    RT_SERIAL_RB_BUFSZ,                     \
    0                                       \
}

#endif //#ifdef RT_USING_UART
#endif //#ifndef __DRV_UART_H__

