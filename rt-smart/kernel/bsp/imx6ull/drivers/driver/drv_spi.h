/*
 * IMX6ULL
 *   imx6ull spi driver file
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-01-13     Lyons        first version
 */

#ifndef __DRV_SPI_H__
#define __DRV_SPI_H__

#include "skt.h"

#ifdef RT_USING_SPI

#ifndef HW_SYS_ECSPI_CLOCK
#define HW_SYS_ECSPI_CLOCK              (480000000 / 8)
#endif

#define SPI_PROBE_FLAG_SHIFT            (0)
#define SPI_GET_SPI_PROBE_FLAG(d)       ((d) & (1 << (SPI_PROBE_FLAG_SHIFT)))
#define SPI_SET_SPI_PROBE_FLAG(d)       (((d) & 0x1) << (SPI_PROBE_FLAG_SHIFT))

#define SPI_FRAMESIZE_8BIT              (8)
#define SPI_DEFAULT_SEND_DATA           (0xFF)

rt_err_t drv_spi_bus_register( rt_uint32_t spi_periph,
                               const char *spi_bus_name );

#endif //#ifdef RT_USING_SPI
#endif //#ifndef __DRV_SPI_H__

