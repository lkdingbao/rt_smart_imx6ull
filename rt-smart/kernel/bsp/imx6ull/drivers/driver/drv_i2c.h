/*
 * IMX6ULL
 *   imx6ull i2c driver file
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-01-13     Lyons        first version
 */

#ifndef __DRV_I2C_H__
#define __DRV_I2C_H__

#include "skt.h"

#ifdef RT_USING_I2C

rt_err_t drv_i2c_bus_register( rt_uint32_t i2c_periph,
                               const char *i2c_bus_name );

void i2c_write_data( struct rt_i2c_bus_device *bus, rt_uint16_t addr,
                     const rt_uint8_t *inbuf, rt_uint32_t inlen );

void i2c_read_data( struct rt_i2c_bus_device *bus, rt_uint16_t addr,
                    const rt_uint8_t *inbuf, rt_uint32_t inlen,
                    rt_uint8_t *outbuf, rt_uint32_t outlen );

void i2c_write_one_data( struct rt_i2c_bus_device *bus, rt_uint16_t addr,
                         rt_uint16_t reg, rt_uint8_t reglen,
                         rt_uint8_t data );

rt_uint8_t i2c_read_one_data( struct rt_i2c_bus_device *bus, rt_uint16_t addr,
                              rt_uint16_t reg, rt_uint8_t reglen );

#endif //#ifdef RT_USING_I2C
#endif //#ifndef __DRV_I2C_H__

