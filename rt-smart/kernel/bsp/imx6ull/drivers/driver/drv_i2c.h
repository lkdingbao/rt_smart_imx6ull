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

#ifndef HW_SYS_I2C_CLOCK
#define HW_SYS_I2C_CLOCK                (68000000U)
#endif

#define I2C_PROBE_FLAG_SHIFT            (0)
#define I2C_GET_I2C_PROBE_FLAG(d)       ((d) & (1 << (I2C_PROBE_FLAG_SHIFT)))
#define I2C_SET_I2C_PROBE_FLAG(d)       (((d) & 0x1) << (I2C_PROBE_FLAG_SHIFT))

#define I2C_FRAMESIZE_8BIT              (8)
#define I2C_DEFAULT_SEND_DATA           (0xFF)

rt_err_t drv_i2c_bus_register( rt_uint32_t i2c_periph,
                               const char *i2c_bus_name );

#endif //#ifndef __DRV_I2C_H__

