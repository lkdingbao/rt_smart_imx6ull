/*
 * MPU6050
 *   MPU6050 driver file
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-04-29     Lyons        first version
 */

#include <rtconfig.h>
#include <rthw.h>
#include <rtdevice.h>

#ifdef RT_USING_MPU6050

#include <board.h>
#ifdef RT_USING_LWP
#include <lwp.h>
#include <lwp_user_mm.h>
#endif

#include "__def.h"
#include "realview.h"
#include "drv_i2c.h"
#include "skt.h"

#define DBG_TAG "drv.mpu6050"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

/* set this to 1 to enable device test demo. */
#define _DEVICE_DEBUG_EN        1

#define _DEVICE_NAME            "mpu6050"
#define _BUS_NAME               "i2c2"

/* the LSB value is rely on hardware AD0 pin. */
#define _BUS_I2C_ADDR           (0x68|0x00)

_internal_rw struct rt_device _s_mpu6050_device;

static rt_err_t _mpu6050_device_init( struct rt_i2c_bus_device *device )
{
    RT_ASSERT(RT_NULL != device);

    return RT_EOK;
}

static rt_err_t _mpu6050_ops_open( rt_device_t dev,
                                   rt_uint16_t oflag )
{
    struct rt_device *i2c_bus = RT_NULL;

    RT_ASSERT(RT_NULL != dev);

    if (!(dev->flag & RT_DEVICE_FLAG_RDONLY))
    {
        LOG_D("only support read option!");
        return -RT_ERROR;
    }

    i2c_bus = rt_device_find(_BUS_NAME);
    if (RT_NULL == i2c_bus)
    {
        LOG_D("not find bus device[%s]", i2c_bus);
        return -RT_EIO;
    }

    rt_device_open(i2c_bus, RT_DEVICE_OFLAG_RDWR);
    dev->user_data = i2c_bus;

    _mpu6050_device_init(dev->user_data);

    return RT_EOK;
}

static rt_err_t _mpu6050_ops_close( rt_device_t dev )
{
    RT_ASSERT(RT_NULL != dev);

    rt_device_close(dev->user_data);
    dev->user_data = RT_NULL;

    return RT_EOK;
}

static rt_size_t _mpu6050_ops_write( rt_device_t dev, 
                                     rt_off_t pos, 
                                     const void *buffer, 
                                     rt_size_t size )
{
    return 0;
}

#ifdef RT_USING_DEVICE_OPS
_internal_ro struct rt_device_ops _k_mpu6050_ops =
{
    RT_NULL,                /* init */
    _mpu6050_ops_open,      /* open */
    _mpu6050_ops_close,     /* close */
    RT_NULL,                /* read */
    _mpu6050_ops_write,     /* write */
    RT_NULL,                /* control */
};
#endif

int rt_hw_mpu6050_init(void)
{
    struct rt_device *device = RT_NULL;

    rt_memset(&_s_mpu6050_device, 0, sizeof(_s_mpu6050_device));
    device = &_s_mpu6050_device;

    device->type    = RT_Device_Class_Sensor;
#ifdef RT_USING_DEVICE_OPS
    device->ops     = &_k_mpu6050_ops;
#else
    device->init    = RT_NULL;
    device->open    = _mpu6050_ops_open;
    device->close   = _mpu6050_ops_close;
    device->read    = RT_NULL;
    device->write   = _mpu6050_ops_write;
    device->control = RT_NULL;
#endif
    device->user_data = RT_NULL;

    rt_device_register(device, _DEVICE_NAME, RT_DEVICE_FLAG_RDONLY);

    return RT_EOK;
}
INIT_COMPONENT_EXPORT(rt_hw_mpu6050_init);

#if defined(_DEVICE_DEBUG_EN) && (_DEVICE_DEBUG_EN)
int mpu6050(int argc, char **argv)
{
    rt_device_t device = RT_NULL;

    device = rt_device_find(_DEVICE_NAME);
    if (RT_NULL == device)
    {
        LOG_RAW("not find device [%s].\n", _DEVICE_NAME);
        return -1;
    }

    rt_device_open(device, RT_DEVICE_FLAG_RDONLY);

    //you can do something here.

    rt_device_close(device);

    return 0;
}
MSH_CMD_EXPORT_ALIAS(mpu6050, mpu6050, <usr> mpu6050 device test);
#endif //#if defined(_DEVICE_DEBUG_EN) && (_DEVICE_DEBUG_EN)

#endif //#ifdef RT_USING_MPU6050

