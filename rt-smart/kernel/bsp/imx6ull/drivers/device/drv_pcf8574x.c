/*
 * PCF8574x
 *   PCF8574x driver file
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-01-17     Lyons        first version
 */

#include <rtconfig.h>
#include <rthw.h>
#include <rtdevice.h>

#ifdef RT_USING_PCF8574

#include <board.h>
#include <lwp.h>
#include <lwp_user_mm.h>

#include "__def.h"
#include "realview.h"
#include "drv_i2c.h"
#include "drv_pcf8574x.h"
#include "skt.h"

#define DBG_TAG "PCF8574x"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

#define _DEVICE_NAME        "pcf8574x"
#define _BUS_NAME           "i2c2"

#define _BUS_I2C_ADDR       0x3F

#define _ROW_SIZE           16  //lcd1602 includes 0x40 chars per row
#define _COL_SIZE           2   //lcd1602 includes 2 rows

_internal_rw struct rt_device _s_pcf8574x_device;

_internal_ro rt_uint8_t _s_pcf8574x_init_tbl[] = 
{
    0x33,   //equal 0x3 + 0x3
    0x32,   //equal 0x3 + 0x2, enter 4-bit mode, need three 0x3 and one 0x2
    0x28,   //4-bit bus, 2-row, 5¡Á7
    0x08,   //display off, no cursor, not blink
    0x01,   //display clear
    0x06,   //cursor auto right move, address auto inc
    0x0C,   //display on, no cursor, not blink
};

static void _write_data( rt_device_t dev, rt_uint8_t data )
{
    rt_uint8_t temp, dummy;

    for (int i=0 ;i<2; i++)
    {
        temp = (data << (4*i)) | 0x0F;

        dummy = temp & 0xFD;
        rt_device_write(dev, _BUS_I2C_ADDR, &dummy, 1);
        rt_hw_us_delay(2);

        dummy = temp & 0xF9;
        rt_device_write(dev, _BUS_I2C_ADDR, &dummy, 1);
    }
}

static void _write_command( rt_device_t dev, rt_uint8_t data )
{
    rt_uint8_t temp, dummy;

    for (int i=0 ;i<2; i++)
    {
        temp = (data << (4*i)) | 0x0F;

        dummy = temp & 0xFC;
        rt_device_write(dev, _BUS_I2C_ADDR, &dummy, 1);
        rt_hw_us_delay(2);

        dummy = temp & 0xF8;
        rt_device_write(dev, _BUS_I2C_ADDR, &dummy, 1);
    }
}

static void _pcf8574x_gpio_init( void )
{
    /* Is empty function! */
}

static rt_err_t _pcf8574x_device_init( struct rt_i2c_bus_device *device )
{
    RT_ASSERT(RT_NULL != device);

    for (int i=0; i<GET_ARRAY_NUM(_s_pcf8574x_init_tbl); i++)
    {
        _write_command((rt_device_t)device, _s_pcf8574x_init_tbl[i]);
        rt_hw_us_delay(100);
    }

    return RT_EOK;
}

static rt_err_t _pcf8574x_ops_open( rt_device_t dev,
                                    rt_uint16_t oflag )
{
    struct rt_device *i2c_bus = RT_NULL;

    RT_ASSERT(RT_NULL != dev);

    if (!(dev->flag & RT_DEVICE_FLAG_WRONLY))
    {
        LOG_D("only support write option!");
        return -RT_ERROR;
    }

    dev->ref_count++;

    if (1 == dev->ref_count)
    {
        i2c_bus = rt_device_find(_BUS_NAME);
        if (RT_NULL == i2c_bus)
        {
            LOG_D("not find bus device[%s]", i2c_bus);
            return -RT_EIO;
        }

        rt_device_open(i2c_bus, RT_DEVICE_OFLAG_RDWR);
        dev->user_data = i2c_bus;

        _pcf8574x_gpio_init();
        _pcf8574x_device_init(dev->user_data);
    }

    return RT_EOK;
}

static rt_err_t _pcf8574x_ops_close( rt_device_t dev )
{
    RT_ASSERT(RT_NULL != dev);

    if (1 == dev->ref_count)
    {
        rt_device_close(dev->user_data);
        dev->user_data = RT_NULL;
    }
    dev->ref_count = (0 == dev->ref_count) ? 0 : (dev->ref_count - 1);

    return RT_EOK;
}

static rt_size_t _pcf8574x_ops_write( rt_device_t dev, 
                                      rt_off_t pos, 
                                      const void *buffer, 
                                      rt_size_t size )
{
    struct rt_i2c_bus_device *bus = RT_NULL;
    rt_uint8_t *pdata = RT_NULL;
    rt_size_t idx, col, row;

    RT_ASSERT(RT_NULL != dev);
    RT_ASSERT(RT_NULL != dev->user_data);

    bus = (struct rt_i2c_bus_device*)(dev->user_data);

    pdata = (rt_uint8_t*)buffer;

    row = pos / _ROW_SIZE;
    col = pos % _ROW_SIZE;

    idx = 0;
    for (; row<_COL_SIZE; row++)
    {
        _write_command(&bus->parent, (0x80+(0x40*row))|(col&0x3F));

        for (; idx<size; idx++)
        {
            col++;
            if (_ROW_SIZE == col)
                break;

            _write_data(&bus->parent, pdata[idx]);
        }
        col = 0;

        if (size == idx)
            break;
    }

    return RT_EOK;
}

#ifdef RT_USING_DEVICE_OPS
_internal_ro struct rt_device_ops _k_pcf8574x_ops =
{
    RT_NULL,                /* init */
    _pcf8574x_ops_open,     /* open */
    _pcf8574x_ops_close,    /* close */
    RT_NULL,                /* read */
    _pcf8574x_ops_write,    /* write */
    RT_NULL,                /* control */
};
#endif

int rt_hw_pcf8574x_init(void)
{
    struct rt_device *device = RT_NULL;

    rt_memset(&_s_pcf8574x_device, 0x00, sizeof(_s_pcf8574x_device));
    device = &_s_pcf8574x_device;

    device->type    = RT_Device_Class_Sensor;

#ifdef RT_USING_DEVICE_OPS
    device->ops     = &_k_pcf8574x_ops;
#else
    device->init    = RT_NULL;
    device->open    = _pcf8574x_ops_open;
    device->close   = _pcf8574x_ops_close;
    device->read    = RT_NULL;
    device->write   = _pcf8574x_ops_write;
    device->control = RT_NULL;
#endif

    device->user_data = RT_NULL;

    rt_device_register(device, _DEVICE_NAME, RT_DEVICE_FLAG_WRONLY);

    return RT_EOK;
}
INIT_COMPONENT_EXPORT(rt_hw_pcf8574x_init);

#endif //#ifdef RT_USING_PCF8574
