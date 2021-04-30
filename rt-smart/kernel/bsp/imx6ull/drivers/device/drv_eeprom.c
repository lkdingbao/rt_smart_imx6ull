/*
 * EEPROM
 *   EEPROM driver file
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-04-29     Lyons        first version
 */

#include <rtconfig.h>
#include <rthw.h>
#include <rtdevice.h>

#ifdef RT_USING_EEPROM

#include <board.h>
#ifdef RT_USING_LWP
#include <lwp.h>
#include <lwp_user_mm.h>
#endif

#include "__def.h"
#include "realview.h"
#include "drv_i2c.h"
#include "skt.h"

#define DBG_TAG "drv.eeprom"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

/* set this to 1 to enable device test demo. */
#define _DEVICE_DEBUG_EN        1

#define _DEVICE_NAME            "eeprom"
#define _BUS_NAME               "i2c2"

/* the LSB value is rely on hardware circuit. */
#define _BUS_I2C_ADDR           (0x50|0x0)

#ifndef ALIGN_DOWN
#define ALIGN_DOWN(size, align) ((size) & ~((align) - 1))
#endif

#ifndef ALIGN_UP
#define ALIGN_UP(size, align)   (ALIGN_DOWN(size, align) + align)
#endif

#define _EEPROM_MEM_POOL_SIZE   (256)

#ifdef BSP_EEPROM_DOUBLE_BAK_EN
#define _EEPROM_USE_DOUBLE_BAK
#endif

#define _EEPROM_PAGE_SIZE       BSP_EEPROM_PAGE_SIZE

#ifdef  _EEPROM_USE_DOUBLE_BAK
#define _EEPROM_SIZE            (BSP_EEPROM_SIZE / 2)
#else
#define _EEPROM_SIZE            (BSP_EEPROM_SIZE)
#endif

#define _EEPROM_BASE            (0x0)
#define _EEPROM_ADDR_START      (_EEPROM_BASE)
#define _EEPROM_ADDR_END        (_EEPROM_ADDR_START + _EEPROM_SIZE - 1)

#ifdef  _EEPROM_USE_DOUBLE_BAK
#define _EEPROM_BAK_BASE        (_EEPROM_SIZE)
#define _EEPROM_BAK_ADDR_START  (_EEPROM_BAK_BASE)
#define _EEPROM_BAK_ADDR_END    (_EEPROM_BAK_ADDR_START + _EEPROM_SIZE - 1)
#endif

#define _EEPROM_DELAY_MS(t)     rt_hw_ms_delay(t)
#define _EEPROM_DELAY_US(t)     rt_hw_us_delay(t)

_internal_rw struct rt_device _s_eeprom_device;

#ifdef _EEPROM_USE_DOUBLE_BAK
static void _get_negate_data( uint8_t *pIn, uint8_t *pOut, uint16_t Length )
{
    uint16_t i;

    if ((RT_NULL == pIn) || (RT_NULL == pOut))
    {
        return;
    }

    for (i=0; i<Length; i++)
    {
        pOut[i] = pIn[i] ^ 0xFF;
    }
}
#endif //#ifdef _EEPROM_USE_DOUBLE_BAK

static int _write_data( rt_device_t i2cdev, uint16_t reg, uint8_t *buf, uint16_t len )
{
    struct rt_i2c_bus_device *bus = (struct rt_i2c_bus_device*)i2cdev;
    uint8_t dummy[2+_EEPROM_PAGE_SIZE];

    RT_ASSERT(len <= _EEPROM_PAGE_SIZE);

    dummy[0] = reg >> 8;
    dummy[1] = reg;
    rt_memcpy(&dummy[2], buf, len);

    i2c_write_data(bus, _BUS_I2C_ADDR, dummy, 2+len);

    return RT_EOK;
}

static int _read_data( rt_device_t i2cdev, uint16_t reg, uint8_t *buf, uint8_t len )
{
    struct rt_i2c_bus_device *bus = (struct rt_i2c_bus_device*)i2cdev;
    uint8_t dummy[2];

    dummy[0] = reg >> 8;
    dummy[1] = reg;
    i2c_read_data(bus, _BUS_I2C_ADDR, dummy, 2, buf, len);

    return RT_EOK;
}

static int _eeprom_read( rt_device_t i2cdev, uint32_t addr, uint8_t *buf, uint16_t size )
{
    uint32_t op_addr;
    uint16_t total_size, count, offset;

    total_size = size;
    op_addr = addr;
    offset = 0;
    count = 0;

    while (total_size > 0)
    {
        if (op_addr % _EEPROM_PAGE_SIZE)
        {
            count = _EEPROM_PAGE_SIZE - (op_addr % _EEPROM_PAGE_SIZE);
            count = (count > total_size) ? total_size : count;
        }
        else
        {
            count = (total_size > _EEPROM_PAGE_SIZE) ? _EEPROM_PAGE_SIZE : total_size;
        }

        RT_ASSERT( (op_addr+count) <= ALIGN_UP(op_addr, _EEPROM_PAGE_SIZE) );

        if (RT_EOK != _read_data(i2cdev, op_addr, &buf[offset], count))
        {
            break;
        }

        total_size -= count;
        op_addr += count;
        offset += count;
    }

    return offset;
}

static int _eeprom_write( rt_device_t i2cdev, uint32_t addr, const uint8_t *buf, uint16_t size )
{
    uint32_t op_addr;
    uint16_t total_size, count, offset;

    total_size = size;
    op_addr = addr;
    offset = 0;
    count = 0;

    _EEPROM_DELAY_MS(5);

    while (total_size > 0)
    {
        if (op_addr % _EEPROM_PAGE_SIZE)
        {
            count = _EEPROM_PAGE_SIZE - (op_addr % _EEPROM_PAGE_SIZE);
            count = (count > total_size) ? total_size : count;
        }
        else
        {
            count = (total_size > _EEPROM_PAGE_SIZE) ? _EEPROM_PAGE_SIZE : total_size;
        }

        RT_ASSERT( (op_addr+count) <= ALIGN_UP(op_addr, _EEPROM_PAGE_SIZE) );

        if (RT_EOK != _write_data(i2cdev, op_addr, (uint8_t*)&buf[offset], count))
        {
            break;
        }
        _EEPROM_DELAY_MS(6);

        total_size -= count;
        op_addr += count;
        offset += count;
    }

    _EEPROM_DELAY_MS(5);

    return offset;
}

static rt_err_t _eeprom_ops_open( rt_device_t dev,
                                  rt_uint16_t oflag )
{
    struct rt_device *i2c_bus = RT_NULL;

    RT_ASSERT(RT_NULL != dev);

    if (!(dev->flag & RT_DEVICE_FLAG_RDWR))
    {
        LOG_D("only support wr/rd option!");
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

    return RT_EOK;
}

static rt_err_t _eeprom_ops_close( rt_device_t dev )
{
    RT_ASSERT(RT_NULL != dev);

    rt_device_close(dev->user_data);
    dev->user_data = RT_NULL;

    return RT_EOK;
}

static rt_size_t _eeprom_ops_read( rt_device_t dev,
                                   rt_off_t pos,
                                   void *buffer,
                                   rt_size_t size )
{
    struct rt_i2c_bus_device *bus = RT_NULL;
    uint8_t _ee_mem_pool[_EEPROM_MEM_POOL_SIZE];
    rt_size_t read_len;

    RT_ASSERT(RT_NULL != dev);
    RT_ASSERT(RT_NULL != dev->user_data);

    RT_ASSERT( (pos+size) <= _EEPROM_SIZE );
    if ((pos + size) > _EEPROM_SIZE)
    {
        return 0;
    }

    RT_ASSERT( size <= _EEPROM_MEM_POOL_SIZE );
    if (size > _EEPROM_MEM_POOL_SIZE)
    {
        return 0;
    }

    bus = (struct rt_i2c_bus_device*)(dev->user_data);

    do
    {
        read_len = _eeprom_read( (rt_device_t)bus, _EEPROM_BASE + pos, buffer, size );
        if ((0 == read_len) || (size != read_len))
        {
            break;
        }

#ifdef _EEPROM_USE_DOUBLE_BAK
        read_len = _eeprom_read( (rt_device_t)bus, _EEPROM_BAK_BASE + pos, _ee_mem_pool, size );
        if ((0 == read_len) || (size != read_len))
        {
            break;
        }

        _get_negate_data( _ee_mem_pool, _ee_mem_pool, size );
        if (0 != rt_memcmp(buffer, _ee_mem_pool, size))
        {
            RT_ASSERT( 0 != 0 );
            read_len = 0;
        }
#endif //#ifdef _EEPROM_USE_DOUBLE_BAK
    } while(0);

    UNUSED(_ee_mem_pool);
    return read_len;
}

static rt_size_t _eeprom_ops_write( rt_device_t dev,
                                    rt_off_t pos,
                                    const void *buffer,
                                    rt_size_t size )
{
    struct rt_i2c_bus_device *bus = RT_NULL;
    uint8_t _ee_mem_pool[_EEPROM_MEM_POOL_SIZE];
    rt_size_t write_len;
    rt_size_t read_len;

    RT_ASSERT(RT_NULL != dev);
    RT_ASSERT(RT_NULL != dev->user_data);

    RT_ASSERT( (pos+size) <= _EEPROM_SIZE );
    if ((pos + size) > _EEPROM_SIZE)
    {
        return 0;
    }

    RT_ASSERT( size <= _EEPROM_MEM_POOL_SIZE );
    if (size > _EEPROM_MEM_POOL_SIZE)
    {
        return 0;
    }

    bus = (struct rt_i2c_bus_device*)(dev->user_data);

    do
    {
        write_len = _eeprom_write( (rt_device_t)bus, _EEPROM_BASE + pos, buffer, size );
        if ((0 == write_len) || (size != write_len))
        {
            break;
        }

#ifdef _EEPROM_USE_DOUBLE_BAK
        _get_negate_data( _ee_mem_pool, (uint8_t*)buffer, size );
        write_len = _eeprom_write( (rt_device_t)bus, _EEPROM_BAK_BASE + pos, _ee_mem_pool, size );
        if ((0 == write_len) || (size != write_len))
        {
            break;
        }
#endif //#ifdef _EEPROM_USE_DOUBLE_BAK

        read_len = _eeprom_ops_read( (rt_device_t)bus, pos, _ee_mem_pool, size );
        if ((0 == read_len) || (write_len != read_len))
        {
            break;
        }

        if (0 != rt_memcmp((uint8_t*)buffer, _ee_mem_pool, size))
        {
            RT_ASSERT( 0 != 0 );
            write_len = 0;
        }
    } while(0);

    UNUSED(_ee_mem_pool);
    return write_len;
}

#ifdef RT_USING_DEVICE_OPS
_internal_ro struct rt_device_ops _k_eeprom_ops =
{
    RT_NULL,                /* init */
    _eeprom_ops_open,       /* open */
    _eeprom_ops_close,      /* close */
    _eeprom_ops_read,       /* read */
    _eeprom_ops_write,      /* write */
    RT_NULL,                /* control */
};
#endif

int rt_hw_eeprom_init(void)
{
    struct rt_device *device = RT_NULL;

    rt_memset(&_s_eeprom_device, 0, sizeof(_s_eeprom_device));
    device = &_s_eeprom_device;

    device->type    = RT_Device_Class_Sensor;
#ifdef RT_USING_DEVICE_OPS
    device->ops     = &_k_eeprom_ops;
#else
    device->init    = RT_NULL;
    device->open    = _eeprom_ops_open;
    device->close   = _eeprom_ops_close;
    device->read    = _eeprom_ops_read;
    device->write   = _eeprom_ops_write;
    device->control = RT_NULL;
#endif
    device->user_data = RT_NULL;

    rt_device_register(device, _DEVICE_NAME, RT_DEVICE_FLAG_RDWR);

    return RT_EOK;
}
INIT_COMPONENT_EXPORT(rt_hw_eeprom_init);

#if defined(_DEVICE_DEBUG_EN) && (_DEVICE_DEBUG_EN)
int eeprom(int argc, char **argv)
{
    rt_device_t device = RT_NULL;
    uint8_t dummy[_EEPROM_PAGE_SIZE];
    uint8_t base;

    device = rt_device_find(_DEVICE_NAME);
    if (RT_NULL == device)
    {
        LOG_RAW("not find device [%s].\n", _DEVICE_NAME);
        return -1;
    }

    if (2 != argc)
    {
        base = 0;
    } else {
        base = atoi(argv[1]);
    }

    rt_device_open(device, RT_DEVICE_FLAG_RDWR);

    for (int i=0; i<_EEPROM_PAGE_SIZE; i++) {
        dummy[i] = base + (i%256);
    }

    rt_device_write(device, 0, dummy, sizeof(dummy));

    rt_memset(dummy, 0, sizeof(dummy));
    rt_device_read(device, 0, dummy, sizeof(dummy));

    LOG_RAW("eeprom test data: base [%02x]\n", base);
    for (int i=0; i<_EEPROM_PAGE_SIZE; i++)
    {
        LOG_RAW("%02x ", dummy[i]);
        if ( (_EEPROM_PAGE_SIZE == (i+1)) || (15 == (i%16))) {
            LOG_RAW("\n");
        }
    }

    rt_device_close(device);

    return 0;
}
MSH_CMD_EXPORT_ALIAS(eeprom, eeprom, <usr> eeprom device test);
#endif //#if defined(_DEVICE_DEBUG_EN) && (_DEVICE_DEBUG_EN)

#endif //#ifdef RT_USING_EEPROM

