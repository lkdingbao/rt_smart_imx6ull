/*
 * DS1307
 *   DS1307 driver file
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-04-29     Lyons        first version
 */

#include <rtconfig.h>
#include <rthw.h>
#include <rtdevice.h>

#ifdef RT_USING_DS1307

#include <board.h>
#ifdef RT_USING_LWP
#include <lwp.h>
#include <lwp_user_mm.h>
#endif

#include "__def.h"
#include "realview.h"
#include "drv_i2c.h"
#include "skt.h"

#define DBG_TAG "drv.ds1307"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

/* set this to 1 to enable device test demo. */
#define _DEVICE_DEBUG_EN        1

#define _DEVICE_NAME            "ds1307"
#define _BUS_NAME               "i2c2"

#define _BUS_I2C_ADDR           (0x68)

_internal_rw struct rt_device _s_ds1307_device;

static int _check_bcd( uint8_t In )
{
    if ((In&0x0F) > 0x09)
    {
        return -RT_ERROR;
    }

    if ((In>>4) > 0x09)
    {
        return -RT_ERROR;
    }

    return RT_EOK;
}

static uint8_t _bcd2bin( uint8_t In )
{
    uint8_t Hex;

    if (RT_EOK != _check_bcd(In)) {
        return 0x00;
    }
    
    Hex = In & 0xF0;
    Hex >>= 1;
    Hex += (Hex>>2);
    Hex += In & 0x0F;

    return Hex;
}

static uint8_t _bin2bcd( uint8_t In )
{
    In %= 100;
    return (In/10) * 0x10 + (In%10);
}

static int _write_data( struct rt_i2c_bus_device *i2cbus, uint8_t reg, uint8_t *buf, uint16_t len )
{
    uint8_t dummy[1+8];

    RT_ASSERT(len <= 8);

    dummy[0] = reg;
    rt_memcpy(&dummy[1], buf, len);

    i2c_write_data(i2cbus, _BUS_I2C_ADDR, dummy, 1+len);

    return RT_EOK;
}

static int _read_data( struct rt_i2c_bus_device *i2cbus, uint8_t reg, uint8_t *buf, uint8_t len )
{
    uint8_t dummy[1];

    dummy[0] = reg;
    i2c_read_data(i2cbus, _BUS_I2C_ADDR, dummy, 1, buf, len);

    return RT_EOK;
}

static time_t _get_rtc_timestamp( struct rt_i2c_bus_device *i2cdev )
{
    struct tm tm_new;
    uint8_t date_time[7];

    _read_data(i2cdev, 0x00, date_time, 7);

    tm_new.tm_sec  = _bcd2bin( date_time[0] & 0x7F );
    tm_new.tm_min  = _bcd2bin( date_time[1] & 0x7F );
    tm_new.tm_hour = _bcd2bin( date_time[2] & 0x1F ); //only support 24-hour system
    tm_new.tm_wday = _bcd2bin( date_time[3] & 0x07 ) - 1;
    tm_new.tm_mday = _bcd2bin( date_time[4] & 0x3F ) + 1;
    tm_new.tm_mon  = _bcd2bin( date_time[5] & 0x1F ) - 1;
    tm_new.tm_year = _bcd2bin( date_time[6] );

    LOG_D("get: %02x-%02x-%02x %02x:%02x:%02x", date_time[6], date_time[5], date_time[4],
                                                date_time[2]&0x1F, date_time[1], date_time[0]);

    return mktime(&tm_new);
}

static rt_err_t _set_rtc_time_stamp( struct rt_i2c_bus_device *i2cdev, time_t time_stamp )
{
    struct tm *p_tm;
    uint8_t date_time[7];

    p_tm = localtime(&time_stamp);

    date_time[0] = _bin2bcd( p_tm->tm_sec );
    date_time[1] = _bin2bcd( p_tm->tm_min );
    date_time[2] = _bin2bcd( p_tm->tm_hour ); //only support 24-hour system
    date_time[3] = _bin2bcd( p_tm->tm_wday + 1 );
    date_time[4] = _bin2bcd( p_tm->tm_mday - 1 );
    date_time[5] = _bin2bcd( p_tm->tm_mon + 1);
    date_time[6] = _bin2bcd( p_tm->tm_year);

    LOG_D("set: %02x-%02x-%02x %02x:%02x:%02x", date_time[6], date_time[5], date_time[4],
                                                date_time[2]&0x1F, date_time[1], date_time[0]);

    _write_data(i2cdev, 0x00, date_time, 7);

    LOG_D("set success.");

    return RT_EOK;
}

static rt_err_t _ds1307_ops_open( rt_device_t dev,
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

static rt_err_t _ds1307_ops_close( rt_device_t dev )
{
    RT_ASSERT(RT_NULL != dev);

    rt_device_close(dev->user_data);
    dev->user_data = RT_NULL;

    return RT_EOK;
}

static rt_err_t _ds1307_ops_control( rt_device_t dev,
                                     int cmd,
                                     void *args )
{
    struct rt_i2c_bus_device *bus = RT_NULL;
    rt_err_t result;

    RT_ASSERT(RT_NULL != dev);
    RT_ASSERT(RT_NULL != dev->user_data);

    bus = (struct rt_i2c_bus_device*)(dev->user_data);

    result = RT_EOK;
    switch (cmd)
    {
        case RT_DEVICE_CTRL_RTC_GET_TIME:
            *(rt_uint32_t *)args = _get_rtc_timestamp(bus);
            LOG_D("RTC: get rtc_time %x", *(rt_uint32_t *)args);
            break;

        case RT_DEVICE_CTRL_RTC_SET_TIME:
            if (_set_rtc_time_stamp(bus, *(rt_uint32_t *)args))
            {
                result = -RT_ERROR;
            }
            LOG_D("RTC: set rtc_time %x", *(rt_uint32_t *)args);
            break;
    }

    return result;
}

#ifdef RT_USING_DEVICE_OPS
_internal_ro struct rt_device_ops _k_ds1307_ops =
{
    RT_NULL,                /* init */
    _ds1307_ops_open,       /* open */
    _ds1307_ops_close,      /* close */
    RT_NULL,                /* read */
    RT_NULL,                /* write */
    _ds1307_ops_control,    /* control */
};
#endif

int rt_hw_ds1307_init(void)
{
    struct rt_device *device = RT_NULL;

    rt_memset(&_s_ds1307_device, 0, sizeof(_s_ds1307_device));
    device = &_s_ds1307_device;

    device->type    = RT_Device_Class_Sensor;
#ifdef RT_USING_DEVICE_OPS
    device->ops     = &_k_ds1307_ops;
#else
    device->init    = RT_NULL;
    device->open    = _ds1307_ops_open;
    device->close   = _ds1307_ops_close;
    device->read    = RT_NULL;
    device->write   = RT_NULL;
    device->control = _ds1307_ops_control;
#endif
    device->user_data = RT_NULL;

    rt_device_register(device, _DEVICE_NAME, RT_DEVICE_FLAG_RDWR);

    return RT_EOK;
}
INIT_COMPONENT_EXPORT(rt_hw_ds1307_init);

#if defined(_DEVICE_DEBUG_EN) && (_DEVICE_DEBUG_EN)
int ds1307(int argc, char **argv)
{
    rt_device_t device = RT_NULL;
    uint32_t date_time;

    device = rt_device_find(_DEVICE_NAME);
    if (RT_NULL == device)
    {
        LOG_RAW("not find device [%s].\n", _DEVICE_NAME);
        return -1;
    }

    rt_device_open(device, RT_DEVICE_FLAG_RDWR);

    /*
     * using command 'ds1307' to add 1 hour.
     * using command 'ds1307 s' to reset date-time.
     */

    if ( (2 == argc) && ('s' == argv[1][0]) )
    {
        LOG_RAW("reset date-time.\n");

        date_time = (12 * 60 * 60);
        rt_device_control(device, RT_DEVICE_CTRL_RTC_SET_TIME, &date_time);
    }
    else
    {
        LOG_RAW("get date-time.\n");
    }

    rt_device_control(device, RT_DEVICE_CTRL_RTC_GET_TIME, &date_time);

    rt_device_close(device);

    return 0;
}
MSH_CMD_EXPORT_ALIAS(ds1307, ds1307, <usr> ds1307 device test);
#endif //#if defined(_DEVICE_DEBUG_EN) && (_DEVICE_DEBUG_EN)

#endif //#ifdef RT_USING_DS1307

