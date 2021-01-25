/*
 * GT9147
 *   GT9147 driver file
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-01-25     Lyons        first version
 */

#include <rtconfig.h>
#include <rthw.h>
#include <rtdevice.h>

#include <board.h>
#include <lwp.h>
#include <lwp_user_mm.h>

#include "__def.h"
#include "realview.h"
#include "drv_pin.h"
#include "drv_i2c.h"
#include "drv_gt9147.h"
#include "skt.h"

#define DBG_TAG "GT9147"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

#define _DEVICE_NAME        "gt9147"
#define _BUS_NAME           "i2c2"

#define _BUS_I2C_ADDR       0x14

#define _TOUCH_POINT_NUM    5

#define INT_PIN         GET_PIN(0,9)
#define RST_PIN         GET_PIN(4,9)
_internal_ro struct skt_gpio _k_gpio_info[] = 
{
    {IOMUXC_GPIO1_IO09_GPIO1_IO09,        0, 0x0080}, //int pin(int mode), must at the first!
    {IOMUXC_GPIO1_IO09_GPIO1_IO09,        0, 0x10B0}, //int pin(out mode)
    {IOMUXC_SNVS_SNVS_TAMPER9_GPIO5_IO09, 0, 0x10B0}, //rst pin
};

_internal_ro struct skt_irq _k_gpio_irq_info =
{
    .name = "touch",
    .irqno = GPIO1_Combined_0_15_IRQn,
};

_internal_rw struct rt_device _s_gt9147_device;

_internal_rw rt_uint8_t _s_gt9147_tpnum = 0;
_internal_rw struct TouchPointData _s_gt9147_tpdata[_TOUCH_POINT_NUM];

/*
 * GT9147 Configuration
 * The first Byte(e.g. 0x41) is the version SN of driver IC
 * When the new version SN is not less than the old one
 * New configuration will take effect
 */
_internal_ro rt_uint8_t _s_gt9147_init_tbl[] = 
{
    0x46,0x0E,0x01,0x10,0x01,0x05,0x0D,0x00,0x01,0x08,
    0x28,0x05,0x50,0x32,0x03,0x05,0x00,0x00,0xFF,0xFF,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x89,0x28,0x0A,
    0x17,0x15,0x31,0x0D,0x00,0x00,0x02,0x9B,0x03,0x25,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x32,0x00,0x00,
    0x00,0x0F,0x94,0x94,0xC5,0x02,0x07,0x00,0x00,0x04,
    0x8D,0x13,0x00,0x5C,0x1E,0x00,0x3C,0x30,0x00,0x29,
    0x4C,0x00,0x1E,0x78,0x00,0x1E,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x08,0x0A,0x0C,0x0E,0x10,0x12,0x14,0x16,
    0x18,0x1A,0x00,0x00,0x00,0x00,0x1F,0xFF,0xFF,0xFF,
    0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
    0xFF,0xFF,0x00,0x02,0x04,0x05,0x06,0x08,0x0A,0x0C,
    0x0E,0x1D,0x1E,0x1F,0x20,0x22,0x24,0x28,0x29,0xFF,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFF,0xFF,
    0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
    0xFF,0xFF,0xFF,0xFF,
};

rt_uint8_t _g_gt9147_flag = 0;

static void _write_data( rt_device_t dev, rt_uint16_t reg, rt_uint8_t *data, rt_uint16_t len )
{
    rt_uint8_t  bus_addr = _BUS_I2C_ADDR;

    rt_device_control(dev, RT_I2C_DEV_CTRL_ADDR, &bus_addr);
    rt_device_write(dev, (RT_I2C_REG_ADDR_16BIT<<16) | reg, data, len);
}

static void _read_data( rt_device_t dev, rt_uint16_t reg, rt_uint8_t *data, rt_uint16_t len )
{
    rt_uint8_t bus_addr = _BUS_I2C_ADDR;

    rt_device_control(dev, RT_I2C_DEV_CTRL_ADDR, &bus_addr);
    rt_device_read(dev, (RT_I2C_REG_ADDR_16BIT<<16) | reg, data, len);
}

static rt_uint8_t _get_verison( rt_device_t dev )
{
    rt_uint8_t pid[5];
    rt_uint8_t data;

    _read_data(dev, GT_PID_REG, pid, 4);
    pid[4] = 0;

    data = 0x02;
    _write_data(dev, GT_CTRL_REG, &data, 1);

    _read_data(dev, GT_CFGS_REG, &data, 1);

    LOG_D("Product ID: %s Ver: %02X", pid, data);

    return data;
}

static void _get_touchdata( rt_device_t dev )
{
    rt_uint8_t dummy[4];
    rt_uint8_t tpnum;

    _read_data(dev, GT_RESULT_REG, &tpnum, 1);
    if ( !(tpnum & 0x80) )
    {
        rt_hw_ms_delay(1);
        _read_data(dev, GT_RESULT_REG, &tpnum, 1);
        if ( !(tpnum & 0x80) )
        {
            return;
        }
    }
    tpnum &= 0x0F;

    dummy[0] = 0;
    _write_data(dev, GT_RESULT_REG, dummy, 1);

    _s_gt9147_tpnum = tpnum;

    if (0 == _s_gt9147_tpnum) {
        return; //must clear GT_RESULT_REG register before return!
    }

    for (int i=0; i<_TOUCH_POINT_NUM; i++)
    {
        _read_data(dev, GT_TP1_REG+(8*i), dummy, 4);

        _s_gt9147_tpdata[i].xPos = (dummy[1]<<8) | dummy[0];
        _s_gt9147_tpdata[i].yPos = (dummy[3]<<8) | dummy[2];
        _s_gt9147_tpdata[i].flag = 1;
    }

    LOG_D("tpnum: %02X", _s_gt9147_tpnum);
    LOG_D("x_pos: %d %d %d %d %d", _s_gt9147_tpdata[0].xPos,
                                   _s_gt9147_tpdata[1].yPos,
                                   _s_gt9147_tpdata[2].xPos,
                                   _s_gt9147_tpdata[3].yPos,
                                   _s_gt9147_tpdata[4].yPos );
    LOG_D("y_pos: %d %d %d %d %d", _s_gt9147_tpdata[0].yPos,
                                   _s_gt9147_tpdata[1].yPos,
                                   _s_gt9147_tpdata[2].yPos,
                                   _s_gt9147_tpdata[3].yPos,
                                   _s_gt9147_tpdata[4].yPos );
}

static void _set_configuration( rt_device_t dev, rt_uint8_t svFlag )
{
    rt_uint8_t check_data[2];

    check_data[0] = 0;
    check_data[1] = svFlag & 0x1;
    for (int i=0; i<GET_ARRAY_NUM(_s_gt9147_init_tbl); i++) {
        check_data[0] += _s_gt9147_init_tbl[i];
    }
    check_data[0] = (~check_data[0]) + 1;

    _write_data(dev, GT_CFGS_REG, (rt_uint8_t*)_s_gt9147_init_tbl, GET_ARRAY_NUM(_s_gt9147_init_tbl));
    _write_data(dev, GT_CHECK_REG, check_data, GET_ARRAY_NUM(check_data));

    rt_hw_ms_delay(10);
}

static void _gt9147_int_isr( int irqno, void* parameter )
{
    rt_uint32_t paddr, vaddr;
    rt_uint8_t port_num, pin_num;

    _g_gt9147_flag = GT_FLAG_NEW_DATA;

    port_num = GET_PORT_FIELD(INT_PIN);
    pin_num = GET_PIN_FIELD(INT_PIN);

    paddr = GET_GPIO_BASE_ADDR(port_num);
    vaddr = platform_get_periph_vaddr((rt_uint32_t)paddr);

    GPIO_ClearPinsInterruptFlags((GPIO_Type*)vaddr, (1 << pin_num));
}

/*
 * mode=0, config the INT_PIN at interrupt mode
 * mode=1, config the INT_PIN/RST_PIN at output mode
 */
static void _gt9147_gpio_init( rt_uint8_t mode )
{
    gpio_pin_config_t config;
    rt_uint32_t paddr, vaddr;
    rt_uint8_t port_num, pin_num;

    if (0 == mode)
    {
        gpio_set_iomux(&_k_gpio_info[0]);

        port_num = GET_PORT_FIELD(INT_PIN);
        pin_num = GET_PIN_FIELD(INT_PIN);

        config.direction = kGPIO_DigitalInput;
        config.interruptMode = kGPIO_IntRisingOrFallingEdge;
        config.outputLogic = PIN_LOW;

        paddr = GET_GPIO_BASE_ADDR(port_num);
        vaddr = platform_get_periph_vaddr((rt_uint32_t)paddr);

        GPIO_PinInit((GPIO_Type*)vaddr, pin_num, &config);
        GPIO_EnableInterrupts((GPIO_Type*)vaddr, (1 << pin_num));
    }
    else
    {
        for (int i=1; i<GET_ARRAY_NUM(_k_gpio_info); i++)
        {
            gpio_set_iomux(&_k_gpio_info[i]);
        }

        config.direction = kGPIO_DigitalOutput;
        config.interruptMode = kGPIO_NoIntmode;
        config.outputLogic = PIN_HIGH;

        port_num = GET_PORT_FIELD(INT_PIN);
        pin_num = GET_PIN_FIELD(INT_PIN);

        paddr = GET_GPIO_BASE_ADDR(port_num);
        vaddr = platform_get_periph_vaddr((rt_uint32_t)paddr);

        GPIO_PinInit((GPIO_Type*)vaddr, pin_num, &config);

        port_num = GET_PORT_FIELD(RST_PIN);
        pin_num = GET_PIN_FIELD(RST_PIN);

        paddr = GET_GPIO_BASE_ADDR(port_num);
        vaddr = platform_get_periph_vaddr((rt_uint32_t)paddr);

        GPIO_PinInit((GPIO_Type*)vaddr, pin_num, &config);
    }
}

static rt_err_t _gt9147_device_init( struct rt_i2c_bus_device *device )
{
    rt_uint8_t data;

    RT_ASSERT(RT_NULL != device);

    _gt9147_gpio_init(1); //config the INT_PIN/RST_PIN at output mode

    rt_pin_write(RST_PIN, PIN_LOW);
    rt_hw_ms_delay(10);

    rt_pin_write(RST_PIN, PIN_HIGH);
    rt_hw_ms_delay(10);

    rt_pin_write(INT_PIN, PIN_LOW); //sel slave chip address with 0x28/0x29
    rt_hw_ms_delay(100);

    _gt9147_gpio_init(0); //config the INT_PIN at interrupt mode
    rt_hw_ms_delay(50);

    _get_verison((rt_device_t)device);
    _set_configuration((rt_device_t)device, 0);

    data = 0;
    _write_data((rt_device_t)device, GT_CTRL_REG, &data, 1);

    _read_data((rt_device_t)device, GT_CFGS_REG, &data, 1);
    LOG_D("New Ver %02X", data);

    return RT_EOK;
}

static rt_err_t _gt9147_ops_open( rt_device_t dev,
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

        _gt9147_device_init(dev->user_data);

        rt_hw_interrupt_install(_k_gpio_irq_info.irqno, _gt9147_int_isr, RT_NULL, _k_gpio_irq_info.name);
        rt_hw_interrupt_umask(_k_gpio_irq_info.irqno);
    }

    return RT_EOK;
}

static rt_err_t _gt9147_ops_close( rt_device_t dev )
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

static rt_size_t _gt9147_ops_read( rt_device_t dev, 
                                   rt_off_t pos, 
                                   void *buffer, 
                                   rt_size_t size )
{
    struct rt_i2c_bus_device *bus = RT_NULL;

    RT_ASSERT(RT_NULL != dev);
    RT_ASSERT(RT_NULL != dev->user_data);

    bus = (struct rt_i2c_bus_device*)(dev->user_data);

    _get_touchdata((rt_device_t)bus);

    return RT_EOK;
}

#ifdef RT_USING_DEVICE_OPS
_internal_ro struct rt_device_ops _k_gt9147_ops =
{
    RT_NULL,                /* init */
    _gt9147_ops_open,       /* open */
    _gt9147_ops_close,      /* close */
    _gt9147_ops_read,       /* read */
    RT_NULL,                /* write */
    RT_NULL,                /* control */
};
#endif

int rt_hw_gt9147_init(void)
{
    struct rt_device *device = RT_NULL;

    rt_memset(&_s_gt9147_device, 0x00, sizeof(_s_gt9147_device));
    device = &_s_gt9147_device;

    device->type    = RT_Device_Class_Sensor;

#ifdef RT_USING_DEVICE_OPS
    device->ops     = &_k_gt9147_ops;
#else
    device->init    = RT_NULL;
    device->open    = _gt9147_ops_open;
    device->close   = _gt9147_ops_close;
    device->read    = _gt9147_ops_read;
    device->write   = RT_NULL;
    device->control = RT_NULL;
#endif

    device->user_data = RT_NULL;

    rt_device_register(device, _DEVICE_NAME, RT_DEVICE_FLAG_WRONLY);

    return RT_EOK;
}
INIT_COMPONENT_EXPORT(rt_hw_gt9147_init);

