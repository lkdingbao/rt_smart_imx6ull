/*
 * IMX6ULL
 *   imx6ull i2c driver file
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-01-13     Lyons        first version
 */

#include <rtconfig.h>
#include <rthw.h>

#ifdef RT_USING_I2C

#include <board.h>
#ifdef RT_USING_LWP
#include <lwp.h>
#endif
     
#include "__def.h"
#include "realview.h"
#include "bsp_gpio.h"
#include "drv_i2c.h"
#include "skt.h"
    
#define DBG_TAG "i2c"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

enum 
{
#ifdef BSP_USING_I2C1
    eDevI2c_I2C1, 
#endif
#ifdef BSP_USING_I2C2
    eDevI2c_I2C2, 
#endif
#ifdef BSP_USING_I2C3
    eDevI2c_I2C3, 
#endif
#ifdef BSP_USING_I2C4
    eDevI2c_I2C4, 
#endif

eDevI2c_Max, 
};

_internal_rw struct skt_i2c _s_i2c_bus[eDevI2c_Max] = {
#ifdef BSP_USING_I2C1
{
    .name = "i2c1",
    .periph.paddr = REALVIEW_I2C1_BASE,
    .gpio = {
        {IOMUXC_GPIO1_IO02_I2C1_SCL,      1, 0X70B0},
        {IOMUXC_GPIO1_IO03_I2C1_SDA,      1, 0X70B0},
//        {IOMUXC_UART4_TX_DATA_I2C1_SCL,   1, 0X70B0},
//        {IOMUXC_UART4_RX_DATA_I2C1_SDA,   1, 0X70B0},
//        {IOMUXC_CSI_PIXCLK_I2C1_SCL,      1, 0X70B0},
//        {IOMUXC_CSI_MCLK_I2C1_SDA,        1, 0X70B0},
    },
    .flag = 0,
},
#endif

#ifdef BSP_USING_I2C2
{
    .name = "i2c2",
    .periph.paddr = REALVIEW_I2C2_BASE,
    .gpio = {
//        {IOMUXC_GPIO1_IO00_I2C2_SCL,      1, 0X70B0},
//        {IOMUXC_GPIO1_IO01_I2C2_SDA,      1, 0X70B0},
        {IOMUXC_UART5_TX_DATA_I2C2_SCL,   1, 0X70B0},
        {IOMUXC_UART5_RX_DATA_I2C2_SDA,   1, 0X70B0},
//        {IOMUXC_CSI_HSYNC_I2C2_SCL,       1, 0X70B0},
//        {IOMUXC_CSI_VSYNC_I2C2_SDA,       1, 0X70B0},
    },
    .flag = 0,
},
#endif

#ifdef BSP_USING_I2C3
    .name = "i2c3",
    .periph.paddr = REALVIEW_I2C3_BASE,
    .gpio = {
        {IOMUXC_UART1_TX_DATA_I2C3_SCL,   1, 0X70B0},
        {IOMUXC_UART1_RX_DATA_I2C3_SDA,   1, 0X70B0},
//        {IOMUXC_ENET2_RX_DATA0_I2C3_SCL,  1, 0X70B0},
//        {IOMUXC_ENET2_RX_DATA1_I2C3_SDA,  1, 0X70B0},
//        {IOMUXC_LCD_DATA01_I2C3_SCL,      1, 0X70B0},
//        {IOMUXC_LCD_DATA00_I2C3_SDA,      1, 0X70B0},
    },
    .flag = 0,
},
#endif

#ifdef BSP_USING_I2C4
    .name = "i2c4",
    .periph.paddr = REALVIEW_I2C4_BASE,
    .gpio = {
        {IOMUXC_UART2_TX_DATA_I2C4_SCL,   1, 0X70B0},
        {IOMUXC_UART2_RX_DATA_I2C4_SDA,   1, 0X70B0},
//        {IOMUXC_ENET2_RX_EN_I2C4_SCL,     1, 0X70B0},
//        {IOMUXC_ENET2_TX_DATA0_I2C4_SDA,  1, 0X70B0},
//        {IOMUXC_LCD_DATA02_I2C4_SDA,      1, 0X70B0},
//        {IOMUXC_LCD_DATA00_I2C3_SDA,      1, 0X70B0},
    },
    .flag = 0,
},
#endif
};

static void _i2c_gpio_init( const struct skt_i2c *device )
{
    RT_ASSERT(RT_NULL != device);

    for (int i=0; i<GET_ARRAY_NUM(device->gpio); i++)
    {
        gpio_set_iomux(&device->gpio[i]);
    }
}

static void _i2c_init( const struct skt_i2c *device )
{
    i2c_master_config_t config;

    RT_ASSERT(RT_NULL != device);

    /* Master config:
     * .baudRate_Bps = 100000;
     * .enableMaster = true;
     */
    I2C_MasterGetDefaultConfig(&config);

    I2C_MasterInit((I2C_Type*)device->periph.vaddr, &config, HW_SYS_I2C_CLOCK);
}

static rt_size_t _i2c_bus_ops_master_xfer( struct rt_i2c_bus_device *device, 
                                           struct rt_i2c_msg msgs[],
                                           rt_uint32_t num )
{
    struct skt_i2c *i2c = RT_NULL;
    i2c_master_transfer_t message_xfer;
    rt_uint32_t idx;

    RT_ASSERT(RT_NULL != device);

    i2c = (struct skt_i2c*)device;

    for (idx=0; idx<num; idx++) 
    {
        message_xfer.slaveAddress = device->addr;
        message_xfer.subaddress = msgs[idx].addr; //little-endian!
        message_xfer.subaddressSize = (msgs[idx].flags & RT_I2C_REG_ADDR_16BIT) ? 2 : 1;
        message_xfer.data = msgs[idx].buf;
        message_xfer.dataSize = msgs[idx].len;
        message_xfer.flags = kI2C_TransferDefaultFlag;

        switch (msgs[idx].flags & RT_I2C_RD)
        {
            case RT_I2C_WR:
                message_xfer.direction = kI2C_Write;
                break;

            case RT_I2C_RD:
                message_xfer.direction = kI2C_Read;
                break;

            default:
                break;
        }

        I2C_MasterTransferBlocking((I2C_Type*)i2c->periph.vaddr, &message_xfer);
    }

    return idx;
}

_internal_ro struct rt_i2c_bus_device_ops _k_i2c_bus_ops = 
{
    _i2c_bus_ops_master_xfer,   /* master_xfer */
    RT_NULL,                    /* slave_xfer */
    RT_NULL,                    /* i2c_bus_control */
};

rt_err_t drv_i2c_bus_register( rt_uint32_t i2c_periph,
                               const char *i2c_bus_name )
{
    rt_err_t result;

    for (int i=0; i < GET_ARRAY_NUM(_s_i2c_bus); i++) 
    {
        if (i2c_periph == _s_i2c_bus[i].periph.paddr)
        {
            if (0 == I2C_GET_I2C_PROBE_FLAG(_s_i2c_bus[i].flag))
            {
                _i2c_gpio_init(&_s_i2c_bus[i]);
                _i2c_init(&_s_i2c_bus[i]);

                _s_i2c_bus[i].parent.ops = &_k_i2c_bus_ops;
                result = rt_i2c_bus_device_register( &_s_i2c_bus[i].parent,
                                                     (RT_NULL == i2c_bus_name) ? _s_i2c_bus[i].name : i2c_bus_name );

                if (result < 0) 
                {
                    LOG_D("i2c bus[%d] register failed!", i);
                    return -RT_ERROR;
                }

                _s_i2c_bus[i].flag |= I2C_SET_I2C_PROBE_FLAG(1);
            }

            return RT_EOK;
        }
    }

    return -RT_ERROR;
}

int rt_hw_i2c_init(void)
{
    for (int i=0; i < GET_ARRAY_NUM(_s_i2c_bus); i++) 
    {
        _s_i2c_bus[i].periph.vaddr = platform_get_periph_vaddr(_s_i2c_bus[i].periph.paddr);
        LOG_D("pddr %08x vaddr %08x", _s_i2c_bus[i].periph.paddr, _s_i2c_bus[i].periph.vaddr);
    }

    drv_i2c_bus_register( _s_i2c_bus[eDevI2c_I2C2].periph.paddr,
                          _s_i2c_bus[eDevI2c_I2C2].name );

    return RT_EOK;
}
INIT_DEVICE_EXPORT(rt_hw_i2c_init);

#endif //#ifdef RT_USING_I2C

