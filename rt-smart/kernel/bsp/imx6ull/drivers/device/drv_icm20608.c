/*
 * ICM20608
 *   icm20608 driver file
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-01-11     Lyons        first version
 */

#include <rtconfig.h>
#include <rthw.h>
#include <rtdevice.h>

#ifdef RT_USING_ICM20608

#include <board.h>
#include <lwp.h>
#include <lwp_user_mm.h>

#include "__def.h"
#include "realview.h"
#include "bsp_gpio.h"
#include "drv_pin.h"
#include "drv_spi.h"
#include "drv_icm20608.h"
#include "skt.h"

#define DBG_TAG "ICM20608"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

#define _DEVICE_NAME        "icm20608"
#define _BUS_NAME           "spi3"

#define NSS_PIN         GET_PIN(0,20)
_internal_rw struct skt_gpio _s_gpio_info[] = 
{
    {IOMUXC_UART2_TX_DATA_GPIO1_IO20, 0, 0X10B0},
};

/* Only use as user_data for param config */
_internal_rw struct skt_spi_cs _s_icm20608_nss_info = 
{
    .gpio_port = GET_PORT_FIELD(NSS_PIN),
    .gpio_pin = GET_PIN_FIELD(NSS_PIN),
    .cs_delay = 0,
    .data_delay = 0,
};

_internal_rw struct rt_spi_device _s_spi_device;

_internal_ro rt_uint8_t _s_im20608_init_tbl[][2] = 
{
    { ICM20_PWR_MGMT_1,     0x80,},
    { ICM20_PWR_MGMT_1,     0x01,},
    { ICM20_SMPLRT_DIV,     0x00,},
    { ICM20_GYRO_CONFIG,    0x18,},
    { ICM20_ACCEL_CONFIG,   0x18,},
    { ICM20_CONFIG,         0x04,},
    { ICM20_ACCEL_CONFIG2,  0x04,},
    { ICM20_PWR_MGMT_2,     0x00,},
    { ICM20_LP_MODE_CFG,    0x00,},
    { ICM20_FIFO_EN,        0x00,},
};

static void _icm20806_gpio_init( void )
{
    gpio_pin_config_t config;
    rt_uint32_t paddr, vaddr;
    rt_uint8_t port_num, pin_num;

    for (int i=0; i<GET_ARRAY_NUM(_s_gpio_info); i++)
    {
        gpio_set_iomux(&_s_gpio_info[i]);
    }

    port_num = GET_PORT_FIELD(NSS_PIN);
    pin_num = GET_PIN_FIELD(NSS_PIN);

    config.direction = kGPIO_DigitalOutput;
    paddr = GET_GPIO_BASE_ADDR(port_num);
    vaddr = platform_get_periph_vaddr((rt_uint32_t)paddr);
    gpio_set_mode((GPIO_Type*)vaddr, pin_num, &config);
}

static rt_err_t _icm20806_device_init( struct rt_spi_device *device )
{
    RT_ASSERT(RT_NULL != device);

    for (int i=0; i<2; i++)
    {
        rt_spi_transfer(device, &_s_im20608_init_tbl[i], RT_NULL, 2);
        rt_hw_ms_delay(50);
    }

    for (int i=2; i<GET_ARRAY_NUM(_s_im20608_init_tbl); i++)
    {
        rt_spi_transfer(device, &_s_im20608_init_tbl[i], RT_NULL, 2);
    }

    return RT_EOK;
}

int rt_hw_icm20806_init(void)
{
    struct rt_spi_configuration cfg;
    rt_err_t result;

    result = rt_spi_bus_attach_device(&_s_spi_device, _DEVICE_NAME, _BUS_NAME, &_s_icm20608_nss_info);
    if (RT_EOK != result)
    {
        LOG_D("not find bus device[%s]", _BUS_NAME);
        return -RT_EIO;
    }

    cfg.mode = RT_SPI_MASTER
             | RT_SPI_MSB
             | RT_SPI_MODE_3;
    cfg.data_width = 8;
    cfg.max_hz = 1*1000*1000;
    rt_spi_configure(&_s_spi_device, &cfg);

    _icm20806_gpio_init();
    _icm20806_device_init(&_s_spi_device);

    return RT_EOK;
}
INIT_COMPONENT_EXPORT(rt_hw_icm20806_init);

#endif //#ifdef RT_USING_ICM20608
