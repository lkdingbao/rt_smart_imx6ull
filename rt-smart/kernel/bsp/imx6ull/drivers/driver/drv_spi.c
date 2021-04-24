/*
 * IMX6ULL
 *   imx6ull spi driver file
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-01-13     Lyons        first version
 */

#include <rtconfig.h>
#include <rthw.h>

#ifdef RT_USING_SPI

#include <board.h>
#ifdef RT_USING_LWP
#include <lwp.h>
#endif
 
#include "__def.h"
#include "realview.h"
#include "bsp_gpio.h"
#include "drv_spi.h"
#include "skt.h"

#define DBG_TAG "drv.spi"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

enum 
{
#ifdef BSP_USING_SPI1
    eDevSpi_SPI1, 
#endif
#ifdef BSP_USING_SPI2
    eDevSpi_SPI2, 
#endif
#ifdef BSP_USING_SPI3
    eDevSpi_SPI3, 
#endif
#ifdef BSP_USING_SPI4
    eDevSpi_SPI4, 
#endif

    eDevSpi_Max, 
};

_internal_rw struct skt_spi _s_spi_bus[eDevSpi_Max] = {
#ifdef BSP_USING_SPI1
{
    .name = "spi1",
    .periph.paddr = REALVIEW_ECSPI1_BASE,
    .gpio = {
        {IOMUXC_LCD_DATA20_ECSPI1_SCLK,     0, 0x10B1},
        {IOMUXC_LCD_DATA23_ECSPI1_MISO,     0, 0x10B1},
        {IOMUXC_LCD_DATA22_ECSPI1_MOSI,     0, 0x10B1},
    },
    .flag = 0,
},
#endif

#ifdef BSP_USING_SPI2
{
    .name = "spi2",
    .periph.paddr = REALVIEW_ECSPI2_BASE,
    .gpio = {
        {IOMUXC_UART4_TX_DATA_ECSPI2_SCLK,  0, 0x10B1},
        {IOMUXC_UART5_RX_DATA_ECSPI2_MISO,  0, 0x10B1},
        {IOMUXC_UART5_TX_DATA_ECSPI2_MOSI,  0, 0x10B1},
    },
    .flag = 0,
},
#endif

#ifdef BSP_USING_SPI3
{
    .name = "spi3",
    .periph.paddr = REALVIEW_ECSPI3_BASE,
    .gpio = {
        {IOMUXC_UART2_RX_DATA_ECSPI3_SCLK,  0, 0x10B1},
        {IOMUXC_UART2_RTS_B_ECSPI3_MISO,    0, 0x10B1},
        {IOMUXC_UART2_CTS_B_ECSPI3_MOSI,    0, 0x10B1},
    },
    .flag = 0,
},
#endif

#ifdef BSP_USING_SPI4
{
    .name = "spi4",
    .periph.paddr = REALVIEW_ECSPI4_BASE,
    .gpio = {
        {IOMUXC_ENET2_TX_DATA1_ECSPI4_SCLK, 0, 0x10B1},
        {IOMUXC_ENET2_TX_CLK_ECSPI4_MISO,   0, 0x10B1},
        {IOMUXC_ENET2_TX_EN_ECSPI4_MOSI,    0, 0x10B1},
    },
    .flag = 0,
},
#endif
};

static rt_err_t _spi_ops_configure( struct rt_spi_device *device,
                                    struct rt_spi_configuration *configuration )
{
    struct skt_spi *spi = RT_NULL;
    ecspi_master_config_t config;

    RT_ASSERT(RT_NULL != device);
    RT_ASSERT(RT_NULL != device->bus);
    RT_ASSERT(RT_NULL != configuration);

    spi = (struct skt_spi*)(device->bus);

    RT_ASSERT(configuration->mode & RT_SPI_MSB);
    RT_ASSERT(!(configuration->mode & RT_SPI_3WIRE));

    /* Master config:
     * .channel = kECSPI_Channel0; //not care! use soft nss control!
     *
     * .channelConfig.channelMode = kECSPI_Maste/kECSPI_Slave;
     * .channelConfig.clockInactiveState = kECSPI_ClockInactiveStateLow;
     * .channelConfig.dataLineInactiveState = kECSPI_DataLineInactiveStateHigh;
     * .channelConfig.chipSlectActiveState = kECSPI_ChipSelectActiveStateLow;
     * .channelConfig.waveForm = kECSPI_WaveFormSingle;
     * .channelConfig.polarity = kECSPI_PolarityActiveHigh;
     * .channelConfig.phase = kECSPI_ClockPhaseFirstEdge;
     *
     * .samplePeriodClock = kECSPI_spiClock;
     * .burstLength = 8;
     * .chipSelectDelay = 0;
     * .samplePeriod = 10;
     * .txFifoThreshold = 0;
     * .rxFifoThreshold = 0;
     * .baudRate_Bps = 500000;
     */
    ECSPI_MasterGetDefaultConfig(&config);

    config.samplePeriod = 10;
    config.txFifoThreshold = 0;
    config.channelConfig.dataLineInactiveState = kECSPI_DataLineInactiveStateHigh;

    config.channelConfig.channelMode = (configuration->mode & RT_SPI_SLAVE) ? kECSPI_Slave : kECSPI_Master;

    if (configuration->data_width <= SPI_FRAMESIZE_8BIT) 
    {
        config.burstLength = SPI_FRAMESIZE_8BIT;
    } 
    else 
    {
        return -RT_EINVAL;
    }

    switch (configuration->mode & RT_SPI_MODE_3) 
    {
        case RT_SPI_MODE_0:
            config.channelConfig.polarity = kECSPI_PolarityActiveHigh;
            config.channelConfig.phase = kECSPI_ClockPhaseFirstEdge;
            config.channelConfig.clockInactiveState = kECSPI_ClockInactiveStateLow;
            break;
        
        case RT_SPI_MODE_1:
            config.channelConfig.polarity = kECSPI_PolarityActiveHigh;
            config.channelConfig.phase = kECSPI_ClockPhaseSecondEdge;
            config.channelConfig.clockInactiveState = kECSPI_ClockInactiveStateLow;
            break;
        
        case RT_SPI_MODE_2:
            config.channelConfig.polarity = kECSPI_PolarityActiveLow;
            config.channelConfig.phase = kECSPI_ClockPhaseFirstEdge;
            config.channelConfig.clockInactiveState = kECSPI_ClockInactiveStateHigh;
            break;
        
        default:
            config.channelConfig.polarity = kECSPI_PolarityActiveLow;
            config.channelConfig.phase = kECSPI_ClockPhaseSecondEdge;
            config.channelConfig.clockInactiveState = kECSPI_ClockInactiveStateHigh;
            break;
    }

    config.baudRate_Bps = configuration->max_hz;

    ECSPI_MasterInit((ECSPI_Type*)spi->periph.vaddr, &config, HW_SYS_ECSPI_CLOCK);

    return RT_EOK;
}

static rt_uint32_t _spi_ops_xfer( struct rt_spi_device *device, 
                                  struct rt_spi_message *message )
{
    struct skt_spi *spi = RT_NULL;
    struct skt_spi_cs *spi_cs = RT_NULL;
    ECSPI_Type *periph = RT_NULL;
    const rt_uint8_t *send_ptr = RT_NULL;
    rt_uint8_t *recv_ptr = RT_NULL;
    rt_uint32_t size;
    rt_uint8_t dummy_data;
    rt_uint32_t gpio_vaddr;

    RT_ASSERT(RT_NULL != device);
    RT_ASSERT(RT_NULL != device->bus);
    RT_ASSERT(RT_NULL != message);

    spi = (struct skt_spi*)(device->bus);
    spi_cs = (struct skt_spi_cs*)(device->parent.user_data);

    if ( (RT_NULL != message->cs_take) 
      && (RT_NULL != spi_cs) ) 
    {
        gpio_vaddr = platform_get_periph_vaddr(GET_GPIO_BASE_ADDR(spi_cs->gpio_port));
        gpio_write((GPIO_Type*)gpio_vaddr, spi_cs->gpio_pin, 0);
        rt_hw_us_delay(spi_cs->cs_delay);
    }

    periph = (ECSPI_Type*)spi->periph.vaddr;
    send_ptr = message->send_buf;
    recv_ptr = message->recv_buf;
    size = message->length;

    periph->CONREG &= ~ECSPI_CONREG_CHANNEL_SELECT_MASK;
    periph->CONREG |=  ECSPI_CONREG_CHANNEL_SELECT(0);

    while (size--) 
    {
        dummy_data = (send_ptr != RT_NULL) ? (*send_ptr++) : SPI_DEFAULT_SEND_DATA;

        while (!(periph->STATREG & ECSPI_STATREG_TE_MASK));
        periph->TXDATA = dummy_data;

        while (!(periph->STATREG & ECSPI_STATREG_RR_MASK));
        dummy_data = periph->RXDATA;

        if (recv_ptr != RT_NULL) 
        {
        	*recv_ptr++ = dummy_data;
        }
    }

    if ( (RT_NULL != message->cs_release) 
      && (RT_NULL != spi_cs) ) 
    {
        gpio_vaddr = platform_get_periph_vaddr(GET_GPIO_BASE_ADDR(spi_cs->gpio_port));
        gpio_write((GPIO_Type*)gpio_vaddr, spi_cs->gpio_pin, 1);
    }

    return message->length;
}

_internal_ro struct rt_spi_ops _k_spi_ops = 
{
    _spi_ops_configure,  /* configure */
    _spi_ops_xfer,       /* xfer */
};

static void _spi_gpio_init( const struct skt_spi *device )
{
    RT_ASSERT(RT_NULL != device);

    for (int i=0; i<GET_ARRAY_NUM(device->gpio); i++)
    {
        gpio_set_iomux(&device->gpio[i]);
    }
}

rt_err_t drv_spi_bus_register( rt_uint32_t spi_periph,
                               const char *spi_bus_name )
{
    rt_err_t result;

    RT_ASSERT(RT_NULL != spi_bus_name)

    for (int i=0; i < GET_ARRAY_NUM(_s_spi_bus); i++) 
    {
        if (spi_periph == _s_spi_bus[i].periph.paddr)
        {
            if (0 == SPI_GET_SPI_PROBE_FLAG(_s_spi_bus[i].flag))
            {
                _spi_gpio_init(&_s_spi_bus[i]);

                result = rt_spi_bus_register( &_s_spi_bus[i].parent,
                                              (RT_NULL == spi_bus_name) ? _s_spi_bus[i].name : spi_bus_name,
                                              &_k_spi_ops );

                if (result < 0) 
                {
                    LOG_D("spi bus[%d] register failed!", i);
                    return -RT_ERROR;
                }

                _s_spi_bus[i].flag |= SPI_SET_SPI_PROBE_FLAG(1);
            }

            return RT_EOK;
        }
    }

    return -RT_ERROR;
}

int rt_hw_spi_init(void)
{
    for (int i=0; i < GET_ARRAY_NUM(_s_spi_bus); i++) 
    {
        _s_spi_bus[i].periph.vaddr = platform_get_periph_vaddr(_s_spi_bus[i].periph.paddr);
        LOG_D("pddr %08x vaddr %08x", _s_spi_bus[i].periph.paddr, _s_spi_bus[i].periph.vaddr);

        drv_spi_bus_register( _s_spi_bus[i].periph.paddr,
                              _s_spi_bus[i].name );
    }

    return RT_EOK;
}
INIT_DEVICE_EXPORT(rt_hw_spi_init);

#endif //#ifdef RT_USING_SPI

