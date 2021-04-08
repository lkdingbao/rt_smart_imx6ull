/*
 * LAN9720A
 *   LAN9720A driver file
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-01-30     Lyons        first version
 */

#include <rtconfig.h>
#include <rthw.h>
#include <rtdevice.h>

#define RT_USING_LAN8720
#ifdef RT_USING_LAN8720

#include <board.h>
#include <lwp.h>
#include <lwp_user_mm.h>

#include "__def.h"
#include "realview.h"
#include "drv_pin.h"
//#include "drv_lan8720.h"
#include "skt.h"

#define DBG_TAG "LAN8720"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

#define _DEVICE_NAME        "lan8720"

#define RST_PIN             GET_PIN(4,8)
_internal_ro struct skt_gpio _k_gpio_info[] = 
{
    {IOMUXC_GPIO1_IO06_ENET2_MDIO,              0U, 0xB829},
    {IOMUXC_GPIO1_IO07_ENET2_MDC,               0U, 0xB0E9},

    {IOMUXC_ENET2_TX_DATA0_ENET2_TDATA00,       0U, 0xB0E9},
    {IOMUXC_ENET2_TX_DATA1_ENET2_TDATA01,       0U, 0xB0E9},
    {IOMUXC_ENET2_TX_CLK_ENET2_REF_CLK2,        0U, 0x0031}, //to ENET2_TX_CLK(CLKIN)
    {IOMUXC_ENET2_TX_EN_ENET2_TX_EN,            0U, 0xB0E9},

    {IOMUXC_ENET2_RX_DATA0_ENET2_RDATA00,       0U, 0xB0E9},
    {IOMUXC_ENET2_RX_DATA1_ENET2_RDATA01,       0U, 0xB0E9},
    {IOMUXC_ENET2_RX_EN_ENET2_RX_EN,            0U, 0xB0E9}, //to ENET2_CRS_DV
    {IOMUXC_ENET2_RX_ER_ENET2_RX_ER,            0U, 0xB0E9},

    {IOMUXC_SNVS_SNVS_TAMPER8_GPIO5_IO08,       0U, 0x70A1}, //to ENET2_RST
};

_internal_rw struct rt_device _s_lan8720_device;

static void _lan8720_gpio_init( void )
{
    gpio_pin_config_t config;
    rt_uint32_t paddr, vaddr;
    rt_uint8_t port_num, pin_num;
    rt_uint32_t reg_value;

    for (int i=0; i<GET_ARRAY_NUM(_k_gpio_info); i++)
    {
        gpio_set_iomux(&_k_gpio_info[i]);
    }

    IOMUXC_GPR_Type *_IOMUXC_GPR = (IOMUXC_GPR_Type*)platform_get_periph_vaddr(REALVIEW_IOMUXC_GPR_BASE);

    reg_value = _IOMUXC_GPR->GPR1;
    reg_value &= ~(IOMUXC_GPR_GPR1_ENET2_CLK_SEL_MASK 
                 | IOMUXC_GPR_GPR1_ENET2_CLK_SEL_MASK);
    reg_value |=  IOMUXC_GPR_GPR1_ENET2_TX_CLK_DIR(1);
    reg_value |=  IOMUXC_GPR_GPR1_ENET2_CLK_SEL(0);
    _IOMUXC_GPR->GPR1 = reg_value;

    config.direction = kGPIO_DigitalOutput;
    config.interruptMode = kGPIO_NoIntmode;
    config.outputLogic = PIN_LOW;

    port_num = GET_PORT_FIELD(RST_PIN);
    pin_num = GET_PIN_FIELD(RST_PIN);

    paddr = GET_GPIO_BASE_ADDR(port_num);
    vaddr = platform_get_periph_vaddr((rt_uint32_t)paddr);

    GPIO_PinInit((GPIO_Type*)vaddr, pin_num, &config);

    GPIO_WritePinOutput((GPIO_Type*)vaddr, pin_num, 0);
    rt_hw_ms_delay(20);
    GPIO_WritePinOutput((GPIO_Type*)vaddr, pin_num, 1);
}

#define EXAMPLE_ENET                ((ENET_Type*)rt_hw_kernel_phys_to_virt(ENET2, sizeof(ENET_Type)))
#define EXAMPLE_PHY                 (1)

#define CORE_CLK_FREQ               CLOCK_GetFreq(kCLOCK_AhbClk)

#define APP_ENET_BUFF_ALIGNMENT     MAX(ENET_BUFF_ALIGNMENT, FSL_FEATURE_L1DCACHE_LINESIZE_BYTE)

#define ENET_RXBD_NUM               (4)
#define ENET_TXBD_NUM               (4)
#define ENET_RXBUFF_SIZE            (ENET_FRAME_MAX_FRAMELEN)
#define ENET_TXBUFF_SIZE            (ENET_FRAME_MAX_FRAMELEN)

#define ENET_DATA_LENGTH            (128)

_internal_rw uint32_t *s_rxBuffDescrip = RT_NULL;
_internal_rw uint32_t *s_txBuffDescrip = RT_NULL;
_internal_rw uint32_t *s_rxDataBuff = RT_NULL;
_internal_rw uint32_t *s_txDataBuff = RT_NULL;
_internal_rw uint8_t *s_macAddr = RT_NULL;

_internal_rw enet_handle_t s_handle;
_internal_rw uint8_t s_frame[ENET_DATA_LENGTH + 14];

_internal_ro uint8_t k_src_mac_addr[6] = {0xd4, 0xbe, 0xd9, 0x45, 0x22, 0x60};
_internal_ro uint8_t k_tgt_mac_addr[6] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};

static rt_err_t BOARD_InitModuleClock( void )
{
    _internal_ro clock_enet_pll_config_t config = {
        false,      /* enableClkOutput0 */
        true,       /* enableClkOutput1 */
        false,      /* enableClkOutput2 */
        1,          /* 0b01 = 50M, for ENET0 */
        1,          /* 0b01 = 50M, for ENET1 */
                    /* fixed 25M,  for ENET2 */
    };

    CLOCK_InitEnetPll(&config);
    return RT_EOK;
}

static void ENET_BuildBroadCastFrame(void)
{
    uint32_t count = 0;
    uint32_t length = ENET_DATA_LENGTH - 14;

    rt_memcpy(&s_frame[0], k_tgt_mac_addr, 6U);
    rt_memcpy(&s_frame[6], k_src_mac_addr, 6U);

    s_frame[12] = (length >> 8) & 0xFFU;
    s_frame[13] = length & 0xFFU;

    for (count = 0; count < length; count++)
    {
        s_frame[count + 14] = count % 0xFFU;
    }
}

void _enet_callback( ENET_Type *base, enet_handle_t *handle, enet_event_t event, void *userData )
{
    LOG_D("_enet_callback");
}

static rt_err_t _lan8720_device_init( void )
{
    enet_config_t config;
    phy_speed_t speed;
    phy_duplex_t duplex;
    uint32_t sysClock;
    status_t result;
    bool link;

    s_rxBuffDescrip = (uint32_t*)rt_pages_alloc(rt_page_bits(8192));
    s_txBuffDescrip = (uint32_t*)rt_pages_alloc(rt_page_bits(8192));
    s_rxDataBuff = (uint32_t*)rt_pages_alloc(rt_page_bits(8192));
    s_txDataBuff  = (uint32_t*)rt_pages_alloc(rt_page_bits(8192));
    s_macAddr = (uint8_t*)rt_pages_alloc(rt_page_bits(6));

    rt_memcpy(s_macAddr, k_src_mac_addr, 6U);

    enet_buffer_config_t buffConfig = {
        ENET_RXBD_NUM,
        ENET_TXBD_NUM,
        SDK_SIZEALIGN(ENET_RXBUFF_SIZE, APP_ENET_BUFF_ALIGNMENT),
        SDK_SIZEALIGN(ENET_TXBUFF_SIZE, APP_ENET_BUFF_ALIGNMENT),
        (volatile enet_rx_bd_struct_t*)(s_rxBuffDescrip),
        (volatile enet_tx_bd_struct_t*)(s_txBuffDescrip),
        (uint8_t*)(s_rxDataBuff),
        (uint8_t*)(s_txDataBuff),
        PV_OFFSET,
    };

    BOARD_InitModuleClock();

    /* Get default configuration. */
    /*
     * config.miiMode = kENET_RmiiMode;
     * config.miiSpeed = kENET_MiiSpeed100M;
     * config.miiDuplex = kENET_MiiFullDuplex;
     * config.rxMaxFrameLen = ENET_FRAME_MAX_FRAMELEN;
     */
    ENET_GetDefaultConfig(&config);

    sysClock = CORE_CLK_FREQ;

    result = PHY_Init(EXAMPLE_ENET, EXAMPLE_PHY, sysClock);
    LOG_D("PHY init %s.", (result?"failed":"success"));

    PHY_GetLinkStatus(EXAMPLE_ENET, EXAMPLE_PHY, &link);
    LOG_D("PHY link %s.", (link?"up":"down"));

    if (link)
    {
        /* Get the actual PHY link speed. */
        PHY_GetLinkSpeedDuplex(EXAMPLE_ENET, EXAMPLE_PHY, &speed, &duplex);
        /* Change the MII speed and duplex for actual link status. */
        config.miiSpeed = (enet_mii_speed_t)speed;
        config.miiDuplex = (enet_mii_duplex_t)duplex;

        LOG_D("PHY speed %dM, duplex %s.", (config.miiSpeed?100:10), (config.miiDuplex?"full":"half"));
    }
    else
    {
        LOG_D("PHY Link down, please check the cable connection and link partner setting.");
    }

//    config.interrupt = kENET_TxFrameInterrupt
//                     | kENET_RxFrameInterrupt;
//    ENET_SetCallback(&s_handle, _enet_callback, RT_NULL);

    ENET_Init(EXAMPLE_ENET, &s_handle, &config, &buffConfig, s_macAddr, sysClock);
    ENET_ActiveRead(EXAMPLE_ENET);

    LOG_D("lan8720 init finished.");

    ENET_BuildBroadCastFrame();
    result = ENET_SendFrame(EXAMPLE_ENET, &s_handle, s_frame, ENET_DATA_LENGTH);
    LOG_D("PHY send %s.", (result?"failed":"success"));

    UNUSED(result);
    return RT_EOK;
}

static rt_err_t _lan8720_ops_open( rt_device_t dev,
                                   rt_uint16_t oflag )
{
    RT_ASSERT(RT_NULL != dev);

    if (!(dev->flag & RT_DEVICE_FLAG_RDWR))
    {
        LOG_W("only support rd/wr option!");
        return -RT_ERROR;
    }

    dev->ref_count++;

    if (1 == dev->ref_count)
    {
        dev->user_data = RT_NULL;
    }

    return RT_EOK;
}

static rt_err_t _lan8720_ops_close( rt_device_t dev )
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

static rt_size_t _lan8720_ops_read( rt_device_t dev, 
                                    rt_off_t pos, 
                                    void *buffer, 
                                    rt_size_t size )
{
    RT_ASSERT(RT_NULL != dev);

    return RT_EOK;
}

#ifdef RT_USING_DEVICE_OPS
_internal_ro struct rt_device_ops _k_lan8720_ops =
{
    RT_NULL,                /* init */
    _lan8720_ops_open,      /* open */
    _lan8720_ops_close,     /* close */
    _lan8720_ops_read,      /* read */
    RT_NULL,                /* write */
    RT_NULL,                /* control */
};
#endif

int rt_hw_lan8720_init(void)
{
    struct rt_device *device = RT_NULL;

    rt_memset(&_s_lan8720_device, 0x00, sizeof(_s_lan8720_device));
    device = &_s_lan8720_device;

    device->type    = RT_Device_Class_PHY;

#ifdef RT_USING_DEVICE_OPS
    device->ops     = &_k_lan8720_ops;
#else
    device->init    = RT_NULL;
    device->open    = _lan8720_ops_open;
    device->close   = _lan8720_ops_close;
    device->read    = _lan8720_ops_read;
    device->write   = RT_NULL;
    device->control = RT_NULL;
#endif

    device->user_data = RT_NULL;

    rt_device_register(device, _DEVICE_NAME, RT_DEVICE_FLAG_RDWR);

    _lan8720_gpio_init();
    _lan8720_device_init();

    return RT_EOK;
}
INIT_DEVICE_EXPORT(rt_hw_lan8720_init);

#endif //#ifdef RT_USING_LAN8720

