/*
 * LAN8720A
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
    {IOMUXC_ENET2_TX_CLK_ENET2_REF_CLK2,        1U, 0x0031}, //to ENET2_TX_CLK(CLKIN)
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

#define ENET_RXBD_NUM               (2)
#define ENET_TXBD_NUM               (2)
#define ENET_RXBUFF_SIZE            (ENET_FRAME_MAX_FRAMELEN)
#define ENET_TXBUFF_SIZE            (ENET_FRAME_MAX_FRAMELEN)

//_internal_rw uint32_t *s_rxBuffDescrip = RT_NULL;
//_internal_rw uint32_t *s_txBuffDescrip = RT_NULL;
//_internal_rw uint32_t *s_rxDataBuff = RT_NULL;
//_internal_rw uint32_t *s_txDataBuff = RT_NULL;
//_internal_rw uint8_t *s_macAddr = RT_NULL;

_internal_rw AT_NONCACHEABLE_SECTION_ALIGN(enet_rx_bd_struct_t s_rxBuffDescrip[ENET_RXBD_NUM], ENET_BUFF_ALIGNMENT);
_internal_rw AT_NONCACHEABLE_SECTION_ALIGN(enet_tx_bd_struct_t s_txBuffDescrip[ENET_TXBD_NUM], ENET_BUFF_ALIGNMENT);
_internal_rw SDK_ALIGN(uint8_t s_rxDataBuff[ENET_RXBD_NUM][SDK_SIZEALIGN(ENET_RXBUFF_SIZE, APP_ENET_BUFF_ALIGNMENT)], APP_ENET_BUFF_ALIGNMENT);
_internal_rw SDK_ALIGN(uint8_t s_txDataBuff[ENET_TXBD_NUM][SDK_SIZEALIGN(ENET_TXBUFF_SIZE, APP_ENET_BUFF_ALIGNMENT)], APP_ENET_BUFF_ALIGNMENT);

_internal_ro uint8_t k_src_mac_addr[6] = {0xd4, 0xbe, 0xd9, 0x45, 0x22, 0x60};
_internal_ro uint8_t k_tgt_mac_addr[6] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};

_internal_rw enet_handle_t s_handle;

static rt_err_t BOARD_InitModuleClock( void )
{
    _internal_ro clock_enet_pll_config_t config = {
        true,       /* enableClkOutput0 */
        true,       /* enableClkOutput1 */
        false,      /* enableClkOutput2 */
        1,          /* 0b01 = 50M, for ENET0 */
        1,          /* 0b01 = 50M, for ENET1 */
                    /* fixed 25M,  for ENET2 */
    };

    CLOCK_InitEnetPll(&config);
    CLOCK_EnableClock(kCLOCK_Enet);

    return RT_EOK;
}

static void enet_build_broadcast_frame( uint8_t *buf, uint16_t size )
{
    uint32_t count = 0;
    uint32_t length = size - 14;

    RT_ASSERT(size >= 14);

    rt_memcpy(&buf[0], k_tgt_mac_addr, 6U);
    rt_memcpy(&buf[6], k_src_mac_addr, 6U);

    buf[12] = (length >> 8) & 0xFFU;
    buf[13] = length & 0xFFU;

    for (count = 0; count < length; count++)
    {
        buf[count + 14] = count % 0xFFU;
    }
}

static void enet_get_mac_from_fuse(uint8_t *mac)
{
    uint8_t dummy[6];

    ENET_GetMacAddr(EXAMPLE_ENET, dummy);

    LOG_D("mac %02x:%02x:%02x:%02x:%02x:%02x", dummy[0], dummy[1], dummy[2],
                                               dummy[3], dummy[4], dummy[5]);

    if (mac) {
        rt_memcpy(mac, dummy, 6);
    }
}

static status_t enet_check_send_flag( ENET_Type *base, uint32_t timeout )
{
    status_t result;

    if (0 == timeout) {
        return kStatus_Success;
    }

    result = kStatus_Timeout;

    while (timeout--)
    {
        if (! (base->TDAR & ENET_TDAR_TDAR_MASK))
        {
            result = kStatus_Success;
            break;
        }
	}

    return result;
}

void _enet_callback( ENET_Type *base, enet_handle_t *handle, enet_event_t event, void *userData )
{
    LOG_W("_enet_callback");
}

static rt_err_t _lan8720_device_init( void )
{
    enet_config_t config;
    phy_speed_t speed;
    phy_duplex_t duplex;
    uint32_t sysClock;
    status_t result;

    enet_buffer_config_t buffConfig = {
        ENET_RXBD_NUM,
        ENET_TXBD_NUM,
        SDK_SIZEALIGN(ENET_RXBUFF_SIZE, APP_ENET_BUFF_ALIGNMENT),
        SDK_SIZEALIGN(ENET_TXBUFF_SIZE, APP_ENET_BUFF_ALIGNMENT),
        &s_rxBuffDescrip[0],
        &s_txBuffDescrip[0],
        &s_rxDataBuff[0][0],
        &s_txDataBuff[0][0],
        PV_OFFSET,
    };

    BOARD_InitModuleClock();

    ENET_GetDefaultConfig(&config);
    config.macSpecialConfig = kENET_ControlMacAddrInsert;

    sysClock = CORE_CLK_FREQ;
    LOG_D("sysclk is %d.", sysClock);

    result = PHY_Init(EXAMPLE_ENET, EXAMPLE_PHY, sysClock);
    LOG_D("PHY init %s.", (result?"failed":"success"));

    PHY_GetLinkSpeedDuplex(EXAMPLE_ENET, EXAMPLE_PHY, &speed, &duplex);
    config.miiSpeed = (enet_mii_speed_t)speed;
    config.miiDuplex = (enet_mii_duplex_t)duplex;

    LOG_D("PHY speed %dM, duplex %s.", (config.miiSpeed?100:10), (config.miiDuplex?"full":"half"));

//    enet_intcoalesce_config_t intConfig = {
//        {1,},
//        {64,},
//        {1,},
//        {64,}
//    };
//
//    config.intCoalesceCfg = &intConfig;
//    config.interrupt = kENET_TxFrameInterrupt
//                     | kENET_RxFrameInterrupt
//                     | kENET_TxBufferInterrupt;

    ENET_Init(EXAMPLE_ENET, &s_handle, &config, &buffConfig, (uint8_t*)k_src_mac_addr, sysClock);

    ENET_SetCallback(&s_handle, _enet_callback, RT_NULL);
    ENET_ActiveRead(EXAMPLE_ENET);

    rt_hw_us_delay(100000);

    enet_get_mac_from_fuse(RT_NULL);
    LOG_D("lan8720 init finished.");

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

static rt_size_t _lan8720_ops_write( rt_device_t dev, 
                                     rt_off_t pos, 
                                     const void *buffer, 
                                     rt_size_t size )
{
    status_t result;

    RT_ASSERT(RT_NULL != dev);

    result = ENET_SendFrame(EXAMPLE_ENET, &s_handle, buffer, size);
    if (kStatus_Success == result) {
        result = enet_check_send_flag(EXAMPLE_ENET, 5000);
    }

    LOG_D("PHY send %s.", (result?"failed":"success"));

    return (kStatus_Success == result) ? RT_EOK : -RT_ERROR;
}

#ifdef RT_USING_DEVICE_OPS
_internal_ro struct rt_device_ops _k_lan8720_ops =
{
    RT_NULL,                /* init */
    _lan8720_ops_open,      /* open */
    _lan8720_ops_close,     /* close */
    _lan8720_ops_read,      /* read */
    _lan8720_ops_write,     /* write */
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
    device->write   = _lan8720_ops_write;
    device->control = RT_NULL;
#endif

    device->user_data = RT_NULL;

    rt_device_register(device, _DEVICE_NAME, RT_DEVICE_FLAG_RDWR);

    _lan8720_gpio_init();
    _lan8720_device_init();

    return RT_EOK;
}
INIT_DEVICE_EXPORT(rt_hw_lan8720_init);

int fec_send(int argc, char **argv)
{
    uint8_t write_buf[128];
    uint16_t length = sizeof(write_buf);

    enet_build_broadcast_frame(write_buf, length);

    for (int i=0; i<length; i++)
    {
        LOG_RAW("%02x ", write_buf[i]);
        if (15 == (i%16)) {
            LOG_RAW("\r\n");
        }
    }

    _lan8720_ops_write((rt_device_t)2, 0, write_buf, length); //2 is noused!

    return 0;
}
MSH_CMD_EXPORT_ALIAS(fec_send, fec_send, <usr> enet fec send test);

int phy(int argc, char **argv)
{
    uint32_t data;

    LOG_D("show lan8720a register value:");

    for (int i=0; i<32; i++)
    {
        PHY_Read(EXAMPLE_ENET, EXAMPLE_PHY, i, &data);
        LOG_RAW("reg[%d]\t= %04x\r\n", i, data);
    }

    return 0;
}
MSH_CMD_EXPORT_ALIAS(phy, phy, <usr> enet phy test);

#endif //#ifdef RT_USING_LAN8720

