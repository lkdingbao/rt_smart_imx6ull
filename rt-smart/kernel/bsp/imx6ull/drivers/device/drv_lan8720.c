/*
 * LAN8720A
 *   LAN8720A driver file
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-01-30     Lyons        first version
 */

#include <rtconfig.h>
#include <rthw.h>
#include <rtdevice.h>

#ifdef RT_USING_LAN8720

#include <board.h>
#ifdef RT_USING_LWP
#include <lwp.h>
#include <lwp_user_mm.h>
#endif

#include "__def.h"
#include "realview.h"
#include "drv_pin.h"
#include "drv_lan8720.h"
#include "skt.h"

#define DBG_TAG "LAN8720"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

#define ENET_PHY_MONITOR_EN (1)
#define ENET_MEM_BY_USER_EN (1)

#if defined(ENET_MEM_BY_USER_EN) && (ENET_MEM_BY_USER_EN)
/* This is only a test method. not used in formal occasion. */
_internal_rw enet_rx_bd_struct_t *_s_rxBuffDescrip;
_internal_rw enet_tx_bd_struct_t *_s_txBuffDescrip;
_internal_rw uint8_t *_s_rxDataBuff;
_internal_rw uint8_t *_s_txDataBuff;
#else
_internal_rw AT_NONCACHEABLE_SECTION_ALIGN(enet_rx_bd_struct_t _s_rxBuffDescrip[ENET_RXBD_NUM], ENET_BUFF_ALIGNMENT);
_internal_rw AT_NONCACHEABLE_SECTION_ALIGN(enet_tx_bd_struct_t _s_txBuffDescrip[ENET_TXBD_NUM], ENET_BUFF_ALIGNMENT);
_internal_rw SDK_ALIGN(uint8_t _s_rxDataBuff[ENET_RXBD_NUM][SDK_SIZEALIGN(ENET_RXBUFF_SIZE, APP_ENET_BUFF_ALIGNMENT)], APP_ENET_BUFF_ALIGNMENT);
_internal_rw SDK_ALIGN(uint8_t _s_txDataBuff[ENET_TXBD_NUM][SDK_SIZEALIGN(ENET_TXBUFF_SIZE, APP_ENET_BUFF_ALIGNMENT)], APP_ENET_BUFF_ALIGNMENT);
#endif

_internal_rw struct skt_netdev _s_lan8720_device = {
    .name = "enet2",
    .periph.paddr = REALVIEW_ENET2_BASE,
    .irqno = ENET2_IRQn,
    .gpio = {
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
    },
    .rst_pin = GET_PIN(4,8),
    .phy_addr = 0x1,
    .mac_addr = {0xd4, 0xbe, 0xd9, 0x45, 0x22, 0x60},
    .flag = 0, //no used!
};

static rt_err_t enet_init_clock( void )
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
    CLOCK_EnableClock(kCLOCK_Enet);

    return RT_EOK;
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

static void _enet_rx_callback( struct skt_netdev *netdev )
{
    ENET_Type *enet = RT_NULL;
    rt_err_t result;

    RT_ASSERT(RT_NULL != netdev);

    enet = (ENET_Type*)(netdev->periph.vaddr);

    ENET_DisableInterrupts(enet, kENET_RxFrameInterrupt);

    result = eth_device_ready(&(netdev->parent));
    if (RT_EOK != result)
    {
        LOG_W("eth ready err = %d", result);
    }
}

static void _enet_tx_callback( struct skt_netdev *netdev )
{
    if (RT_TRUE == netdev->tx_wait_flag)
    {
        netdev->tx_wait_flag = RT_FALSE;
        rt_sem_release(&netdev->tx_wait);
    }
}

static void _enet_int_isr( int irqno, void* parameter )
{
    struct skt_netdev *netdev = RT_NULL;
    ENET_Type *enet = RT_NULL;
    uint32_t flag;

    RT_ASSERT(RT_NULL != parameter);

    netdev = (struct skt_netdev*)(parameter);
    enet = (ENET_Type*)(netdev->periph.vaddr);

    flag = enet->EIR;

    if (flag & ENET_RX_INTERRUPT)
    {
        _enet_rx_callback(netdev);
        enet->EIR = ENET_RX_INTERRUPT;
    }

    if (flag & ENET_TX_INTERRUPT)
    {
        _enet_tx_callback(netdev);
        enet->EIR = ENET_TX_INTERRUPT;
    }
}

static struct pbuf* _imx_enet_rx( rt_device_t dev )
{
    struct skt_netdev *netdev = RT_NULL;
    ENET_Type *enet = RT_NULL;
    struct pbuf *pbuf = RT_NULL;
    enet_data_error_stats_t eErrStatic;
    uint32_t rxlen = 0;
    status_t result;

    RT_ASSERT(RT_NULL != dev);

    netdev = (struct skt_netdev*)(dev);
    enet = (ENET_Type*)(netdev->periph.vaddr);

    result = ENET_GetRxFrameSize(&netdev->handle, &rxlen);

    if (0 != rxlen)
    {
        pbuf = pbuf_alloc(PBUF_RAW, rxlen, PBUF_POOL);
        if (pbuf)
        {
            result = ENET_ReadFrame(enet, &netdev->handle, pbuf->payload, rxlen);
            if (kStatus_Success == result)
            {
                return pbuf;
            } else {
                LOG_D("frame read failed!");
                pbuf_free(pbuf);
            }
        }
    }
    else if (kStatus_ENET_RxFrameError == result)
    {
        ENET_GetRxErrBeforeReadFrame(&netdev->handle, &eErrStatic);
        ENET_ReadFrame(enet, &netdev->handle, RT_NULL, 0);
    }

    ENET_EnableInterrupts(enet, kENET_RxFrameInterrupt);

    UNUSED(eErrStatic);
    return RT_NULL;
}

static rt_err_t _imx_enet_tx( rt_device_t dev, struct pbuf* p )
{
    struct skt_netdev *netdev = RT_NULL;
    ENET_Type *enet = RT_NULL;
    status_t result;

    RT_ASSERT(RT_NULL != dev);
    RT_ASSERT(RT_NULL != p);

    netdev = (struct skt_netdev*)(dev);
    enet = (ENET_Type*)(netdev->periph.vaddr);

    do
    {
        result = ENET_SendFrame(enet, &netdev->handle, (const uint8_t*)p, p->tot_len);

        if (kStatus_ENET_TxFrameBusy == result)
        {
            netdev->tx_wait_flag = RT_FALSE;
            rt_sem_take(&netdev->tx_wait, RT_WAITING_FOREVER);
        }
    } while (kStatus_ENET_TxFrameBusy == result);

    UNUSED(result);
    return RT_EOK;
}

static void _lan8720_gpio_init( struct skt_netdev *netdev )
{
    gpio_pin_config_t config;
    rt_uint32_t reg_value;

    RT_ASSERT(RT_NULL != netdev);

    for (int i=0; i<ENET_CONTROL_PIN_NUM; i++)
    {
        gpio_set_iomux(&netdev->gpio[i]);
    }

    IOMUXC_GPR_Type *_IOMUXC_GPR = (IOMUXC_GPR_Type*)platform_get_periph_vaddr(REALVIEW_IOMUXC_GPR_BASE);

    reg_value = _IOMUXC_GPR->GPR1;
    if      (REALVIEW_ENET1_BASE == netdev->periph.paddr)
    {
        reg_value &= ~(IOMUXC_GPR_GPR1_ENET1_CLK_SEL_MASK 
                     | IOMUXC_GPR_GPR1_ENET1_CLK_SEL_MASK);
        reg_value |=  IOMUXC_GPR_GPR1_ENET1_TX_CLK_DIR(1);
        reg_value |=  IOMUXC_GPR_GPR1_ENET1_CLK_SEL(0);
    }
    else if (REALVIEW_ENET2_BASE == netdev->periph.paddr)
    {
        reg_value &= ~(IOMUXC_GPR_GPR1_ENET2_CLK_SEL_MASK 
                     | IOMUXC_GPR_GPR1_ENET2_CLK_SEL_MASK);
        reg_value |=  IOMUXC_GPR_GPR1_ENET2_TX_CLK_DIR(1);
        reg_value |=  IOMUXC_GPR_GPR1_ENET2_CLK_SEL(0);
    }
    _IOMUXC_GPR->GPR1 = reg_value;

    config.direction = kGPIO_DigitalOutput;
    config.interruptMode = kGPIO_NoIntmode;
    config.outputLogic = PIN_LOW;
    gpio_config(netdev->rst_pin, &config);

    gpio_output(netdev->rst_pin, 0);
    rt_hw_ms_delay(20);
    gpio_output(netdev->rst_pin, 1);
}

static rt_err_t _lan8720_device_init( struct skt_netdev *netdev )
{
    enet_config_t config;
    phy_speed_t speed;
    phy_duplex_t duplex;
    uint32_t sysClock;
    status_t result;
    ENET_Type *enet = RT_NULL;
    bool link;

#if defined(ENET_MEM_BY_USER_EN) && (ENET_MEM_BY_USER_EN)
    /* This is only a test method. not used in formal occasion. */
    _s_rxBuffDescrip = (enet_rx_bd_struct_t*)(KERNEL_VADDR_START + 0x0fc00000 + 0x0); //2K Byte, offset 0x0 //defined at board.c!
    _s_txBuffDescrip = (enet_tx_bd_struct_t*)(KERNEL_VADDR_START + 0x0fc00000 + 0x800); //2K Byte, offset 0x800
    _s_rxDataBuff = (uint8_t*)(KERNEL_VADDR_START + 0x0fc00000 + 0x100000); //1M Byte, offset 0x100000
    _s_txDataBuff = (uint8_t*)(KERNEL_VADDR_START + 0x0fc00000 + 0x200000); //1M Byte, offset 0x200000
#endif

    enet_buffer_config_t buffConfig = {
        ENET_RXBD_NUM,
        ENET_TXBD_NUM,
        SDK_SIZEALIGN(ENET_RXBUFF_SIZE, APP_ENET_BUFF_ALIGNMENT),
        SDK_SIZEALIGN(ENET_TXBUFF_SIZE, APP_ENET_BUFF_ALIGNMENT),
        (volatile enet_rx_bd_struct_t*)_s_rxBuffDescrip,
        (volatile enet_tx_bd_struct_t*)_s_txBuffDescrip,
        (uint8_t*)_s_rxDataBuff,
        (uint8_t*)_s_txDataBuff,
    };

    RT_ASSERT(RT_NULL != netdev);

    enet = (ENET_Type*)netdev->periph.vaddr;

    enet_init_clock();

    ENET_GetDefaultConfig(&config);
    config.macSpecialConfig = kENET_ControlMacAddrInsert;

    sysClock = CLOCK_GetFreq(kCLOCK_AhbClk);
//    LOG_D("sysclk is %d.", sysClock);

    result = PHY_Init(enet, netdev->phy_addr, sysClock);
    LOG_D("PHY init %s.", (result?"failed":"success"));

    result = PHY_GetLinkStatus(enet, netdev->phy_addr, &link);
    if (kStatus_Success == result)
    {
        PHY_GetLinkSpeedDuplex(enet, netdev->phy_addr, &speed, &duplex);
        config.miiSpeed = (enet_mii_speed_t)speed;
        config.miiDuplex = (enet_mii_duplex_t)duplex;

        LOG_D("PHY speed %dM, duplex %s.", (config.miiSpeed?100:10), (config.miiDuplex?"full":"half"));

        netdev->link_up_flag = true;
        netdev->link_up_status = ((uint8_t)config.miiSpeed << 4) | ((uint8_t)config.miiDuplex);;

        eth_device_linkchange(&_s_lan8720_device.parent, link);
    } else {
        LOG_W("PHY commuicate failed!");
    }

    config.interrupt = ENET_TX_INTERRUPT
                     | ENET_RX_INTERRUPT;

    rt_hw_interrupt_install(netdev->irqno, _enet_int_isr, netdev, netdev->name);
    rt_hw_interrupt_umask(netdev->irqno);

    ENET_Init(enet, &netdev->handle, &config, &buffConfig, netdev->mac_addr, sysClock);
    ENET_ActiveRead(enet);

    LOG_D("lan8720 init finished.");

    UNUSED(result);
    return RT_EOK;
}

static rt_err_t _lan8720_ops_init( rt_device_t dev )
{
    struct skt_netdev *netdev = RT_NULL;

    RT_ASSERT(RT_NULL != dev);

    netdev = (struct skt_netdev*)(dev);

    _lan8720_gpio_init(netdev);
    _lan8720_device_init(netdev);

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
    struct skt_netdev *netdev = RT_NULL;
    ENET_Type *enet = RT_NULL;
    status_t result;

    RT_ASSERT(RT_NULL != dev);

    netdev = (struct skt_netdev*)(dev);
    enet = (ENET_Type*)(netdev->periph.vaddr);

    result = ENET_SendFrame(enet, &netdev->handle, buffer, size);
    if (kStatus_Success == result) {
        result = enet_check_send_flag(enet, 5000);
    }

    LOG_D("PHY send %s.", (result?"failed":"success"));

    return (kStatus_Success == result) ? RT_EOK : -RT_ERROR;
}

static rt_err_t _lan8720_ops_control( rt_device_t dev, 
                                      int cmd,
                                      void *args )
{
    struct skt_netdev *netdev = RT_NULL;

    RT_ASSERT(RT_NULL != dev);

    netdev = (struct skt_netdev*)(dev);

    switch (cmd)
    {
        case NIOCTL_GADDR:
            if (args) {
                rt_memcpy(args, netdev->mac_addr, 6);
            } else {
                return -RT_ERROR;
            }
            break;

        default :
            break;
    }

    return RT_EOK;
}

#ifdef RT_USING_DEVICE_OPS
_internal_ro struct rt_device_ops _k_enet_ops =
{
    _lan8720_ops_init,      /* init */
    _lan8720_ops_open,      /* open */
    _lan8720_ops_close,     /* close */
    _lan8720_ops_read,      /* read */
    _lan8720_ops_write,     /* write */
    _lan8720_ops_control,   /* control */
};
#endif

#if defined(ENET_PHY_MONITOR_EN) && (ENET_PHY_MONITOR_EN)
static void enet_get_phy_link_state( void )
{
    ENET_Type *enet = RT_NULL;
    uint8_t phyAddr;
    status_t result;
    phy_speed_t speed;
    phy_duplex_t duplex;
    bool linkUpFlag;
    uint8_t linkUpStatus;

    enet = (ENET_Type*)(_s_lan8720_device.periph.vaddr);
    phyAddr = _s_lan8720_device.phy_addr;

    result = PHY_GetLinkStatus(enet, phyAddr, &linkUpFlag);
    if (kStatus_Success == result)
    {
        _s_lan8720_device.link_up_flag = linkUpFlag;

        PHY_GetLinkSpeedDuplex(enet, phyAddr, &speed, &duplex);
        linkUpStatus = ((uint8_t)speed << 4) | ((uint8_t)duplex);

        if (linkUpStatus != _s_lan8720_device.link_up_status)
        {
            if (true == _s_lan8720_device.link_up_flag)
            {
                LOG_D("link on.");
                _lan8720_device_init(&_s_lan8720_device);
                eth_device_linkchange(&_s_lan8720_device.parent, RT_TRUE);
            } else {
                LOG_D("link down.");
                eth_device_linkchange(&_s_lan8720_device.parent, RT_FALSE);
            }
        }

        _s_lan8720_device.link_up_status = linkUpStatus;
    }
}

static void phy_monitor_thread_entry( void *param )
{
    while (1)
    {
        enet_get_phy_link_state();
        rt_thread_mdelay(5000);
    }
}
#endif //#if defined(ENET_PHY_MONITOR_EN) && (ENET_PHY_MONITOR_EN)

int rt_hw_lan8720_init(void)
{   
    struct eth_device *ethdev = RT_NULL;
#if defined(ENET_PHY_MONITOR_EN) && (ENET_PHY_MONITOR_EN)
    rt_thread_t tid;
#endif

    ethdev = &(_s_lan8720_device.parent);

#ifdef RT_USING_DEVICE_OPS
    ethdev->parent.ops     = &_k_enet_ops;
#else
    ethdev->parent.init    = _lan8720_ops_init;
    ethdev->parent.open    = _lan8720_ops_open;
    ethdev->parent.close   = _lan8720_ops_close;
    ethdev->parent.read    = _lan8720_ops_read;
    ethdev->parent.write   = _lan8720_ops_write;
    ethdev->parent.control = _lan8720_ops_control;
#endif
    ethdev->parent.user_data = RT_NULL;

    ethdev->eth_tx = _imx_enet_tx;
    ethdev->eth_rx = _imx_enet_rx;

    _s_lan8720_device.link_up_flag = false;
    _s_lan8720_device.link_up_status = ~0;

    /* semaphore must be init before enet. */
    rt_sem_init(&_s_lan8720_device.tx_wait, "tx_wait", 0, RT_IPC_FLAG_FIFO);
    _s_lan8720_device.tx_wait_flag = RT_FALSE;

    _s_lan8720_device.periph.vaddr = platform_get_periph_vaddr(_s_lan8720_device.periph.paddr);

    eth_device_init(ethdev, "e0");

#if defined(ENET_PHY_MONITOR_EN) && (ENET_PHY_MONITOR_EN)
    /* start phy monitor thread in the end. */
    tid = rt_thread_create( "phy_mntr",
                            phy_monitor_thread_entry,
                            RT_NULL,
                            512,
                            RT_THREAD_PRIORITY_MAX - 2,
                            2 );

    if (RT_NULL != tid)
    {
        rt_thread_startup(tid);
    } else {
        LOG_E("phy monitor start failed!");
    }
#endif //#if defined(ENET_PHY_MONITOR_EN) && (ENET_PHY_MONITOR_EN)

    return RT_EOK;
}
INIT_DEVICE_EXPORT(rt_hw_lan8720_init);

#ifndef RT_USING_LWIP
static void enet_build_broadcast_frame( struct skt_netdev *netdev, uint8_t *buf, uint16_t size )
{
    uint32_t count, index;
    uint32_t length = size - 14;
    uint8_t tgt_mac_addr[ENET_MAC_ADDR_LENGTH] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};

    RT_ASSERT(RT_NULL != netdev);
    RT_ASSERT(size >= 14);

    rt_memcpy(&buf[0], tgt_mac_addr, ENET_MAC_ADDR_LENGTH);
    rt_memcpy(&buf[ENET_MAC_ADDR_LENGTH], netdev->mac_addr, ENET_MAC_ADDR_LENGTH);

    index = ENET_MAC_ADDR_LENGTH * 2;
    buf[index++] = (length >> 8) & 0xFFU;
    buf[index++] = length & 0xFFU;

    for (count = 0; count < length; count++)
    {
        buf[index + count] = count % 0xFFU;
    }
}

int fec_send(int argc, char **argv)
{
    uint8_t write_buf[128];
    uint16_t length = sizeof(write_buf);

    enet_build_broadcast_frame(&_s_lan8720_device, write_buf, length);

    for (int i=0; i<length; i++)
    {
        LOG_RAW("%02x ", write_buf[i]);
        if (15 == (i%16)) {
            LOG_RAW("\r\n");
        }
    }

    _lan8720_ops_write((rt_device_t)&_s_lan8720_device, 0, write_buf, length);

    return 0;
}
MSH_CMD_EXPORT_ALIAS(fec_send, fec_send, <usr> enet fec send test);
#endif //#ifndef RT_USING_LWIP

int phy(int argc, char **argv)
{
    ENET_Type *enet = (ENET_Type*)_s_lan8720_device.periph.vaddr;
    uint32_t data;

    LOG_D("show lan8720a register value:");

    for (int i=0; i<32; i++)
    {
        PHY_Read(enet, _s_lan8720_device.phy_addr, i, &data);
        LOG_RAW("reg[%d]\t= %04x\r\n", i, data);
    }

    return 0;
}
//MSH_CMD_EXPORT_ALIAS(phy, phy, <usr> enet phy test);

#endif //#ifdef RT_USING_LAN8720
