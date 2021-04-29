/*
 * IMX6ULL
 *   imx6ull flexcan driver file
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-04-29     Lyons        first version
 */

#include <rtconfig.h>
#include <rthw.h>

#ifdef RT_USING_CAN

#include <board.h>
#ifdef RT_USING_LWP
#include <lwp.h>
#endif
 
#include "__def.h"
#include "realview.h"
#include "drv_can.h"
#include "skt.h"

#define DBG_TAG "drv.can"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

/* set this to 1 to enable can test demo. */
#define _FLEXCAN_DEBUG_EN       1

enum 
{
#ifdef RT_USING_FLEXCAN1
    eDevCan_CAN1,
#endif
#ifdef RT_USING_FLEXCAN2
    eDevCan_CAN2,
#endif

    eDevCan_Max, 
};

_internal_rw struct skt_can _s_can_bus[eDevCan_Max] = {
#ifdef RT_USING_FLEXCAN1
{
    .name = "can1",
    .periph.paddr = REALVIEW_CAN1_BASE,
    .irqno = CAN1_IRQn,
    .gpio = {
        {IOMUXC_UART3_CTS_B_FLEXCAN1_TX,     0, 0x1B0B0},
        {IOMUXC_UART3_RTS_B_FLEXCAN1_RX,     0, 0x1B0B0},
    },
    .rx_mb_id = 0x123,
    .tx_mb_idx = 8,
    .rx_mb_idx = 9,
    .flag = 0,
},
#endif

#ifdef RT_USING_FLEXCAN2
{
    .name = "can2",
    .periph.paddr = REALVIEW_CAN2_BASE,
    .irqno = CAN2_IRQn,
    .gpio = {
        {IOMUXC_UART2_CTS_B_FLEXCAN2_TX,    0, 0x1B0B0},
        {IOMUXC_UART2_RTS_B_FLEXCAN2_RX,    0, 0x1B0B0},
    },
    .rx_mb_id = 0x123,
    .tx_mb_idx = 8,
    .rx_mb_idx = 9,
    .flag = 0,
},
#endif
};

_internal_rw volatile bool _s_rx_complete = false;

static void _can_int_isr( int irqno, void* parameter )
{
    struct skt_can *device = RT_NULL;
    CAN_Type *periph = RT_NULL;

    RT_ASSERT(RT_NULL != parameter);

    device = (struct skt_can*)parameter;
    periph = (CAN_Type*)device->periph.vaddr;

    if (FLEXCAN_GetMbStatusFlags(periph, 1 << device->rx_mb_idx))
    {
        FLEXCAN_ClearMbStatusFlags(periph, 1 << device->rx_mb_idx);
        _s_rx_complete = true;
    }
}

static void _can_gpio_init( struct skt_can *device )
{
    RT_ASSERT(RT_NULL != device);

    for (int i=0; i<GET_ARRAY_NUM(device->gpio); i++)
    {
        gpio_set_iomux(&device->gpio[i]);
    }
}

static void _can_clock_init( struct skt_can *device )
{
    RT_ASSERT(RT_NULL != device);

    /* Set CAN source to OSC 24M */
    CLOCK_SetMux(kCLOCK_CanMux, 1U);
    CLOCK_SetDiv(kCLOCK_CanDiv, 0);

    if (REALVIEW_CAN1_BASE == device->periph.paddr)
    {
        CLOCK_EnableClock(kCLOCK_Can1);
    } else if (REALVIEW_CAN2_BASE == device->periph.paddr) {
        CLOCK_EnableClock(kCLOCK_Can2);
    }
}

static void _can_periph_init( struct skt_can *device, rt_bool_t loopBack )
{
    CAN_Type *periph = RT_NULL;
    flexcan_config_t config;
    flexcan_rx_mb_config_t mbconfig;

    RT_ASSERT(RT_NULL != device);

    periph = (CAN_Type*)device->periph.vaddr;

    FLEXCAN_GetDefaultConfig(&config);

#if (!defined(FSL_FEATURE_FLEXCAN_SUPPORT_ENGINE_CLK_SEL_REMOVE)) || !FSL_FEATURE_FLEXCAN_SUPPORT_ENGINE_CLK_SEL_REMOVE
    config.clkSrc = kFLEXCAN_ClkSrcPeri;
#endif
    config.enableLoopBack = loopBack;
    FLEXCAN_Init(periph, &config, CLOCK_GetOscFreq());

    FLEXCAN_SetRxMbGlobalMask(periph, FLEXCAN_RX_MB_STD_MASK(device->rx_mb_id, 0, 0));

    mbconfig.format = kFLEXCAN_FrameFormatStandard;
    mbconfig.type = kFLEXCAN_FrameTypeData;
    mbconfig.id = FLEXCAN_ID_STD(device->rx_mb_id);
    FLEXCAN_SetRxMbConfig(periph, device->rx_mb_idx, &mbconfig, true);

    FLEXCAN_SetTxMbConfig(periph, device->tx_mb_idx, true);

    FLEXCAN_EnableMbInterrupts(periph, 1 << device->rx_mb_idx);

    rt_hw_interrupt_install(device->irqno, _can_int_isr, device, device->name);
    rt_hw_interrupt_umask(device->irqno);
}

static rt_err_t _can_ops_open( rt_device_t dev,
                               rt_uint16_t oflag )
{
    RT_ASSERT(RT_NULL != dev);

    if (!(dev->flag & RT_DEVICE_FLAG_RDWR))
    {
        LOG_W("only support rd/wr option!");
        return -RT_ERROR;
    }

    return RT_EOK;
}

static rt_err_t _can_ops_close(rt_device_t dev)
{
    return RT_EOK;
}

static rt_size_t _can_ops_read( rt_device_t dev,
                                rt_off_t pos,
                                void *buffer,
                                rt_size_t size )
{
    struct skt_can *device = RT_NULL;
    CAN_Type *periph = RT_NULL;
    flexcan_frame_t rxframe;

    RT_ASSERT(RT_NULL != dev);
    RT_ASSERT(RT_NULL != buffer);

    device = (struct skt_can*)dev;
    periph = (CAN_Type*)device->periph.vaddr;

    while (!_s_rx_complete);
    _s_rx_complete = false;

    rxframe.length = 0;
    FLEXCAN_ReadRxMb(periph, device->rx_mb_idx, &rxframe);

    LOG_D("rx word0 = 0x%x", rxframe.dataWord0);
    LOG_D("rx word1 = 0x%x", rxframe.dataWord1);

    return rxframe.length;
}

static rt_size_t _can_ops_write( rt_device_t dev,
                                 rt_off_t pos,
                                 const void *buffer,
                                 rt_size_t size )
{
    struct skt_can *device = RT_NULL;
    struct rt_can_msg *msg = RT_NULL;
    CAN_Type *periph = RT_NULL;
    flexcan_frame_t txframe;

    RT_ASSERT(RT_NULL != dev);
    RT_ASSERT(RT_NULL != buffer);

    if (sizeof(struct rt_can_msg) != size)
    {
        LOG_D("can message size error!");
        return 0;
    }

    device = (struct skt_can*)dev;
    msg = (struct rt_can_msg*)buffer;

    periph = (CAN_Type*)device->periph.vaddr;

    txframe.id = FLEXCAN_ID_STD(msg->id);
    txframe.format = (RT_CAN_EXTID == msg->ide) ? kFLEXCAN_FrameFormatExtend : kFLEXCAN_FrameFormatStandard;
    txframe.type = (RT_CAN_RTR == msg->rtr) ? kFLEXCAN_FrameTypeRemote : kFLEXCAN_FrameTypeData;;
    txframe.length = msg->len;

    txframe.dataWord0 = CAN_WORD0_DATA_BYTE_0(msg->data[0])
                      | CAN_WORD0_DATA_BYTE_1(msg->data[1])
                      | CAN_WORD0_DATA_BYTE_2(msg->data[2])
                      | CAN_WORD0_DATA_BYTE_3(msg->data[3]);
    txframe.dataWord1 = CAN_WORD1_DATA_BYTE_4(msg->data[4])
                      | CAN_WORD1_DATA_BYTE_5(msg->data[5])
                      | CAN_WORD1_DATA_BYTE_6(msg->data[6])
                      | CAN_WORD1_DATA_BYTE_7(msg->data[7]);

    FLEXCAN_TransferSendBlocking(periph, device->tx_mb_idx, &txframe);

    return msg->len;
}

static rt_err_t _can_ops_control( rt_device_t dev,
                                  int cmd,
                                  void *args )
{
    struct skt_can *device = RT_NULL;
    rt_bool_t loopBack;

    RT_ASSERT(RT_NULL != dev);
    device = (struct skt_can*)dev;

    switch (cmd)
    {
        case RT_CAN_MODE_NORMAL:
            loopBack = false;
            break;

        case RT_CAN_MODE_LOOPBACK:
            loopBack = true;
            break;

        default:
            break;
    }

    if ( (RT_CAN_MODE_NORMAL == cmd) || (RT_CAN_MODE_LOOPBACK == cmd) ) {
        _can_periph_init(device, loopBack);
    }

    return RT_EOK;
}

#ifdef RT_USING_DEVICE_OPS
_internal_ro struct rt_device_ops _k_can_ops =
{
    RT_NULL,                /* init */
    _can_ops_open,          /* open */
    _can_ops_close,         /* close */
    _can_ops_read,          /* read */
    _can_ops_write,         /* write */
    _can_ops_control,       /* control */
};
#endif

int rt_hw_can_init(void)
{
    struct rt_device *device = RT_NULL;

    for (int i=0; i < GET_ARRAY_NUM(_s_can_bus); i++) 
    {
        device = &_s_can_bus[i].parent;

        device->type    = RT_Device_Class_CAN;
#ifdef RT_USING_DEVICE_OPS
        device->ops     = &_k_can_ops;
#else
        device->init    = RT_NULL;
        device->open    = _can_ops_open;
        device->close   = _can_ops_close;
        device->read    = _can_ops_read;
        device->write   = _can_ops_write;
        device->control = _can_ops_control;
#endif
        device->user_data = RT_NULL;

        _s_can_bus[i].periph.vaddr = platform_get_periph_vaddr(_s_can_bus[i].periph.paddr);
//        LOG_D("pddr %08x vaddr %08x", _s_can_bus[i].periph.paddr, _s_can_bus[i].periph.vaddr);

        _can_gpio_init(&_s_can_bus[i]);
        _can_clock_init(&_s_can_bus[i]);

        rt_device_register(device, _s_can_bus[i].name, RT_DEVICE_FLAG_RDWR);
    }

    return RT_EOK;
}
INIT_DEVICE_EXPORT(rt_hw_can_init);

#if defined(_FLEXCAN_DEBUG_EN) && (_FLEXCAN_DEBUG_EN)
int flexcan(int argc, char **argv)
{
    rt_device_t device = RT_NULL;
    struct rt_can_msg msg = {0};
    uint8_t dummy[8];

    device = rt_device_find("can1");
    if (RT_NULL == device)
    {
        LOG_RAW("not find device [%s].\n", "can1");
        return -1;
    }

    rt_device_open(device, RT_DEVICE_FLAG_RDWR);

    rt_device_control(device, RT_CAN_MODE_LOOPBACK, RT_NULL);

    msg.id  = 0x123;
    msg.ide = RT_CAN_STDID;
    msg.rtr = RT_CAN_DTR;
    msg.len = 8;

    msg.data[0] = 0x11;
    msg.data[1] = 0x22;
    msg.data[2] = 0x33;
    msg.data[3] = 0x44;
    msg.data[4] = 0x55;
    msg.data[5] = 0x66;
    msg.data[6] = 0x77;
    msg.data[7] = 0x88;

    rt_device_write(device, 0, &msg, sizeof(struct rt_can_msg));
    rt_device_read(device, 0, dummy, 8);

    rt_device_close(device);

    return 0;
}
MSH_CMD_EXPORT_ALIAS(flexcan, flexcan, <usr> can device test);
#endif //#if defined(_FLEXCAN_DEBUG_EN) && (_FLEXCAN_DEBUG_EN)

#endif //#ifdef RT_USING_CAN

