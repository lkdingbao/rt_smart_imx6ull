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

#define CAN_RX_MESSAGE_BUFFER_NUM (9)
#define CAN_TX_MESSAGE_BUFFER_NUM (8)

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
    .flag = 0,
},
#endif
};

_internal_rw volatile bool rxComplete = false;
_internal_rw flexcan_frame_t txFrame, rxFrame;

static void _can_int_isr( int irqno, void* parameter )
{
    struct skt_can *device = RT_NULL;
    CAN_Type *periph = RT_NULL;

    RT_ASSERT(RT_NULL != parameter);

    device = (struct skt_can*)parameter;
    periph = (CAN_Type*)device->periph.vaddr;

    if (FLEXCAN_GetMbStatusFlags(periph, 1 << CAN_RX_MESSAGE_BUFFER_NUM))
    {
        FLEXCAN_ClearMbStatusFlags(periph, 1 << CAN_RX_MESSAGE_BUFFER_NUM);
        FLEXCAN_ReadRxMb(periph, CAN_RX_MESSAGE_BUFFER_NUM, &rxFrame);
        rxComplete = true;
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

static void _can_periph_init( struct skt_can *device )
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
    config.enableLoopBack = true;
    FLEXCAN_Init(periph, &config, CLOCK_GetOscFreq());

    FLEXCAN_SetRxMbGlobalMask(periph, FLEXCAN_RX_MB_STD_MASK(0x123, 0, 0));

    mbconfig.format = kFLEXCAN_FrameFormatStandard;
    mbconfig.type = kFLEXCAN_FrameTypeData;
    mbconfig.id = FLEXCAN_ID_STD(0x123);
    FLEXCAN_SetRxMbConfig(periph, CAN_RX_MESSAGE_BUFFER_NUM, &mbconfig, true);

    FLEXCAN_SetTxMbConfig(periph, CAN_TX_MESSAGE_BUFFER_NUM, true);

    FLEXCAN_EnableMbInterrupts(periph, 1 << CAN_RX_MESSAGE_BUFFER_NUM);

    rt_hw_interrupt_install(device->irqno, _can_int_isr, device, device->name);
    rt_hw_interrupt_umask(device->irqno);

    txFrame.id = FLEXCAN_ID_STD(0x123);
    txFrame.format = kFLEXCAN_FrameFormatStandard;
    txFrame.type = kFLEXCAN_FrameTypeData;
    txFrame.length = CAN_TX_MESSAGE_BUFFER_NUM;
    txFrame.dataWord0 = CAN_WORD0_DATA_BYTE_0(0xde)
                      | CAN_WORD0_DATA_BYTE_1(0xad)
                      | CAN_WORD0_DATA_BYTE_2(0xbe)
                      | CAN_WORD0_DATA_BYTE_3(0xef);
    txFrame.dataWord1 = CAN_WORD1_DATA_BYTE_4(0x55)
                      | CAN_WORD1_DATA_BYTE_5(0x66)
                      | CAN_WORD1_DATA_BYTE_6(0x77)
                      | CAN_WORD1_DATA_BYTE_7(0x88);

    rxFrame.dataWord0 = 0;
    rxFrame.dataWord1 = 0;

    FLEXCAN_TransferSendBlocking(periph, CAN_TX_MESSAGE_BUFFER_NUM, &txFrame);

    while (!rxComplete);
    rxComplete = false;

    LOG_D("Receved message from MB%d", CAN_RX_MESSAGE_BUFFER_NUM);
    LOG_D("rx word0 = 0x%x", rxFrame.dataWord0);
    LOG_D("rx word1 = 0x%x", rxFrame.dataWord1);
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

static rt_err_t _can_ops_control( rt_device_t dev,
                                  int cmd,
                                  void *args )
{
    struct skt_can *device = RT_NULL;
    rt_err_t result = RT_EOK;;

    RT_ASSERT(RT_NULL != dev);
    device = (struct skt_can*)dev;

    switch (cmd)
    {
        default:
            result = -RT_ERROR;
            break;
    }

    UNUSED(device);
    return result;
}

#ifdef RT_USING_DEVICE_OPS
_internal_ro struct rt_device_ops _k_can_ops =
{
  RT_NULL,                    /* init */
  _can_ops_open,              /* open */
  _can_ops_close,             /* close */
  RT_NULL,                    /* read */
  RT_NULL,                    /* write */
  _can_ops_control,           /* control */
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
        device->read    = RT_NULL;
        device->write   = RT_NULL;
        device->control = _can_ops_control;
#endif
        device->user_data = RT_NULL;

        _s_can_bus[i].periph.vaddr = platform_get_periph_vaddr(_s_can_bus[i].periph.paddr);
        LOG_D("pddr %08x vaddr %08x", _s_can_bus[i].periph.paddr, _s_can_bus[i].periph.vaddr);

        _can_gpio_init(&_s_can_bus[i]);
        _can_clock_init(&_s_can_bus[i]);
        _can_periph_init(&_s_can_bus[i]);

        rt_device_register(device, _s_can_bus[i].name, RT_DEVICE_FLAG_RDWR);
    }

    return RT_EOK;
}
INIT_DEVICE_EXPORT(rt_hw_can_init);

#endif //#ifdef RT_USING_CAN

