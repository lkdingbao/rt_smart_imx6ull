/*
 * IMX6ULL
 *   imx6ull nand driver file
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-04-27     Lyons        first version
 */

#include <rtconfig.h>
#include <rthw.h>
#include <rtdevice.h>

#ifdef BSP_USING_NAND

#include <board.h>
#ifdef RT_USING_LWP
#include <lwp.h>
#include <lwp_user_mm.h>
#endif

#include "__def.h"
#include "realview.h"
#include "drv_nand.h"
#include "skt.h"

#define DBG_TAG "drv.nand"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

#define NAND_FLASH_SHOW_INFO 0

#define NAND_ADDRESS_DATA GPMI_CTRL0_ADDRESS(0x0)
#define NAND_ADDRSSS_CLE GPMI_CTRL0_ADDRESS(0x1)
#define NAND_ADDRSSS_ALE GPMI_CTRL0_ADDRESS(0x2)

#define NAND_MODE_WR GPMI_CTRL0_COMMAND_MODE(0x0)
#define NAND_MODE_RD GPMI_CTRL0_COMMAND_MODE(0x1)
#define NAND_MODE_RDCMP GPMI_CTRL0_COMMAND_MODE(0x2)
#define NAND_MODE_WAIT GPMI_CTRL0_COMMAND_MODE(0x3)

_internal_rw struct skt_nanddev _s_nand = {
    .name = "nand",
    .periph[0].paddr = REALVIEW_GPMI_BASE,
    .periph[1].paddr = REALVIEW_BCH_BASE,
    .periph[2].paddr = REALVIEW_APBH_BASE,
    .irqno = RT_NULL, //not use irq yet!
    .gpio = {
        {IOMUXC_NAND_DATA00_RAWNAND_DATA00,     0U, 0xB0B1},
        {IOMUXC_NAND_DATA01_RAWNAND_DATA01,     0U, 0xB0B1},
        {IOMUXC_NAND_DATA02_RAWNAND_DATA02,     0U, 0xB0B1},
        {IOMUXC_NAND_DATA03_RAWNAND_DATA03,     0U, 0xB0B1},
        {IOMUXC_NAND_DATA04_RAWNAND_DATA04,     0U, 0xB0B1},
        {IOMUXC_NAND_DATA05_RAWNAND_DATA05,     0U, 0xB0B1},
        {IOMUXC_NAND_DATA06_RAWNAND_DATA06,     0U, 0xB0B1},
        {IOMUXC_NAND_DATA07_RAWNAND_DATA07,     0U, 0xB0B1},

        {IOMUXC_NAND_CLE_RAWNAND_CLE,           0U, 0xB0B1},
        {IOMUXC_NAND_ALE_RAWNAND_ALE,           0U, 0xB0B1},
        {IOMUXC_NAND_CE0_B_RAWNAND_CE0_B,       0U, 0xB0B1},
        {IOMUXC_NAND_CE1_B_RAWNAND_CE1_B,       0U, 0xB0B1},
        {IOMUXC_NAND_RE_B_RAWNAND_RE_B,         0U, 0xB0B1},
        {IOMUXC_NAND_WE_B_RAWNAND_WE_B,         0U, 0xB0B1},
        {IOMUXC_NAND_WP_B_RAWNAND_WP_B,         0U, 0xB0B1},
        {IOMUXC_NAND_READY_B_RAWNAND_READY_B,   0U, 0xB0B1},

        {IOMUXC_NAND_DQS_RAWNAND_DQS,           0U, 0xB0B1},
    },
    .flag = 0,
};

_internal_rw struct nand_attribute _s_nand_info;

#if defined(NAND_FLASH_SHOW_INFO) && (NAND_FLASH_SHOW_INFO)
static void _nand_show_info( struct nand_attribute *info )
{
    LOG_RAW("------------------------------\n");
    LOG_RAW("nand flash infomation\n");
    LOG_RAW("------------------------------\n");

    LOG_RAW("  id: %02X %02X %02X %02X %02X\n", info->id[0], info->id[1], info->id[2],
                                                info->id[3], info->id[4] );

    LOG_RAW("  total size: %d Bytes\n", info->total_size);

    LOG_RAW("  page size: %d Bytes\n", info->page_size);
    LOG_RAW("  page oob size: %d Bytes\n", info->page_oobsize);

    LOG_RAW("  block size: %d pages\n", info->block_size);

    LOG_RAW("  plane size: %d blocks\n", info->plane_size);
}
#endif //#if defined(NAND_FLASH_SHOW_INFO) && (NAND_FLASH_SHOW_INFO)

static rt_err_t nand_init_clock( struct skt_nanddev *nanddev )
{
    RT_ASSERT(RT_NULL != nanddev);

    /* config gpmi nand iomux */
    clrbits_le32( &CCM->CCGR4, 
        CCM_CCGR4_CG15_MASK | CCM_CCGR4_CG14_MASK |
        CCM_CCGR4_CG13_MASK | CCM_CCGR4_CG12_MASK |
        CCM_CCGR4_CG6_MASK );

    clrbits_le32( &CCM->CSCMR1,
        CCM_CSCMR1_GPMI_CLK_SEL_MASK | CCM_CSCMR1_BCH_CLK_SEL_MASK);

    clrsetbits_le32( &CCM->CSCDR1,
        CCM_CSCDR1_GPMI_PODF_MASK | CCM_CSCDR1_BCH_PODF_MASK,
        CCM_CSCDR1_GPMI_PODF(3) | CCM_CSCDR1_BCH_PODF(3) );

    clrbits_le32( &CCM->CCGR4,
        CCM_CCGR4_CG15(1) | CCM_CCGR4_CG14(1) |
        CCM_CCGR4_CG13(1) | CCM_CCGR4_CG12(1) |
        CCM_CCGR4_CG6(1) );

    CLOCK_EnableClock(kCLOCK_RawNandBch);
    CLOCK_EnableClock(kCLOCK_RawNandGpmi);
    CLOCK_EnableClock(kCLOCK_RawNandBchApb);
    CLOCK_EnableClock(kCLOCK_RawNandGpmiApb);
    CLOCK_EnableClock(kCLOCK_Apbhdma);

    return RT_EOK;
}

static int nand_init( struct skt_nanddev *nanddev )
{
    GPMI_Type *gpmi =RT_NULL;
    BCH_Type *bch =RT_NULL;
    APBH_Type *apbh =RT_NULL;
    uint32_t version;

    RT_ASSERT(RT_NULL != nanddev);
    RT_ASSERT(0 != nanddev->periph[0].vaddr);
    RT_ASSERT(0 != nanddev->periph[1].vaddr);
    RT_ASSERT(0 != nanddev->periph[2].vaddr);

    gpmi = (GPMI_Type*)(nanddev->periph[0].vaddr);
    bch = (BCH_Type*)(nanddev->periph[1].vaddr);
    apbh = (APBH_Type*)(nanddev->periph[2].vaddr);

    version = readl(&gpmi->VERSION);
    LOG_D("gpmi version: %d.%d.%d", (version&GPMI_VERSION_MAJOR_MASK) >> GPMI_VERSION_MAJOR_SHIFT,
                                    (version&GPMI_VERSION_MINOR_MASK) >> GPMI_VERSION_MINOR_SHIFT,
                                    (version&GPMI_VERSION_STEP_MASK) >> GPMI_VERSION_STEP_SHIFT );

    version = readl(&apbh->VERSION);
    LOG_D("apbh version: %d.%d.%d", (version&GPMI_VERSION_MAJOR_MASK) >> GPMI_VERSION_MAJOR_SHIFT,
                                    (version&GPMI_VERSION_MINOR_MASK) >> GPMI_VERSION_MINOR_SHIFT,
                                    (version&GPMI_VERSION_STEP_MASK) >> GPMI_VERSION_STEP_SHIFT );

    /* Reset the GPMI block. */
    mxs_reset_block((struct mxs_register_32*)&gpmi->CTRL0);
    mxs_reset_block((struct mxs_register_32*)&bch->CTRL);
    mxs_reset_block((struct mxs_register_32*)&apbh->CTRL0);

    /*
     * Choose NAND mode, set IRQ polarity, disable write protection and
     * select BCH ECC.
     */
    clrsetbits_le32( &gpmi->CTRL1,
        GPMI_CTRL1_GPMI_MODE_MASK,
        GPMI_CTRL1_ATA_IRQRDY_POLARITY(1) | GPMI_CTRL1_DEV_RESET(1) |
        GPMI_CTRL1_BCH_MODE(1) );

    return RT_EOK;
}

_internal_rw struct mxs_dma_desc nand_desc;

static void nand_cmd_ctrl( uint8_t data, int ctrl )
{
    APBH_Type *apbh = (APBH_Type*)platform_get_periph_vaddr(REALVIEW_APBH_BASE);
    uint8_t cmd_buf[16];
    int index;
    int ch = 0;

    index = 0;

    if (ctrl & (NAND_ALE | NAND_CLE))
    {
        if (NAND_CMD_NONE != data)
            cmd_buf[index++] = data;
    }

    if (0 == index)
        return;

    rt_memset(&nand_desc, 0, sizeof(struct mxs_dma_desc));

    nand_desc.cmd.next = 0;
    nand_desc.cmd.data =
        MXS_DMA_DESC_COMMAND_DMA_WRITE |
        MXS_DMA_DESC_CHAIN | MXS_DMA_DESC_DEC_SEM |
        MXS_DMA_DESC_WAIT4END | (3 << MXS_DMA_DESC_PIO_WORDS_OFFSET) |
        (index << MXS_DMA_DESC_BYTES_OFFSET);

    nand_desc.cmd.address = (unsigned long)&cmd_buf[0];

    nand_desc.cmd.pio_words[0] = 
        GPMI_CTRL0_WORD_LENGTH(1) |
        GPMI_CTRL0_CS(1<<ch) |
        GPMI_CTRL0_ADDRESS_INCREMENT(1) |
        NAND_MODE_WR |
        NAND_ADDRSSS_CLE |
        index;

    writel( mem_map_v2p((uint32_t)&nand_desc.cmd), &apbh->CH0_NXTCMDAR );
    writel( 1, &apbh->CH0_SEMA );
    writel( APBH_CTRL0_SET_CLKGATE_CHANNEL(1<<ch), &apbh->CTRL0_CLR );

    LOG_D("nxt cmd wr %08x", mem_map_v2p((uint32_t)&nand_desc.cmd));
    LOG_D("cur cmd rd %08x", readl(&apbh->CH0_CURCMDAR));
    LOG_D("nxt cmd rd %08x", readl(&apbh->CH0_NXTCMDAR));

//    if (mxs_wait_mask_set((struct mxs_register_32*)&apbh->CTRL1, 1<<ch, 10000000))
//    {
//        LOG_W("time out.");
//    }
}

static void nand_cmd( uint8_t command, int column, int page )
{
    nand_cmd_ctrl(command, NAND_CLE);

    if (-1 != column)
    {
        nand_cmd_ctrl(column, NAND_ALE);
        nand_cmd_ctrl(column >> 8, NAND_ALE);
    }
    if (-1 != page)
    {
        nand_cmd_ctrl(page, NAND_ALE);
        nand_cmd_ctrl(page >> 8, NAND_ALE);
        nand_cmd_ctrl(page >> 8, NAND_ALE);
    }

    nand_cmd_ctrl(NAND_CMD_NONE, 0);

    if (command == NAND_CMD_READ0)
    {
        nand_cmd_ctrl(NAND_CMD_READSTART, NAND_CLE);
        nand_cmd_ctrl(NAND_CMD_NONE, 0);
    }

    rt_hw_us_delay(1);
}

static void nand_read_buf( uint8_t *buf, int length )
{
    APBH_Type *apbh = (APBH_Type*)platform_get_periph_vaddr(APBH_BASE);
    int ch = 0;

    rt_memset(&nand_desc, 0, sizeof(struct mxs_dma_desc));

    nand_desc.cmd.next = 0;
    nand_desc.cmd.data =
        MXS_DMA_DESC_COMMAND_DMA_READ |
        MXS_DMA_DESC_CHAIN | MXS_DMA_DESC_DEC_SEM |
        MXS_DMA_DESC_WAIT4END | (3 << MXS_DMA_DESC_PIO_WORDS_OFFSET) |
        (length << MXS_DMA_DESC_BYTES_OFFSET);

    nand_desc.cmd.address = (unsigned long)buf;

    nand_desc.cmd.pio_words[0] = 
        GPMI_CTRL0_WORD_LENGTH(1) |
        GPMI_CTRL0_CS(1<<ch) |
        NAND_MODE_RD |
        NAND_ADDRESS_DATA |
        length;

    writel( mem_map_v2p((uint32_t)&nand_desc.cmd), &apbh->CH0_NXTCMDAR );
    writel( 1, &apbh->CH0_SEMA );
    writel( APBH_CTRL0_SET_CLKGATE_CHANNEL(1<<ch), &apbh->CTRL0_CLR );

    LOG_D("nxt cmd wr %08x", mem_map_v2p((uint32_t)&nand_desc.cmd));
    LOG_D("cur cmd rd %08x", readl(&apbh->CH0_CURCMDAR));
    LOG_D("nxt cmd rd %08x", readl(&apbh->CH0_NXTCMDAR));

//    if (mxs_wait_mask_set((struct mxs_register_32*)&apbh->CTRL1, 1<<ch, 10000000))
//    {
//        LOG_W("time out.");
//    }
}

static int nand_readid( struct skt_nanddev *nanddev,
                        struct nand_attribute *info )
{
    RT_ASSERT(RT_NULL != nanddev);

    nand_cmd(NAND_CMD_RESET, -1, -1);
    nand_cmd(NAND_CMD_READID, 0x00, -1);

    uint8_t vid = 0xFF;
    nand_read_buf(&vid, 1);

    LOG_D("vid = %02X", vid);

    return RT_EOK;
}

static void _nand_gpio_init( struct skt_nanddev *nanddev )
{
    RT_ASSERT(RT_NULL != nanddev);

    for (int i=0; i<NAND_CONTROL_PIN_NUM; i++)
    {
        gpio_set_iomux(&nanddev->gpio[i]);
    }
}

static rt_err_t _nand_device_init( struct skt_nanddev *nanddev )
{
    RT_ASSERT(RT_NULL != nanddev);

    nand_init_clock(nanddev);

    if (RT_EOK != nand_init(nanddev))
    {
        LOG_W("nand flash init failed.");
        return -RT_ERROR;
    }

    LOG_D("nand flash init.");

    nand_readid(nanddev, &_s_nand_info);

    _s_nand_info.total_size = 4 * 1024 * 1024 * 128; //128 is to bytes, equal to 1024 bits

    _s_nand_info.page_size = 2048;
    _s_nand_info.page_oobsize = 64;
    _s_nand_info.block_size = 64;
    _s_nand_info.plane_size = _s_nand_info.total_size / 
                             (_s_nand_info.block_size * _s_nand_info.page_size);

    nanddev->flag = 1;

    LOG_D("nand flash init finished.");

#if defined(NAND_FLASH_SHOW_INFO) && (NAND_FLASH_SHOW_INFO)
    _nand_show_info(&_s_nand_info);
#endif

    return RT_EOK;
}

static rt_err_t _nand_ops_init( rt_device_t dev )
{
    return RT_EOK;
}

static rt_err_t _nand_ops_open( rt_device_t dev,
                                rt_uint16_t oflag )
{
    RT_ASSERT(RT_NULL != dev);

    if (!(dev->flag & RT_DEVICE_FLAG_RDWR))
    {
        LOG_W("only support rd/wr option!");
        return -RT_ERROR;
    }

    dev->ref_count++;

    return RT_EOK;
}

static rt_err_t _nand_ops_close( rt_device_t dev )
{
    RT_ASSERT(RT_NULL != dev);

    dev->ref_count = (0 == dev->ref_count) ? 0 : (dev->ref_count - 1);

    return RT_EOK;
}

static rt_size_t _nand_ops_read( rt_device_t dev, 
                                 rt_off_t pos, 
                                 void *buffer, 
                                 rt_size_t size )
{
    RT_ASSERT(RT_NULL != dev);

    if (0 == dev->ref_count)
    {
        return 0;
    }

    return size;
}

static rt_size_t _nand_ops_write( rt_device_t dev, 
                                  rt_off_t pos, 
                                  const void *buffer, 
                                  rt_size_t size )
{
    RT_ASSERT(RT_NULL != dev);

    if (0 == dev->ref_count)
    {
        return 0;
    }

    return size;
}

static rt_err_t _nand_ops_control( rt_device_t dev, 
                                   int cmd,
                                   void *args )
{
    struct rt_device_blk_geometry *pgeometry = RT_NULL;
    rt_err_t result;

    switch (cmd)
    {
        case RT_DEVICE_CTRL_BLK_GETGEOME:
            if (args)
            {
                pgeometry = (struct rt_device_blk_geometry*)args;
                UNUSED(pgeometry);
                result = RT_EOK;
            }
            else
            {
                result = -RT_EINVAL;
            }
            break;

        case RT_DEVICE_CTRL_BLK_SYNC:
            result = RT_EOK;
            break;

        default :
            result = -RT_EINVAL;
            break;
    }

    return result;
}

#ifdef RT_USING_DEVICE_OPS
_internal_ro struct rt_device_ops _k_nand_ops =
{
    _nand_ops_init,     /* init */
    _nand_ops_open,     /* open */
    _nand_ops_close,    /* close */
    _nand_ops_read,     /* read */
    _nand_ops_write,    /* write */
    _nand_ops_control,  /* control */
};
#endif

int rt_hw_nand_init(void)
{
    struct rt_device *device = RT_NULL;

    device = &_s_nand.parent;

    device->type    = RT_Device_Class_Block;
#ifdef RT_USING_DEVICE_OPS
    device->ops     = &_k_nand_ops;
#else
    device->init    = _nand_ops_init;
    device->open    = _nand_ops_open;
    device->close   = _nand_ops_close;
    device->read    = _nand_ops_read;
    device->write   = _nand_ops_write;
    device->control = _nand_ops_control;
#endif
    device->user_data = RT_NULL;

    rt_device_register(device, _s_nand.name, RT_DEVICE_FLAG_RDWR);

    for (int i=0; i<GET_ARRAY_NUM(_s_nand.periph); i++) {
        _s_nand.periph[i].vaddr = platform_get_periph_vaddr(_s_nand.periph[i].paddr);
    }

    _s_nand.flag = 0; //wait for nand init!

    _nand_gpio_init(&_s_nand);
    _nand_device_init(&_s_nand);

    return RT_EOK;
}
INIT_DEVICE_EXPORT(rt_hw_nand_init);

#endif //#ifdef BSP_USING_NAND

