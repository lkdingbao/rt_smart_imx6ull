/*
 * IMX6ULL
 *   imx6ull nand driver file
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-04-27     Lyons        first version
 */

#ifndef __DRV_NAND_H__
#define __DRV_NAND_H__

#include "skt.h"

#ifdef BSP_USING_NAND

/*
 * Standard NAND flash commands
 */
#define NAND_CMD_READ0          0
#define NAND_CMD_READ1          1
#define NAND_CMD_RNDOUT         5
#define NAND_CMD_PAGEPROG       0x10
#define NAND_CMD_READOOB        0x50
#define NAND_CMD_ERASE1         0x60
#define NAND_CMD_STATUS         0x70
#define NAND_CMD_SEQIN          0x80
#define NAND_CMD_RNDIN          0x85
#define NAND_CMD_READID         0x90
#define NAND_CMD_ERASE2         0xd0
#define NAND_CMD_PARAM          0xec
#define NAND_CMD_GET_FEATURES   0xee
#define NAND_CMD_SET_FEATURES   0xef
#define NAND_CMD_RESET          0xff

#define NAND_CMD_LOCK           0x2a
#define NAND_CMD_UNLOCK1        0x23
#define NAND_CMD_UNLOCK2        0x24

/* Extended commands for large page devices */
#define NAND_CMD_READSTART      0x30
#define NAND_CMD_RNDOUTSTART    0xE0
#define NAND_CMD_CACHEDPROG     0x15

/* Extended commands for AG-AND device */
/*
 * Note: the command for NAND_CMD_DEPLETE1 is really 0x00 but
 *       there is no way to distinguish that from NAND_CMD_READ0
 *       until the remaining sequence of commands has been completed
 *       so add a high order bit and mask it off in the command.
 */
#define NAND_CMD_DEPLETE1       0x100
#define NAND_CMD_DEPLETE2       0x38
#define NAND_CMD_STATUS_MULTI   0x71
#define NAND_CMD_STATUS_ERROR   0x72
/* multi-bank error status (banks 0-3) */
#define NAND_CMD_STATUS_ERROR0  0x73
#define NAND_CMD_STATUS_ERROR1  0x74
#define NAND_CMD_STATUS_ERROR2  0x75
#define NAND_CMD_STATUS_ERROR3  0x76
#define NAND_CMD_STATUS_RESET   0x7f
#define NAND_CMD_STATUS_CLEAR   0xff

#define NAND_CMD_NONE           -1

/* Select the chip by setting nCE to low */
#define NAND_NCE                0x01
/* Select the command latch by setting CLE to high */
#define NAND_CLE                0x02
/* Select the address latch by setting ALE to high */
#define NAND_ALE                0x04

/* Fixed value, not edit! */
#define NAND_CONTROL_PIN_NUM        (17)

struct skt_nanddev
{
    struct rt_device                parent;

    const char                      *name;
    struct skt_periph               periph[3];
    rt_uint32_t                     irqno;

    struct skt_gpio                 gpio[NAND_CONTROL_PIN_NUM];

    rt_uint32_t                     flag;
};

struct nand_attribute
{
    uint8_t                         id[5];
    uint8_t                         reserved[3];

    uint32_t                        total_size; //bytes, not over(include) 4G Bytes 

    uint16_t                        page_size; //bytes
    uint16_t                        page_oobsize; //bytes

    uint16_t                        block_size; //pages
    uint16_t                        plane_size; //blocks

    uint8_t                         organization; //0:x8  1:x16
};

#endif //#ifdef BSP_USING_NAND
#endif //#ifndef __DRV_NAND_H__

