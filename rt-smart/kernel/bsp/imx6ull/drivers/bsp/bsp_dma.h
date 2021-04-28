/*
 * IMX6ULL
 *   imx6ull dma bsp file
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-04-27     Lyons        first version
 */

#ifndef __BSP_DMA_H__
#define __BSP_DMA_H__

#include <rtconfig.h>
#include <rthw.h>

#define	DMA_PIO_WORDS		15

/*
 * MXS DMA channels
 */
enum {
	MXS_DMA_CHANNEL_AHB_APBH_GPMI0 = 0,
	MXS_DMA_CHANNEL_AHB_APBH_GPMI1,
	MXS_DMA_CHANNEL_AHB_APBH_GPMI2,
	MXS_DMA_CHANNEL_AHB_APBH_GPMI3,
	MXS_DMA_CHANNEL_AHB_APBH_GPMI4,
	MXS_DMA_CHANNEL_AHB_APBH_GPMI5,
	MXS_DMA_CHANNEL_AHB_APBH_GPMI6,
	MXS_DMA_CHANNEL_AHB_APBH_GPMI7,
	MXS_MAX_DMA_CHANNELS,
};

/*
 * MXS DMA hardware command.
 *
 * This structure describes the in-memory layout of an entire DMA command,
 * including space for the maximum number of PIO accesses. See the appropriate
 * reference manual for a detailed description of what these fields mean to the
 * DMA hardware.
 */
#define	MXS_DMA_DESC_COMMAND_MASK	0x3
#define	MXS_DMA_DESC_COMMAND_OFFSET	0
#define	MXS_DMA_DESC_COMMAND_NO_DMAXFER	0x0
#define	MXS_DMA_DESC_COMMAND_DMA_WRITE	0x1
#define	MXS_DMA_DESC_COMMAND_DMA_READ	0x2
#define	MXS_DMA_DESC_COMMAND_DMA_SENSE	0x3
#define	MXS_DMA_DESC_CHAIN		(1 << 2)
#define	MXS_DMA_DESC_IRQ		(1 << 3)
#define	MXS_DMA_DESC_NAND_LOCK		(1 << 4)
#define	MXS_DMA_DESC_NAND_WAIT_4_READY	(1 << 5)
#define	MXS_DMA_DESC_DEC_SEM		(1 << 6)
#define	MXS_DMA_DESC_WAIT4END		(1 << 7)
#define	MXS_DMA_DESC_HALT_ON_TERMINATE	(1 << 8)
#define	MXS_DMA_DESC_TERMINATE_FLUSH	(1 << 9)
#define	MXS_DMA_DESC_PIO_WORDS_MASK	(0xf << 12)
#define	MXS_DMA_DESC_PIO_WORDS_OFFSET	12
#define	MXS_DMA_DESC_BYTES_MASK		(0xffff << 16)
#define	MXS_DMA_DESC_BYTES_OFFSET	16

struct mxs_dma_cmd {
	unsigned long		next;
	unsigned long		data;
	union {
		unsigned long	address;
		unsigned long	alternate;
	};
	unsigned long		pio_words[DMA_PIO_WORDS];
};

/*
 * MXS DMA command descriptor.
 *
 * This structure incorporates an MXS DMA hardware command structure, along
 * with metadata.
 */
#define	MXS_DMA_DESC_FIRST	(1 << 0)
#define	MXS_DMA_DESC_LAST	(1 << 1)
#define	MXS_DMA_DESC_READY	(1 << 31)

struct list_head {
	struct list_head *next, *prev;
};

struct mxs_dma_desc {
	struct mxs_dma_cmd	cmd;
	unsigned int		flags;
	unsigned long		address;
	void			*buffer;
	struct list_head	node;
};

#ifndef offsetof
#define offsetof(s,m) ((size_t)&(((s*)0)->m))
#endif

#endif	//#ifndef __BSP_DMA_H__

