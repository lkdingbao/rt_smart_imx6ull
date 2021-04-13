/*
 * IMX6ULL
 *   imx6ull board file
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-01-13     Lyons        first version
 */


#ifndef __IMX6ULL_BOARD_H__
#define __IMX6ULL_BOARD_H__

/* for 'rt_inline' */
#include <rtdef.h>
/* for 'rt_hw_kernel_phys_to_virt' */
#ifdef RT_USING_LWP
#include <lwp.h>
#endif

/* SOC-relative definitions */
#include <realview.h>

#include "bsp_clock.h"
#include "bsp_gpio.h"
#include "drv_timer.h"

/* the maximum entries of the exception table */
#define MAX_HANDLERS    ARM_GIC_NR_IRQS

/* the basic constants and interfaces needed by gic */
rt_inline rt_uint32_t platform_get_gic_dist_base(void)
{
    return REALVIEW_GIC_DIST_BASE;
}

rt_inline rt_uint32_t platform_get_gic_cpu_base(void)
{
    return REALVIEW_GIC_CPU_BASE;
}

/* get peripg virtual address, alignment at REALVIEW_PERIPH_SIZEs */
rt_inline rt_uint32_t platform_get_periph_vaddr(rt_uint32_t paddr)
{
#ifdef RT_USING_USERSPACE
    rt_uint32_t mask = REALVIEW_PERIPH_SIZE - 1;
    return (rt_uint32_t)rt_hw_kernel_phys_to_virt((void*)(paddr&(~mask)), REALVIEW_PERIPH_SIZE) + (paddr & mask);
#else
    return paddr;
#endif
}

rt_inline uint32_t mem_map_v2p(uint32_t virt)
{
#ifdef RT_USING_USERSPACE
    return virt + PV_OFFSET;
#else
    return virt;
#endif
}

rt_inline uint32_t mem_map_p2v(uint32_t phys)
{
#ifdef RT_USING_USERSPACE
    return phys - PV_OFFSET;
#else
    return phys;
#endif
}

#define GIC_IRQ_START   0

#define GIC_ACK_INTID_MASK  0x000003ff

#define readb(reg)		(*((volatile unsigned char *) (reg)))
#define readw(reg)		(*((volatile unsigned short *) (reg)))
#define readl(reg)		(*((volatile unsigned int *) (reg)))

#define writeb(data, reg)	((*((volatile unsigned char *)(reg))) = (unsigned char)(data))
#define writew(data, reg)	((*((volatile unsigned short *)(reg))) = (unsigned short)(data))
#define writel(data, reg)	((*((volatile unsigned int *)(reg))) = (unsigned int)(data))

#endif //#ifndef __IMX6ULL_BOARD_H__

