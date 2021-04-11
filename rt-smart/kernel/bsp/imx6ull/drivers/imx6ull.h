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
#include <lwp.h>

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
    return virt + PV_OFFSET;
}

rt_inline uint32_t mem_map_p2v(uint32_t phys)
{
    return phys - PV_OFFSET;
}

#define GIC_IRQ_START   0

#define GIC_ACK_INTID_MASK  0x000003ff

#endif //#ifndef __IMX6ULL_BOARD_H__

