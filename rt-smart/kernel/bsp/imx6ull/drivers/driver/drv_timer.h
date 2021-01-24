/*
 * IMX6ULL
 *   imx6ull arch timer driver file
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-01-12     Lyons        first version
 */

#ifndef __DRV_TIMER_H__
#define __DRV_TIMER_H__

#include <rtdef.h>
#include <realview.h>

#ifndef HW_SYS_COUNTER_CLOCK
#define HW_SYS_COUNTER_CLOCK    8000000
#endif

#ifndef __IO
#define __IO                    volatile
#endif

/** SCTR - Register Layout Typedef */
typedef struct{
    __IO uint32_t               cntcr;
    __IO uint32_t               cntsr;
    __IO uint32_t               cntcv1;
    __IO uint32_t               cntcv2;
         uint32_t               resv1[4];
    __IO uint32_t               cntfid0;
    __IO uint32_t               cntfid1;
    __IO uint32_t               cntfid2;
         uint32_t               resv2[1001];
    __IO uint32_t               counterid[1];
}SCTR_Type;

#define SCTR                    ((SCTR_Type *)REALVIEW_SCTL_BASE)

#define SC_CNTCR_ENABLE         (1 << 0)
#define SC_CNTCR_HDBG           (1 << 1)
#define SC_CNTCR_FREQ0          (1 << 8)
#define SC_CNTCR_FREQ1          (1 << 9)


#define isb()                   __asm__ __volatile__ ("" : : : "memory")
#define dsb()                   __asm__ __volatile__ ("" : : : "memory")
#define dmb()                   __asm__ __volatile__ ("" : : : "memory")

/** phys access */
rt_inline void arch_timer_reg_write_cp15_ctrl(rt_uint32_t val)
{
    asm volatile("mcr p15, 0, %0, c14, c2, 1" : : "r" (val));
    isb();
}

/** phys access */
rt_inline void arch_timer_reg_write_cp15_tval(rt_uint32_t val)
{
    asm volatile("mcr p15, 0, %0, c14, c2, 0" : : "r" (val));
    isb();
}

rt_inline void arch_timer_reg_write_cp15_cval(rt_uint64_t val)
{
    asm volatile("mcrr p15, 2, %Q0, %R0, c14" : : "r" (val));
    isb();
}

rt_inline void arch_timer_reg_write_cp15_cntfrq(rt_uint32_t val)
{
	asm volatile("mcr p15, 0, %0, c14, c0, 0" : : "r" (val));
    isb();
}

rt_inline rt_uint64_t arch_timer_reg_get_cp15_cval(void)
{
    rt_uint64_t val;
    asm volatile("mrrc p15, 2, %Q0, %R0, c14" : "=r" (val));
    return val;

}

rt_inline rt_uint32_t arch_timer_reg_get_cp15_cntfrq(void)
{
    rt_uint32_t val;
    asm volatile("mrc p15, 0, %0, c14, c0, 0" : "=r" (val));
    return val;
}

#endif //#ifndef __DRV_TIMER_H__

