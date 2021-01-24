/*
 * IMX6ULL
 *   imx6ull arch timer driver file
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-01-12     Lyons        first version
 */

#include <rtconfig.h>
#include <rthw.h>
 
#include <board.h>
#include <lwp.h>
 
#include "__def.h"
#include "realview.h"
#include "drv_timer.h"

#define SYS_COUNTER_TICK_PERIOD     (rt_uint32_t)(_s_system_freq / RT_TICK_PER_SECOND)

#define IRQ_SYS_COUNTER             SecurePhyTimer_IRQn
//SecurePhyTimer_IRQn
//NonSecurePhyTimer_IRQn

_internal_rw rt_uint32_t _s_system_freq = 0;

static void arch_timer_init(void)
{
    _s_system_freq = arch_timer_reg_get_cp15_cntfrq();

    arch_timer_reg_write_cp15_ctrl(0);
    arch_timer_reg_write_cp15_tval(_s_system_freq);
    arch_timer_reg_write_cp15_ctrl(1);
}

static void sys_counter_clock_init(void)
{
    //using imx6ull kernel rom clock setting.
}

static void sys_counter_init(void)
{
    SCTR_Type *sctr = RT_NULL;
    rt_uint32_t reg_val;
    
    sys_counter_clock_init();

    sctr = (SCTR_Type*)platform_get_periph_vaddr(REALVIEW_SCTL_BASE);

    arch_timer_reg_write_cp15_cntfrq(HW_SYS_COUNTER_CLOCK);
    sctr->cntfid0 = HW_SYS_COUNTER_CLOCK;
    
    reg_val = sctr->cntcr;
    reg_val &= ~(SC_CNTCR_FREQ1 | SC_CNTCR_FREQ0);
    reg_val |=  (SC_CNTCR_FREQ0 | SC_CNTCR_HDBG | SC_CNTCR_ENABLE);
    sctr->cntcr = reg_val;

    CCM->CLPCR &= ~(CCM_CLPCR_ARM_CLK_DIS_ON_LPM_MASK | CCM_CLPCR_LPM_MASK);
}

static void _sys_counter_isr(int vector, void *param)
{
    rt_tick_increase();

    /* set next irq */
    arch_timer_reg_write_cp15_ctrl(0);
    arch_timer_reg_write_cp15_cval(arch_timer_reg_get_cp15_cval() + SYS_COUNTER_TICK_PERIOD);
    arch_timer_reg_write_cp15_ctrl(1);
}

int rt_hw_timer_init(void)
{
    sys_counter_init();
    arch_timer_init();

    rt_hw_interrupt_install(IRQ_SYS_COUNTER, _sys_counter_isr, RT_NULL, "tick");
    rt_hw_interrupt_umask(IRQ_SYS_COUNTER);

    return RT_EOK;
}
INIT_BOARD_EXPORT(rt_hw_timer_init);

