/*
 * IMX6ULL
 *   imx6ull clock bsp file
 *
 * Change Logs:
 * Date           Author       Notes
 * 2020-12-20     Lyons        first version
 */

#include <rtconfig.h>
#include <rthw.h>

#include <board.h>

#include "__def.h"
#include "realview.h"
#include "skt.h"

#define _P2V(pa)                    platform_get_periph_vaddr((rt_uint32_t)(pa))

#if (defined(CPU_MCIMX6Y2CVM05) || defined(CPU_MCIMX6Y2DVM05) || defined(CPU_MCIMX6Y0CVM05) || \
     defined(CPU_MCIMX6Y0DVM05) || defined(CPU_MCIMX6Y1CVM05) || defined(CPU_MCIMX6Y1CVK05) || \
     defined(CPU_MCIMX6Y1DVM05) || defined(CPU_MCIMX6Y1DVK05) || defined(CPU_MCIMX6Y7DVK05))
_internal_ro clock_arm_pll_config_t g_ccmConfigArmPll_1056M = {.loopDivider = 88U};
#elif defined(CPU_MCIMX6Y2CVM08)
_internal_ro clock_arm_pll_config_t g_ccmConfigArmPll_792M  = {.loopDivider = 66U};
#elif(defined(CPU_MCIMX6Y2DVM09) || defined(CPU_MCIMX6Y7DVM09))
_internal_ro clock_arm_pll_config_t g_ccmConfigArmPll_900M  = {.loopDivider = 75U};
#endif

_internal_rw uint32_t _g_gpt1_vbase = REALVIEW_GPT1_BASE;
_internal_rw uint32_t _k_gpt1_load_value = RT_UINT32_MAX;

/* only used by MCIMX6Y2.h */
uint32_t *_g_ccm_vbase = (uint32_t*)REALVIEW_CCM_BASE;
uint32_t *_g_ccm_analog_vbase = (uint32_t*)REALVIEW_CCM_ANALOGY_BASE;
uint32_t *_g_pmu_vbase = (uint32_t*)REALVIEW_PMU_BASE;

rt_inline void _clk_enable( CCM_Type *base )
{
    base->CCGR0 = 0XFFFFFFFF;
    base->CCGR1 = 0XFFFFFFFF;
    base->CCGR2 = 0XFFFFFFFF;
    base->CCGR3 = 0XFFFFFFFF;
    base->CCGR4 = 0XFFFFFFFF;
    base->CCGR5 = 0XFFFFFFFF;
    base->CCGR6 = 0XFFFFFFFF;
}

void BOARD_BootClockRUN(void)
{
    /* Boot ROM did initialize the XTAL, here we only sets external XTAL OSC freq */
    CLOCK_SetXtalFreq(24000000U);
    CLOCK_SetRtcXtalFreq(32768U);

    /* Switch CPU off ARM PLL */
    if (CLOCK_GetMux(kCLOCK_Pll1SwMux) == 0)    /* CPU runs on ARM PLL */
    {
        CLOCK_SetMux(kCLOCK_StepMux, 0);        /* Set Step MUX to OSC */
        CLOCK_SetMux(kCLOCK_Pll1SwMux, 1);      /* Let CPU run on Step MUX */
    }
#if (defined(CPU_MCIMX6Y2CVM05) || defined(CPU_MCIMX6Y2DVM05) || defined(CPU_MCIMX6Y0CVM05) || \
     defined(CPU_MCIMX6Y0DVM05) || defined(CPU_MCIMX6Y1CVM05) || defined(CPU_MCIMX6Y1CVK05) || \
     defined(CPU_MCIMX6Y1DVM05) || defined(CPU_MCIMX6Y1DVK05) || defined(CPU_MCIMX6Y7DVK05))
    CLOCK_InitArmPll(&g_ccmConfigArmPll_1056M); /* Configure ARM PLL to 1056M */
    CLOCK_SetMux(kCLOCK_Pll1SwMux, 0);          /* Now CPU runs again on ARM PLL*/
    CLOCK_SetDiv(kCLOCK_ArmDiv, 1);             /* Configure ARM clock root with divide 2 */
#elif defined(CPU_MCIMX6Y2CVM08)
    CLOCK_InitArmPll(&g_ccmConfigArmPll_792M);  /* Configure ARM PLL to 792M */
    CLOCK_SetMux(kCLOCK_Pll1SwMux, 0);          /* Now CPU runs again on ARM PLL*/
    CLOCK_SetDiv(kCLOCK_ArmDiv, 0);             /* Configure ARM clock root with divide 1 */
#elif(defined(CPU_MCIMX6Y2DVM09) || defined(CPU_MCIMX6Y7DVM09))
    CLOCK_InitArmPll(&g_ccmConfigArmPll_900M);  /* Configure ARM PLL to 900M */
    CLOCK_SetMux(kCLOCK_Pll1SwMux, 0);          /* Now CPU runs again on ARM PLL*/
    CLOCK_SetDiv(kCLOCK_ArmDiv, 0);             /* Configure ARM clock root with divide 1 */
#endif

    /* Power down all unused PLL */
    CLOCK_DeinitUsb2Pll();
    CLOCK_DeinitAudioPll();
    CLOCK_DeinitVideoPll();
    CLOCK_DeinitEnetPll();

    /* Configure UART divider to default */
    CLOCK_SetMux(kCLOCK_UartMux, 0);            /* Set UART source to PLL3 80M */
    CLOCK_SetDiv(kCLOCK_UartDiv, 0);            /* Set UART divider to 1 */

    CLOCK_SetMux(kCLOCK_PerclkMux, 0);          /* Set I2C/EPIT source to IPG 66M */
    CLOCK_SetDiv(kCLOCK_PerclkDiv, 0);          /* Set I2C/EPIT divider to 1 */
}

void BOARD_DelayInit(void)
{
    GPT_Type *_GPT = RT_NULL;

#ifdef RT_USING_USERSPACE
    _g_gpt1_vbase = _P2V(_g_gpt1_vbase);
#endif
    _GPT = (GPT_Type*)_g_gpt1_vbase;

    _GPT->CR = 0;

    _GPT->CR = GPT_CR_SWR(1);
    while (_GPT->CR & GPT_CR_SWR_MASK);

    /*
     * 000 No clock
     * 001 derive clock from ipg_clk
     * 010 derive clock from ipg_clk_highfreq
     * 011 derive clock from External Clock
     * 100 derive clock from ipg_clk_32k
     * 101 derive clock from ipg_clk_24M
     */
    _GPT->CR = GPT_CR_CLKSRC(0x1);

    _GPT->PR = GPT_PR_PRESCALER(65); //Set GPT1 Clock to 66MHz/66 = 1MHz

    _GPT->OCR[0] = GPT_OCR_COMP(_k_gpt1_load_value);

    _GPT->CR |= GPT_CR_EN(1);
}

void SystemClockInit(void)
{
#ifdef RT_USING_USERSPACE
    _g_ccm_vbase = (uint32_t*)_P2V(_g_ccm_vbase);
    _g_ccm_analog_vbase = (uint32_t*)_P2V(_g_ccm_analog_vbase);
    _g_pmu_vbase = (uint32_t*)_P2V(_g_pmu_vbase);
#endif

    BOARD_BootClockRUN();
    BOARD_DelayInit();

    _clk_enable(CCM);
}

void rt_hw_us_delay(rt_uint32_t us)
{
    GPT_Type *_GPT = RT_NULL;
    rt_uint64_t old_cnt, new_cnt;
    rt_uint64_t total = 0;

    _GPT = (GPT_Type*)_g_gpt1_vbase;

    old_cnt = _GPT->CNT;
    while (1)
    {
        new_cnt = _GPT->CNT;
        if (old_cnt != new_cnt)
        {
            if (new_cnt > old_cnt)
            {
                total += (new_cnt - old_cnt);
            } else {
                total += (new_cnt + _k_gpt1_load_value - old_cnt);
            }
            old_cnt = new_cnt;

            if (total >= us)
                break;
        }
    }
}

void rt_hw_ms_delay(rt_uint32_t ms)
{
    while (ms--)
    {
        rt_hw_us_delay(1000);
    }
}

