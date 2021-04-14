/*
 * IMX6ULL
 *   imx6ull clock bsp file
 *
 * Change Logs:
 * Date           Author       Notes
 * 2020-12-20     Lyons        first version
 */

#ifndef __BSP_CLOCK_H__
#define __BSP_CLOCK_H__

/* only used by MCIMX6Y2.h */
extern uint32_t *_g_ccm_vbase;
extern uint32_t *_g_ccm_analog_vbase;
extern uint32_t *_g_pmu_vbase;

void SystemClockInit(void);

void rt_hw_ms_delay(uint32_t ms);

#endif //#ifndef __BSP_CLOCK_H__
 
