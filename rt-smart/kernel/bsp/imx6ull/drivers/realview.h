/*
 * IMX6ULL
 *   imx6ull arch timer driver file
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-01-13     Lyons        first version
 */

#ifndef __REALVIEW_H__
#define __REALVIEW_H__

#include "imx6ull_common.h"

#define __REG32(x)                  (*((volatile unsigned int *)(x)))
#define __REG16(x)                  (*((volatile unsigned short *)(x)))

#define REALVIEW_PERIPH_SIZE        (16 * 1024)

/* Interrupt Control Interface */
#define ARM_GIC_CPU_BASE            0x00A00000

/*
 * Peripheral addresses
 */
#define REALVIEW_UART1_BASE         UART1_BASE  /* UART 1 */
#define REALVIEW_UART2_BASE         UART2_BASE  /* UART 2 */
#define REALVIEW_UART3_BASE         UART3_BASE  /* UART 3 */
#define REALVIEW_UART4_BASE         UART4_BASE  /* UART 4 */
#define REALVIEW_UART5_BASE         UART5_BASE  /* UART 5 */
#define REALVIEW_UART6_BASE         UART6_BASE  /* UART 6 */
#define REALVIEW_UART7_BASE         UART7_BASE  /* UART 7 */
#define REALVIEW_UART8_BASE         UART8_BASE  /* UART 8 */

#define REALVIEW_SSP_BASE           0U          /* Synchronous Serial Port */
#define REALVIEW_WATCHDOG_BASE      0U          /* watchdog interface */

#define REALVIEW_GPIO1_BASE         GPIO1_BASE  /* GPIO port 0 */
#define REALVIEW_GPIO2_BASE         GPIO2_BASE  /* GPIO port 1 */
#define REALVIEW_GPIO3_BASE         GPIO3_BASE  /* GPIO port 2 */
#define REALVIEW_GPIO4_BASE         GPIO4_BASE  /* GPIO port 3 */
#define REALVIEW_GPIO5_BASE         GPIO5_BASE  /* GPIO port 4 */

#define REALVIEW_RTC_BASE           0U          /* Real Time Clock */
#define REALVIEW_TIMER0_1_BASE      0U          /* Timer 0 and 1 */
#define REALVIEW_TIMER2_3_BASE      0U          /* Timer 2 and 3 */
#define REALVIEW_TIMER4_5_BASE      0U          /* Timer 4/5 */
#define REALVIEW_TIMER6_7_BASE      0U          /* Timer 6/7 */

#define REALVIEW_SCTL_BASE          0x021DC000u /* System Controller */

#define REALVIEW_CLCD_BASE          LCDIF_BASE  /* CLCD */

#define REALVIEW_DMC_BASE           0U          /* DMC configuration */
#define REALVIEW_SMC_BASE           0U          /* SMC configuration */
#define REALVIEW_CAN_BASE           0U          /* CAN bus */

#define REALVIEW_GIC_DIST_BASE      (ARM_GIC_CPU_BASE+0x1000)  /* Generic interrupt controller distributor */
#define REALVIEW_GIC_CPU_BASE       (ARM_GIC_CPU_BASE+0x2000)  /* Generic interrupt controller CPU interface */

#define REALVIEW_IOMUXC_BASE        IOMUXC_BASE
#define REALVIEW_IOMUXC_SNVS_BASE   IOMUXC_SNVS_BASE
#define REALVIEW_IOMUXC_GPR_BASE    IOMUXC_GPR_BASE

#define REALVIEW_CCM_BASE           0x20C4000u
#define REALVIEW_CCM_ANALOGY_BASE   0x20C8000u
#define REALVIEW_PMU_BASE           0x20C8110u

#define REALVIEW_ENET1_BASE         ENET1_BASE
#define REALVIEW_ENET2_BASE         ENET2_BASE

#define REALVIEW_GPT1_BASE          GPT1_BASE
#define REALVIEW_GPT2_BASE          GPT2_BASE

#define REALVIEW_ECSPI1_BASE        ECSPI1_BASE
#define REALVIEW_ECSPI2_BASE        ECSPI2_BASE
#define REALVIEW_ECSPI3_BASE        ECSPI3_BASE
#define REALVIEW_ECSPI4_BASE        ECSPI4_BASE

#define REALVIEW_I2C1_BASE          I2C1_BASE
#define REALVIEW_I2C2_BASE          I2C2_BASE
#define REALVIEW_I2C3_BASE          I2C3_BASE
#define REALVIEW_I2C4_BASE          I2C4_BASE

#define REALVIEW_SDMA_BASE          SDMAARM_BASE

#include <rtdef.h>
#include <armv7.h>

/* number of interrupts on board */
#define ARM_GIC_NR_IRQS             160
/* only one GIC available */
#define ARM_GIC_MAX_NR              1

#endif //#ifndef __REALVIEW_H__

