#include <rtconfig.h>
#include <rthw.h>
#include <rtdevice.h>

#include <board.h>
#ifdef RT_USING_LWP
#include <lwp.h>
#include <lwp_user_mm.h>
#endif

#include "__def.h"
#include "realview.h"

int do_reboot(int argc, char **argv)
{
    wdog_config_t config;
    SRC_Type *src = (SRC_Type*)platform_get_periph_vaddr(REALVIEW_SRC_BASE);
    WDOG_Type *wdog = (WDOG_Type*)platform_get_periph_vaddr(REALVIEW_WATCHDOG1_BASE);

    rt_kprintf("resetting ...\n");

    rt_hw_ms_delay(50);

    src->SCR &= ~SRC_SCR_WARM_RESET_ENABLE_MASK;

    CLOCK_EnableClock(kCLOCK_Wdog1);

    WDOG_GetDefaultConfig(&config);
    config.timeoutValue = 0x00u;

    WDOG_Init(wdog, &config);

    while (1)
    {
        //waiting...
    }

    /* NOTREACHED */
    return 0;
}
MSH_CMD_EXPORT_ALIAS(do_reboot, reboot, reboot system);

