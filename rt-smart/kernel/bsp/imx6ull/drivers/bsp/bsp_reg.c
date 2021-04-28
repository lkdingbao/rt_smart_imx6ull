/*
 * IMX6ULL
 *   imx6ull reg bsp file
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-04-27     Lyons        first version
 */

#include "bsp_reg.h"
#include "bsp_io.h"

/* 1 second delay should be plenty of time for block reset. */
#define RESET_MAX_TIMEOUT   1000000

#define MXS_BLOCK_SFTRST    (1 << 31)
#define MXS_BLOCK_CLKGATE   (1 << 30)

int mxs_wait_mask_set(struct mxs_register_32 *reg, uint32_t mask, unsigned
                                int timeout)
{
    while (--timeout) {
        if ((readl(&reg->reg) & mask) == mask)
            break;
        rt_hw_us_delay(1);
    }

    return !timeout;
}

int mxs_wait_mask_clr(struct mxs_register_32 *reg, uint32_t mask, unsigned
                                int timeout)
{
    while (--timeout) {
        if ((readl(&reg->reg) & mask) == 0)
            break;
        rt_hw_us_delay(1);
    }

    return !timeout;
}

int mxs_reset_block(struct mxs_register_32 *reg)
{
    /* Clear SFTRST */
    writel(MXS_BLOCK_SFTRST, &reg->reg_clr);

    if (mxs_wait_mask_clr(reg, MXS_BLOCK_SFTRST, RESET_MAX_TIMEOUT))
    	return 1;

    /* Clear CLKGATE */
    writel(MXS_BLOCK_CLKGATE, &reg->reg_clr);

    /* Set SFTRST */
    writel(MXS_BLOCK_SFTRST, &reg->reg_set);

    /* Wait for CLKGATE being set */
    if (mxs_wait_mask_set(reg, MXS_BLOCK_CLKGATE, RESET_MAX_TIMEOUT))
        return 1;

    /* Clear SFTRST */
    writel(MXS_BLOCK_SFTRST, &reg->reg_clr);

    if (mxs_wait_mask_clr(reg, MXS_BLOCK_SFTRST, RESET_MAX_TIMEOUT))
        return 1;

    /* Clear CLKGATE */
    writel(MXS_BLOCK_CLKGATE, &reg->reg_clr);

    if (mxs_wait_mask_clr(reg, MXS_BLOCK_CLKGATE, RESET_MAX_TIMEOUT))
        return 1;

    return 0;
}

