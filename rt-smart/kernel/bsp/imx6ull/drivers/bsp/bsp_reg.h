/*
 * IMX6ULL
 *   imx6ull reg bsp file
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-04-27     Lyons        first version
 */

#ifndef __BSP_REG_H__
#define __BSP_REG_H__

#include <rtconfig.h>
#include <rthw.h>

#define __mxs_reg_32(name)      \
    uint32_t name;              \
    uint32_t name##_set;        \
    uint32_t name##_clr;        \
    uint32_t name##_tog;

struct mxs_register_32 {
    __mxs_reg_32(reg)
};

int mxs_wait_mask_set(struct mxs_register_32 *reg, uint32_t mask, unsigned
                                int timeout);

int mxs_wait_mask_clr(struct mxs_register_32 *reg, uint32_t mask, unsigned
                                int timeout);

int mxs_reset_block(struct mxs_register_32 *reg);

#endif //#ifndef __BSP_REG_H__

