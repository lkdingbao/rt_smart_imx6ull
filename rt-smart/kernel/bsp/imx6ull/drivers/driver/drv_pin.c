/*
 * IMX6ULL
 *   imx6ull pin driver file
 *
 * Change Logs:
 * Date           Author       Notes
 * 2020-01-13     Lyons        edit and remove irq setting
 */

#include <rtconfig.h>
#include <rthw.h>
 
#include <board.h>
#ifdef RT_USING_LWP
#include <lwp.h>
#endif

#include "__def.h"
#include "realview.h"
#include "bsp_gpio.h"
#include "drv_pin.h"

#define _DEVICE_NAME            "pin"

/* imx6ull support only 5 port! */
#define _GPIO_PORT_NUM          (5)

/* The valid pin bit mask, continuous for imx6ull */
_internal_ro rt_uint32_t _k_pin_valid_mask_tbl[_GPIO_PORT_NUM] = 
{
    0xFFFFFFFF,     /* IO31~IO0 supported 32 pins */
    0x003FFFFF,     /* IO21~IO0 supported 22 pins */
    0x01FFFFFF,     /* IO28~IO0 supported 29 pins */
    0x01FFFFFF,     /* IO28~IO0 supported 29 pins */
    0x00000FFF,     /* IO11~IO0 supported 12 pins */
};

/* The phys base address of iomux periph */
_internal_ro rt_uint32_t _k_periph_base_tbl[_GPIO_PORT_NUM] =
{
    REALVIEW_IOMUXC_BASE,
    REALVIEW_IOMUXC_BASE,
    REALVIEW_IOMUXC_BASE,
    REALVIEW_IOMUXC_BASE,
    REALVIEW_IOMUXC_SNVS_BASE,
};

/* The offset of reg SW_MUX_CTL_PAD or SW_PAD_CTL_PAD */
_internal_ro rt_uint8_t _k_periph_reg_offset_tbl[_GPIO_PORT_NUM][32] =
{
    /* GPIO1 use IOMUXC as periph address */
    {   0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D,
        0x0E, 0x0F, 0x00, 0x01, 0x02, 0x03, 0x04, 0x05,
        0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17,
        0x18, 0x19, 0x1A, 0x1B, 0x1C, 0x1D, 0x1E, 0x1F, },
    /* GPIO2 use IOMUXC as periph address */
    {   0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27,
        0x28, 0x29, 0x2A, 0x2B, 0x2C, 0x2D, 0x2E, 0x2F,
        0x5E, 0x5F, 0x60, 0x61, 0x62, 0x63 },
    /* GPIO3 use IOMUXC as periph address */
    {   0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37,
        0x38, 0x39, 0x3A, 0x3B, 0x3C, 0x3D, 0x3E, 0x3F,
        0x40, 0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47,
        0x48, 0x49, 0x4A, 0x4B, 0x4C },
    /* GPIO4 use IOMUXC as periph address */
    {   0x4D, 0x4E, 0x4F, 0x50, 0x51, 0x52, 0x53, 0x54,
        0x55, 0x56, 0x57, 0x58, 0x59, 0x5A, 0x5B, 0x5C,
        0x5D, 0x64, 0x65, 0x66, 0x67, 0x68, 0x69, 0x6A,
        0x6B, 0x6C, 0x6D, 0x6E, 0x6F },
    /* GPIO5 use IOMUXC_SNVS as periph address */
    {   0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09,
        0x0A, 0x0B, 0x00, 0x01 },
};

static void _pin_ops_pin_mode( struct rt_device *device,
                               rt_base_t pin,
                               rt_base_t mode )
{
    gpio_pin_config_t config;
    rt_uint32_t paddr, vaddr, offset, value;
    rt_uint32_t reg_addr[2]; //[0]-mux reg, [1]-pad reg
    rt_uint8_t port_num, pin_num;

    RT_ASSERT(RT_NULL != device);

    port_num = GET_PORT_FIELD(pin);
    pin_num = GET_PIN_FIELD(pin);

    RT_ASSERT(port_num < _GPIO_PORT_NUM);
    RT_ASSERT(_k_pin_valid_mask_tbl[port_num] & (1<<pin_num));

    config.interruptMode = kGPIO_NoIntmode;
    config.outputLogic = PIN_LOW;

    switch (mode)
    {
        case PIN_MODE_OUTPUT:
            config.direction = kGPIO_DigitalOutput;
            value = 0x0030U;
            break;

        case PIN_MODE_INPUT:
            config.direction = kGPIO_DigitalInput;
            value = 0x0830U;
            break;

        case PIN_MODE_INPUT_PULLDOWN:
            config.direction = kGPIO_DigitalInput;
            value = 0x3030U;
            break;

        case PIN_MODE_INPUT_PULLUP:
            config.direction = kGPIO_DigitalInput;
            value = 0xB030U;
            break;

        case PIN_MODE_OUTPUT_OD:
            config.direction = kGPIO_DigitalOutput;
            value = 0x0830U;
            break;

        default:
            config.direction = kGPIO_DigitalOutput;
            value = 0x0830U;
            break;
    }

    offset = _k_periph_reg_offset_tbl[port_num][pin_num];
    paddr = _k_periph_base_tbl[port_num];

    if (REALVIEW_IOMUXC_SNVS_BASE == paddr)
    {
        IOMUXC_SNVS_Type *periph = (IOMUXC_SNVS_Type*)platform_get_periph_vaddr((rt_uint32_t)paddr);

        reg_addr[0] = (uint32_t)&periph->SW_MUX_CTL_PAD[offset];
        reg_addr[1] = (uint32_t)&periph->SW_PAD_CTL_PAD[offset];
    } else {
        IOMUXC_Type *periph = (IOMUXC_Type*)platform_get_periph_vaddr((rt_uint32_t)paddr);

        reg_addr[0] = (uint32_t)&periph->SW_MUX_CTL_PAD[offset];
        reg_addr[1] = (uint32_t)&periph->SW_PAD_CTL_PAD[offset];
    }

    IOMUXC_SetPinMux(reg_addr[0], 0x5U, 0x00000000U, 0x0U, reg_addr[1], 0);
    IOMUXC_SetPinConfig(reg_addr[0], 0x5U, 0x00000000U, 0x0U, reg_addr[1], value);

    paddr = GET_GPIO_BASE_ADDR(port_num);
    vaddr = platform_get_periph_vaddr((rt_uint32_t)paddr);
    gpio_set_mode((GPIO_Type*)vaddr, pin_num, &config);
}

static void _pin_ops_pin_write( struct rt_device *device,
                                rt_base_t pin,
                                rt_base_t value )
{
    rt_uint8_t port_num, pin_num;
    rt_uint32_t paddr, vaddr;

    RT_ASSERT(RT_NULL != device);

    port_num = GET_PORT_FIELD(pin);
    pin_num = GET_PIN_FIELD(pin);

    RT_ASSERT(port_num < _GPIO_PORT_NUM);
    RT_ASSERT(_k_pin_valid_mask_tbl[port_num] & (1<<pin_num));

    paddr = GET_GPIO_BASE_ADDR(port_num);
    vaddr = platform_get_periph_vaddr((rt_uint32_t)paddr);
    GPIO_WritePinOutput((GPIO_Type*)vaddr, pin_num, value);
}

static int _pin_ops_pin_read( struct rt_device *device,
                              rt_base_t pin )
{
    rt_uint8_t port_num, pin_num;
    rt_uint32_t paddr, vaddr, value;

    RT_ASSERT(RT_NULL != device);

    port_num = GET_PORT_FIELD(pin);
    pin_num = GET_PIN_FIELD(pin);

    RT_ASSERT(port_num < _GPIO_PORT_NUM);
    RT_ASSERT(_k_pin_valid_mask_tbl[port_num] & (1<<pin_num));

    paddr = GET_GPIO_BASE_ADDR(port_num);
    vaddr = platform_get_periph_vaddr((rt_uint32_t)paddr);
    value = GPIO_ReadPadStatus((GPIO_Type*)vaddr, pin_num);

    return (!value) ? PIN_HIGH : PIN_LOW;
}

_internal_ro struct rt_pin_ops _k_ops_pin =
{
    _pin_ops_pin_mode,      /* pin_mode */
    _pin_ops_pin_write,     /* pin_write */
    _pin_ops_pin_read,      /* pin_read */
    RT_NULL,                /* pin_attach_irq */
    RT_NULL,                /* pin_detach_irq */
    RT_NULL,                /* pin_irq_enable */
    RT_NULL,                /* pin_get */
};

int rt_hw_pin_init(void)
{
    rt_device_pin_register(_DEVICE_NAME, &_k_ops_pin, RT_NULL);

    return RT_EOK;
}
INIT_BOARD_EXPORT(rt_hw_pin_init);

