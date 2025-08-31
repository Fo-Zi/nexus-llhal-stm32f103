#include <stddef.h>
#include "nhal_common.h"
#include "nhal_pin.h"
#include "nhal_pin_stm32f103.h"
#include "stm32f103_registers.h"
#include "stm32f103_helpers.h"
#include "internal/stm32f103_internal.h"

// Platform configuration - should be defined by Platform Integration Layer
#ifndef PLATFORM_STM32F103_PACKAGE
#define PLATFORM_STM32F103_PACKAGE STM32F103_PACKAGE_48PIN_LQFP  // Default to Blue Pill
#endif

static bool platform_stm32f103_is_pin_valid(stm32f103_gpio_port_t port, stm32f103_gpio_pin_t pin) {
    stm32f103_pin_t pin_struct = { port, pin };
    return stm32f103_is_pin_available(PLATFORM_STM32F103_PACKAGE, pin_struct);
}

static void stm32f103_gpio_enable_clock(stm32f103_gpio_port_t port) {
    uint32_t clk_mask = stm32f103_get_gpio_clk_mask(port);
    RCC_APB2ENR |= clk_mask;
}

static void stm32f103_gpio_config_pin(uint32_t gpio_base, uint32_t pin, 
                                      nhal_pin_dir_t direction, nhal_pin_pull_mode_t pull_mode) {
    uint32_t mode = 0;
    uint32_t cnf = 0;
    
    if (direction == NHAL_PIN_DIR_OUTPUT) {
        mode = GPIO_MODE_OUTPUT_50M;
        cnf = GPIO_CNF_OUTPUT_PUSHPULL;
    } else {
        mode = GPIO_MODE_INPUT;
        if (pull_mode == NHAL_PIN_PMODE_PULL_UP || pull_mode == NHAL_PIN_PMODE_PULL_DOWN) {
            cnf = GPIO_CNF_INPUT_PULLUP;
        } else {
            cnf = GPIO_CNF_INPUT_FLOATING;
        }
    }
    
    uint32_t config = (cnf << 2) | mode;
    
    if (pin < 8) {
        uint32_t shift = pin * 4;
        uint32_t mask = 0xF << shift;
        uint32_t current = GPIO_CRL(gpio_base);
        GPIO_CRL(gpio_base) = (current & ~mask) | (config << shift);
    } else {
        uint32_t shift = (pin - 8) * 4;
        uint32_t mask = 0xF << shift;
        uint32_t current = GPIO_CRH(gpio_base);
        GPIO_CRH(gpio_base) = (current & ~mask) | (config << shift);
    }
    
    // Set pull-up/pull-down by writing to ODR for input pins
    if (direction == NHAL_PIN_DIR_INPUT && pull_mode == NHAL_PIN_PMODE_PULL_UP) {
        GPIO_BSRR(gpio_base) = (1U << pin);
    } else if (direction == NHAL_PIN_DIR_INPUT && pull_mode == NHAL_PIN_PMODE_PULL_DOWN) {
        GPIO_BRR(gpio_base) = (1U << pin);
    }
}

nhal_result_t nhal_pin_init(struct nhal_pin_context *pin_ctxt) {
    if (!pin_ctxt || !pin_ctxt->pin_id) {
        return NHAL_ERR_INVALID_ARG;
    }
    
    // Validate pin availability on this MCU package
    if (!platform_stm32f103_is_pin_valid(pin_ctxt->pin_id->port, pin_ctxt->pin_id->pin)) {
        return NHAL_ERR_INVALID_CONFIG;
    }
    
    // Validate port exists
    uint32_t gpio_base = stm32f103_get_gpio_base(pin_ctxt->pin_id->port);
    if (gpio_base == 0) {
        return NHAL_ERR_INVALID_CONFIG;
    }
    
    stm32f103_gpio_enable_clock(pin_ctxt->pin_id->port);
    RCC_APB2ENR |= RCC_APB2ENR_AFIOEN;
    
    return NHAL_OK;
}

nhal_result_t nhal_pin_deinit(struct nhal_pin_context *pin_ctxt) {
    if (!pin_ctxt || !pin_ctxt->pin_id) {
        return NHAL_ERR_INVALID_ARG;
    }
    
    return NHAL_OK;
}

nhal_result_t nhal_pin_set_config(struct nhal_pin_context *pin_ctxt, struct nhal_pin_config *config) {
    if (!pin_ctxt || !pin_ctxt->pin_id || !config) {
        return NHAL_ERR_INVALID_ARG;
    }
    
    uint32_t gpio_base = stm32f103_get_gpio_base(pin_ctxt->pin_id->port);
    stm32f103_gpio_config_pin(gpio_base, 
                              pin_ctxt->pin_id->pin,
                              config->direction, 
                              config->pull_mode);
    
    return NHAL_OK;
}

nhal_result_t nhal_pin_get_config(struct nhal_pin_context *pin_ctxt, struct nhal_pin_config *config) {
    if (!pin_ctxt || !pin_ctxt->pin_id || !config) {
        return NHAL_ERR_INVALID_ARG;
    }
    
    // TODO: Read actual pin configuration from hardware registers
    // For now, return basic config structure
    config->impl_config = NULL;
    
    return NHAL_OK;
}

nhal_result_t nhal_pin_set_state(struct nhal_pin_context *pin_ctxt, nhal_pin_state_t value) {
    if (!pin_ctxt || !pin_ctxt->pin_id) {
        return NHAL_ERR_INVALID_ARG;
    }
    
    uint32_t gpio_base = stm32f103_get_gpio_base(pin_ctxt->pin_id->port);
    uint32_t pin_mask = 1U << pin_ctxt->pin_id->pin;
    
    if (value == NHAL_PIN_HIGH) {
        GPIO_BSRR(gpio_base) = pin_mask;
    } else {
        GPIO_BRR(gpio_base) = pin_mask;
    }
    
    return NHAL_OK;
}

nhal_result_t nhal_pin_get_state(struct nhal_pin_context *pin_ctxt, nhal_pin_state_t *value) {
    if (!pin_ctxt || !pin_ctxt->pin_id || !value) {
        return NHAL_ERR_INVALID_ARG;
    }
    
    uint32_t gpio_base = stm32f103_get_gpio_base(pin_ctxt->pin_id->port);
    uint32_t idr_value = GPIO_IDR(gpio_base);
    uint32_t pin_state = (idr_value >> pin_ctxt->pin_id->pin) & 0x01;
    
    *value = pin_state ? NHAL_PIN_HIGH : NHAL_PIN_LOW;
    
    return NHAL_OK;
}

nhal_result_t nhal_pin_set_callback(struct nhal_pin_context *pin_ctxt, nhal_pin_callback_t callback) {
    (void)pin_ctxt;
    (void)callback;
    return NHAL_ERR_UNSUPPORTED;
}

nhal_result_t nhal_pin_set_direction(struct nhal_pin_context *pin_ctxt, nhal_pin_dir_t direction, nhal_pin_pull_mode_t pull_mode) {
    if (!pin_ctxt || !pin_ctxt->pin_id) {
        return NHAL_ERR_INVALID_ARG;
    }
    
    uint32_t gpio_base = stm32f103_get_gpio_base(pin_ctxt->pin_id->port);
    stm32f103_gpio_config_pin(gpio_base, 
                              pin_ctxt->pin_id->pin,
                              direction, 
                              pull_mode);
    
    return NHAL_OK;
}