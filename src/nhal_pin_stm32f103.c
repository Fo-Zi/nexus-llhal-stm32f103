#include <stddef.h>
#include <stdbool.h>
#include "nhal_common.h"
#include "nhal_pin.h"
#include "nhal_pin_stm32f103.h"
#include "stm32f103_registers.h"
#include "stm32f103_gpio.h"
#include "stm32f103_clock.h"

#include "stm32f103_exti.h"

// EXTI interrupt state tracking
typedef struct {
    struct nhal_pin_context *owner;      // Which pin context owns this EXTI line
    nhal_pin_callback_t callback;        // Active callback
    void *user_data;                     // User data for callback
} exti_line_state_t;

static exti_line_state_t exti_lines[16] = {0}; // One per EXTI line

// Platform configuration - should be defined by Platform Integration Layer
#ifndef PLATFORM_STM32F103_PACKAGE
#define PLATFORM_STM32F103_PACKAGE STM32F103_PACKAGE_48PIN_LQFP  // Default to Blue Pill
#endif

static bool platform_stm32f103_is_pin_valid(stm32f103_gpio_port_t port, stm32f103_gpio_pin_t pin) {
    stm32f103_pin_t pin_struct = { port, pin };
    return stm32f103_is_pin_available(PLATFORM_STM32F103_PACKAGE, pin_struct);
}

// EXTI interrupt helper functions
static nhal_result_t exti_claim_line(struct nhal_pin_context *pin_ctx, uint8_t line) {
    // Check if line is free or owned by same pin context
    if (exti_lines[line].owner != NULL && exti_lines[line].owner != pin_ctx) {
        return NHAL_ERR_BUSY; // Hardware conflict
    }

    exti_lines[line].owner = pin_ctx;
    return NHAL_OK;
}

// EXTI Interrupt Service Routines - Our Winner Approach!

void EXTI0_IRQHandler(void) {
    EXTI_PR = (1U << 0);  // Clear pending
    if (exti_lines[0].callback) {
        exti_lines[0].callback(exti_lines[0].owner, exti_lines[0].user_data);
    }
}

void EXTI1_IRQHandler(void) {
    EXTI_PR = (1U << 1);  // Clear pending
    if (exti_lines[1].callback) {
        exti_lines[1].callback(exti_lines[1].owner, exti_lines[1].user_data);
    }
}

void EXTI2_IRQHandler(void) {
    EXTI_PR = (1U << 2);  // Clear pending
    if (exti_lines[2].callback) {
        exti_lines[2].callback(exti_lines[2].owner, exti_lines[2].user_data);
    }
}

void EXTI3_IRQHandler(void) {
    EXTI_PR = (1U << 3);  // Clear pending
    if (exti_lines[3].callback) {
        exti_lines[3].callback(exti_lines[3].owner, exti_lines[3].user_data);
    }
}

void EXTI4_IRQHandler(void) {
    EXTI_PR = (1U << 4);  // Clear pending
    if (exti_lines[4].callback) {
        exti_lines[4].callback(exti_lines[4].owner, exti_lines[4].user_data);
    }
}

void EXTI9_5_IRQHandler(void) {
    uint32_t pending = EXTI_PR & 0x3E0; // Lines 5-9 only
    EXTI_PR = pending; // Clear all pending - KEY optimization!

    for (int line = 5; line <= 9 && pending; line++) {
        if (pending & (1U << line)) {
            if (exti_lines[line].callback) {
                exti_lines[line].callback(exti_lines[line].owner, exti_lines[line].user_data);
            }
            pending &= ~(1U << line); // Early termination optimization
        }
    }
}

void EXTI15_10_IRQHandler(void) {
    uint32_t pending = EXTI_PR & 0xFC00; // Lines 10-15 only
    EXTI_PR = pending; // Clear all pending - KEY optimization!

    for (int line = 10; line <= 15 && pending; line++) {
        if (pending & (1U << line)) {
            if (exti_lines[line].callback) {
                exti_lines[line].callback(exti_lines[line].owner, exti_lines[line].user_data);
            }
            pending &= ~(1U << line); // Early termination optimization
        }
    }
}

static void exti_release_line(struct nhal_pin_context *pin_ctx, uint8_t line) {
    if (exti_lines[line].owner == pin_ctx) {
        exti_lines[line].owner = NULL;
        exti_lines[line].callback = NULL;
        exti_lines[line].user_data = NULL;
    }
}

static nhal_result_t configure_afio_exti_mapping(stm32f103_gpio_port_t port, uint8_t line) {
    // Enable AFIO clock if not already enabled
    RCC_APB2ENR |= RCC_APB2ENR_AFIOEN;

    // Configure AFIO_EXTICRx register
    uint8_t reg_index = line / 4;        // Which EXTICR register (0-3)
    uint8_t field_pos = (line % 4) * 4;  // Bit position within register

    uint32_t port_value = (uint32_t)port; // Port enum to register value
    uint32_t mask = 0xF << field_pos;

    volatile uint32_t *exticr_reg = &AFIO_EXTICR1 + reg_index;
    *exticr_reg = (*exticr_reg & ~mask) | (port_value << field_pos);

    return NHAL_OK;
}

static nhal_result_t configure_exti_trigger(uint8_t line, nhal_pin_int_trigger_t trigger) {
    uint32_t line_mask = 1U << line;

    // Clear existing configuration
    EXTI_RTSR &= ~line_mask;
    EXTI_FTSR &= ~line_mask;

    switch (trigger) {
        case NHAL_PIN_INT_TRIGGER_RISING_EDGE:
            EXTI_RTSR |= line_mask;
            break;
        case NHAL_PIN_INT_TRIGGER_FALLING_EDGE:
            EXTI_FTSR |= line_mask;
            break;
        case NHAL_PIN_INT_TRIGGER_BOTH_EDGES:
            EXTI_RTSR |= line_mask;
            EXTI_FTSR |= line_mask;
            break;
        case NHAL_PIN_INT_TRIGGER_HIGH_LEVEL:
        case NHAL_PIN_INT_TRIGGER_LOW_LEVEL:
            // Level triggers not supported by STM32F103 hardware
            return NHAL_ERR_UNSUPPORTED;
        default:
            return NHAL_ERR_INVALID_ARG;
    }

    return NHAL_OK;
}

static void exti_enable_irq(uint8_t line) {
    IRQn_Type irq;
    if (line <= 4) {
        irq = (IRQn_Type)(EXTI0_IRQn + line);
    } else if (line <= 9) {
        irq = EXTI9_5_IRQn;
    } else {
        irq = EXTI15_10_IRQn;
    }
    NVIC_EnableIRQ(irq);
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
    
    // Small delay to ensure configuration takes effect
    for (volatile int i = 0; i < 10; i++) {
        __asm__ volatile ("nop");
    }

    // For STM32F103, input pull-up/pull-down is controlled by ODR register
    // Set pull-up/pull-down by writing to ODR for input pins
    if (direction == NHAL_PIN_DIR_INPUT) {
        if (pull_mode == NHAL_PIN_PMODE_PULL_UP) {
            GPIO_BSRR(gpio_base) = (1U << pin);  // Set bit to enable pull-up
        } else if (pull_mode == NHAL_PIN_PMODE_PULL_DOWN) {
            GPIO_BRR(gpio_base) = (1U << pin);   // Clear bit to enable pull-down
        }
        // For floating input, don't change ODR
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



nhal_result_t nhal_pin_set_interrupt_config(
    struct nhal_pin_context *pin_ctx,
    nhal_pin_int_trigger_t trigger,
    nhal_pin_callback_t callback,
    void *user_data
) {
    if (!pin_ctx || !pin_ctx->pin_id || !callback) {
        return NHAL_ERR_INVALID_ARG;
    }

    // Reject level triggers immediately - keep it simple!
    if (trigger == NHAL_PIN_INT_TRIGGER_HIGH_LEVEL ||
        trigger == NHAL_PIN_INT_TRIGGER_LOW_LEVEL) {
        return NHAL_ERR_UNSUPPORTED;
    }

    uint8_t exti_line = pin_ctx->pin_id->pin; // Pin number = EXTI line

    // Check for resource conflicts
    nhal_result_t result = exti_claim_line(pin_ctx, exti_line);
    if (result != NHAL_OK) {
        return result; // NHAL_ERR_BUSY if conflict
    }

    // Configure AFIO for pin-to-EXTI mapping
    result = configure_afio_exti_mapping(pin_ctx->pin_id->port, exti_line);
    if (result != NHAL_OK) {
        exti_release_line(pin_ctx, exti_line);
        return result;
    }

    // Configure EXTI trigger type
    result = configure_exti_trigger(exti_line, trigger);
    if (result != NHAL_OK) {
        exti_release_line(pin_ctx, exti_line);
        return result;
    }

    // Store callback information
    exti_lines[exti_line].callback = callback;
    exti_lines[exti_line].user_data = user_data;

    // Update pin context
    pin_ctx->callback = callback;
    pin_ctx->user_data = user_data;
    pin_ctx->trigger = trigger;
    pin_ctx->interrupt_configured = true;
    pin_ctx->interrupt_enabled = false; // Not enabled yet

    return NHAL_OK;
}

nhal_result_t nhal_pin_interrupt_enable(struct nhal_pin_context *pin_ctx) {
    if (!pin_ctx || !pin_ctx->pin_id) {
        return NHAL_ERR_INVALID_ARG;
    }

    if (!pin_ctx->interrupt_configured) {
        return NHAL_ERR_NOT_CONFIGURED;
    }

    uint8_t exti_line = pin_ctx->pin_id->pin;
    uint32_t line_mask = 1U << exti_line;

    // Enable EXTI interrupt mask
    EXTI_IMR |= line_mask;

    // Enable corresponding NVIC interrupt
    exti_enable_irq(exti_line);

    pin_ctx->interrupt_enabled = true;

    return NHAL_OK;
}

nhal_result_t nhal_pin_interrupt_disable(struct nhal_pin_context *pin_ctx) {
    if (!pin_ctx || !pin_ctx->pin_id) {
        return NHAL_ERR_INVALID_ARG;
    }

    if (!pin_ctx->interrupt_configured) {
        return NHAL_ERR_NOT_CONFIGURED;
    }

    uint8_t exti_line = pin_ctx->pin_id->pin;
    uint32_t line_mask = 1U << exti_line;

    // Disable EXTI interrupt mask
    EXTI_IMR &= ~line_mask;

    pin_ctx->interrupt_enabled = false;

    return NHAL_OK;
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
