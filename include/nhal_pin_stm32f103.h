#ifndef NHAL_PIN_STM32F103_H
#define NHAL_PIN_STM32F103_H

#include "nhal_pin_types.h"
#include "stm32f103_helpers.h"

struct nhal_pin_id {
    stm32f103_gpio_port_t port;
    stm32f103_gpio_pin_t pin;
};

struct nhal_pin_impl_config {
    uint32_t reserved;  // No implementation-specific config needed
};

/**
 * @brief STM32F103 pin context structure
 * 
 * Contains pin identification and interrupt state for STM32F103 implementation.
 */
struct nhal_pin_context {
    struct nhal_pin_id *pin_id;           // Pin identification (port + pin)
    
    // Interrupt support
    nhal_pin_callback_t callback;         // User callback function  
    void *user_data;                      // User data for callback
    nhal_pin_int_trigger_t trigger;       // Current trigger type
    bool interrupt_configured;            // Whether interrupt is configured
    bool interrupt_enabled;               // Whether interrupt is enabled
};

#endif