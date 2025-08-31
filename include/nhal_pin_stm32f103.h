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

// No impl_ctx needed - pin_id contains all necessary information

#endif