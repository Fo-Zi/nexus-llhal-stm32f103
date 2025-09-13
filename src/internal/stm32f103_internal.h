#ifndef STM32F103_INTERNAL_H
#define STM32F103_INTERNAL_H

#include <stdint.h>
#include <stdbool.h>
#include "stm32f103_registers.h"
#include "stm32f103_helpers.h"

// Internal clock functions for STM32F103 HAL implementation
// These read actual hardware register state for accurate timing

/**
 * @brief Get system clock frequency by reading hardware registers
 * @return System clock frequency in Hz
 */
uint32_t stm32f103_get_sysclk_hz(void);

/**
 * @brief Get AHB clock frequency by reading hardware registers
 * @return AHB clock frequency in Hz
 */
uint32_t stm32f103_get_ahb_hz(void);

/**
 * @brief Get APB1 clock frequency by reading hardware registers
 * @return APB1 clock frequency in Hz
 */
uint32_t stm32f103_get_apb1_hz(void);

/**
 * @brief Get APB2 clock frequency by reading hardware registers
 * @return APB2 clock frequency in Hz
 */
uint32_t stm32f103_get_apb2_hz(void);

// STM32F103 constants
// STM32F103 package types
typedef enum {
    STM32F103_PACKAGE_48PIN_LQFP,    // C8T6, C6T6
    STM32F103_PACKAGE_64PIN_LQFP,    // R8T6, R6T6
    STM32F103_PACKAGE_100PIN_LQFP    // V8T6, V6T6
} stm32f103_package_t;

// Pin structure for cleaner API
typedef struct {
    stm32f103_gpio_port_t port;
    stm32f103_gpio_pin_t pin;
} stm32f103_pin_t;

/**
 * @brief Check if specific pin is available on STM32F103 package
 * @param package Package type
 * @param pin Pin to check
 * @return true if pin is available, false otherwise
 */
bool stm32f103_is_pin_available(stm32f103_package_t package, stm32f103_pin_t pin);

#endif
