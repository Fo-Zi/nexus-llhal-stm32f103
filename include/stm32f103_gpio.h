#ifndef STM32F103_GPIO_H
#define STM32F103_GPIO_H

#include <stdint.h>
#include <stdbool.h>
#include "stm32f103_registers.h"
#include "stm32f103_mcu.h"
#include "stm32f103_gpio_defs.h"

/**
 * @file stm32f103_gpio.h
 * @brief STM32F103 GPIO Utilities
 * 
 * This module provides GPIO port and pin management utilities for STM32F103.
 * It includes helper functions for port addressing, clock management, and
 * pin configuration.
 */

/*==============================================================================
 * GPIO Types and Definitions
 *============================================================================*/

// All GPIO type definitions are now in stm32f103_gpio_defs.h

/*==============================================================================
 * GPIO Port Utilities
 *============================================================================*/

/**
 * @brief Get GPIO port base address
 * @param port GPIO port enumeration
 * @return Base address of the GPIO port
 */
uint32_t stm32f103_get_gpio_base(stm32f103_gpio_port_t port);

/**
 * @brief Get GPIO port clock enable mask
 * @param port GPIO port enumeration
 * @return Clock enable mask for RCC_APB2ENR
 */
uint32_t stm32f103_get_gpio_clk_mask(stm32f103_gpio_port_t port);

/**
 * @brief Enable GPIO port clock
 * @param port GPIO port to enable
 * @return 0 on success, negative on error
 */
int stm32f103_gpio_enable_port_clock(stm32f103_gpio_port_t port);

/**
 * @brief Disable GPIO port clock
 * @param port GPIO port to disable
 * @return 0 on success, negative on error
 */
int stm32f103_gpio_disable_port_clock(stm32f103_gpio_port_t port);

/*==============================================================================
 * GPIO Pin Configuration
 *============================================================================*/

/**
 * @brief Configure individual GPIO pin
 * @param gpio_base GPIO port base address
 * @param pin Pin number (0-15)
 * @param mode GPIO mode (0-3)
 * @param cnf GPIO configuration (0-3)
 */
void stm32f103_gpio_configure_pin_raw(uint32_t gpio_base, uint8_t pin, uint8_t mode, uint8_t cnf);

/**
 * @brief Configure GPIO pin with structured parameters
 * @param config GPIO configuration structure
 * @return 0 on success, negative on error
 */
int stm32f103_gpio_configure_pin(const stm32f103_gpio_config_t *config);

/**
 * @brief Configure multiple GPIO pins with same settings
 * @param port GPIO port
 * @param pin_mask Bitmask of pins to configure (bit 0 = pin 0, etc.)
 * @param mode GPIO mode
 * @param cnf GPIO configuration
 * @return 0 on success, negative on error
 */
int stm32f103_gpio_configure_pins(stm32f103_gpio_port_t port, uint16_t pin_mask,
                                  stm32f103_gpio_mode_t mode, stm32f103_gpio_cnf_t cnf);

/*==============================================================================
 * GPIO Pin Control
 *============================================================================*/

/**
 * @brief Set GPIO pin high
 * @param port GPIO port
 * @param pin GPIO pin
 */
void stm32f103_gpio_set_pin(stm32f103_gpio_port_t port, stm32f103_gpio_pin_t pin);

/**
 * @brief Set GPIO pin low
 * @param port GPIO port
 * @param pin GPIO pin
 */
void stm32f103_gpio_clear_pin(stm32f103_gpio_port_t port, stm32f103_gpio_pin_t pin);

/**
 * @brief Toggle GPIO pin
 * @param port GPIO port
 * @param pin GPIO pin
 */
void stm32f103_gpio_toggle_pin(stm32f103_gpio_port_t port, stm32f103_gpio_pin_t pin);

/**
 * @brief Write value to GPIO pin
 * @param port GPIO port
 * @param pin GPIO pin
 * @param value true for high, false for low
 */
void stm32f103_gpio_write_pin(stm32f103_gpio_port_t port, stm32f103_gpio_pin_t pin, bool value);

/**
 * @brief Read GPIO pin value
 * @param port GPIO port
 * @param pin GPIO pin
 * @return true if pin is high, false if low
 */
bool stm32f103_gpio_read_pin(stm32f103_gpio_port_t port, stm32f103_gpio_pin_t pin);

/*==============================================================================
 * GPIO Port Control
 *============================================================================*/

/**
 * @brief Write value to entire GPIO port
 * @param port GPIO port
 * @param value 16-bit value to write (bit 0 = pin 0, etc.)
 */
void stm32f103_gpio_write_port(stm32f103_gpio_port_t port, uint16_t value);

/**
 * @brief Read entire GPIO port value
 * @param port GPIO port
 * @return 16-bit port value (bit 0 = pin 0, etc.)
 */
uint16_t stm32f103_gpio_read_port(stm32f103_gpio_port_t port);

/*==============================================================================
 * Utility Functions
 *============================================================================*/

/**
 * @brief Validate GPIO port
 * @param port GPIO port to validate
 * @return true if valid, false otherwise
 */
bool stm32f103_gpio_is_valid_port(stm32f103_gpio_port_t port);

/**
 * @brief Validate GPIO pin
 * @param pin GPIO pin to validate
 * @return true if valid, false otherwise
 */
bool stm32f103_gpio_is_valid_pin(stm32f103_gpio_pin_t pin);

/**
 * @brief Get port name as string
 * @param port GPIO port
 * @return Port name string ("PORTA", "PORTB", etc.) or "INVALID"
 */
const char* stm32f103_gpio_get_port_name(stm32f103_gpio_port_t port);

/*==============================================================================
 * Pin Availability Functions
 *============================================================================*/

/**
 * @brief Pin structure for cleaner API
 */
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

#endif /* STM32F103_GPIO_H */