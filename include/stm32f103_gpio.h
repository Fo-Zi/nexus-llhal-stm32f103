#ifndef STM32F103_GPIO_H
#define STM32F103_GPIO_H

#include <stdint.h>
#include <stdbool.h>
#include "stm32f103_registers.h"
#include "stm32f103_mcu.h"

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

/**
 * @brief GPIO port enumeration
 */
typedef enum {
    STM32F103_GPIO_PORT_A = 0,
    STM32F103_GPIO_PORT_B,
    STM32F103_GPIO_PORT_C,
    STM32F103_GPIO_PORT_D,
    STM32F103_GPIO_PORT_E
} stm32f103_gpio_port_t;

/**
 * @brief GPIO pin enumeration
 */
typedef enum {
    STM32F103_GPIO_PIN_0 = 0,
    STM32F103_GPIO_PIN_1,
    STM32F103_GPIO_PIN_2,
    STM32F103_GPIO_PIN_3,
    STM32F103_GPIO_PIN_4,
    STM32F103_GPIO_PIN_5,
    STM32F103_GPIO_PIN_6,
    STM32F103_GPIO_PIN_7,
    STM32F103_GPIO_PIN_8,
    STM32F103_GPIO_PIN_9,
    STM32F103_GPIO_PIN_10,
    STM32F103_GPIO_PIN_11,
    STM32F103_GPIO_PIN_12,
    STM32F103_GPIO_PIN_13,
    STM32F103_GPIO_PIN_14,
    STM32F103_GPIO_PIN_15
} stm32f103_gpio_pin_t;

/**
 * @brief GPIO mode enumeration
 */
typedef enum {
    STM32F103_GPIO_MODE_INPUT = 0,      /**< Input mode */
    STM32F103_GPIO_MODE_OUTPUT_10MHz,   /**< Output mode, max speed 10 MHz */
    STM32F103_GPIO_MODE_OUTPUT_2MHz,    /**< Output mode, max speed 2 MHz */
    STM32F103_GPIO_MODE_OUTPUT_50MHz    /**< Output mode, max speed 50 MHz */
} stm32f103_gpio_mode_t;

/**
 * @brief GPIO configuration enumeration
 */
typedef enum {
    /* Input configurations */
    STM32F103_GPIO_CNF_INPUT_ANALOG = 0,    /**< Analog input */
    STM32F103_GPIO_CNF_INPUT_FLOATING,      /**< Floating input */
    STM32F103_GPIO_CNF_INPUT_PULL,          /**< Input with pull-up/pull-down */
    
    /* Output configurations */
    STM32F103_GPIO_CNF_OUTPUT_PUSH_PULL = 0,    /**< General purpose output push-pull */
    STM32F103_GPIO_CNF_OUTPUT_OPEN_DRAIN,       /**< General purpose output open-drain */
    STM32F103_GPIO_CNF_ALTFN_PUSH_PULL,         /**< Alternate function output push-pull */
    STM32F103_GPIO_CNF_ALTFN_OPEN_DRAIN         /**< Alternate function output open-drain */
} stm32f103_gpio_cnf_t;

/**
 * @brief Complete GPIO pin configuration
 */
typedef struct {
    stm32f103_gpio_port_t port;     /**< GPIO port */
    stm32f103_gpio_pin_t pin;       /**< GPIO pin */
    stm32f103_gpio_mode_t mode;     /**< GPIO mode */
    stm32f103_gpio_cnf_t cnf;       /**< GPIO configuration */
} stm32f103_gpio_config_t;

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