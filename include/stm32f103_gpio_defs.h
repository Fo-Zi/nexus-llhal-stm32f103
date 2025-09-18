/**
 * @file stm32f103_gpio_defs.h
 * @brief STM32F103 GPIO Type Definitions
 */

#ifndef STM32F103_GPIO_DEFS_H
#define STM32F103_GPIO_DEFS_H

#include <stdint.h>
#include <stdbool.h>

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

/**
 * @brief GPIO interrupt configuration
 */
typedef struct {
    stm32f103_gpio_port_t port;
    stm32f103_gpio_pin_t pin;
    uint8_t trigger;             // NHAL trigger type
    uint8_t priority;            // NVIC interrupt priority (0-15)
    bool enabled;                // Enable interrupt immediately
} stm32f103_gpio_interrupt_config_t;

#endif /* STM32F103_GPIO_DEFS_H */