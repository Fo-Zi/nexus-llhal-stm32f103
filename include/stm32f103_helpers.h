#ifndef STM32F103_HELPERS_H
#define STM32F103_HELPERS_H

#include <stdint.h>
#include <stdbool.h>

typedef enum {
    STM32F103_GPIO_PORT_A = 0,
    STM32F103_GPIO_PORT_B,
    STM32F103_GPIO_PORT_C,
    STM32F103_GPIO_PORT_D,
    STM32F103_GPIO_PORT_E
} stm32f103_gpio_port_t;

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
 * @brief STM32F103 system configuration structure
 * Platform Integration Layer provides this to configure the HAL
 */
typedef struct {
    uint32_t target_sysclk_hz;          /**< Target system clock frequency */
    bool enable_systick;                 /**< Enable SysTick for timing */
    uint32_t systick_freq_hz;            /**< SysTick frequency (typically 1000 for 1ms) */
} stm32f103_system_config_t;

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
 * @brief Microsecond delay using SysTick
 * @param microseconds Number of microseconds to delay
 */
void stm32f103_delay_us(uint32_t microseconds);

/**
 * @brief Millisecond delay using SysTick
 * @param milliseconds Number of milliseconds to delay
 */
void stm32f103_delay_ms(uint32_t milliseconds);

/**
 * @brief Initialize STM32F103 system with platform-specific configuration
 * @param config Platform configuration provided by Platform Integration Layer
 * @return 0 on success, non-zero on error
 */
int stm32f103_system_init(const stm32f103_system_config_t *config);

/**
 * @brief Initialize UART1 for debugging (PA9=TX, PA10=RX, 115200 baud)
 * @return 0 on success, non-zero on error
 */
int stm32f103_uart_debug_init(void);

/**
 * @brief Send a string via UART1 for debugging
 * @param str Null-terminated string to send
 */
void stm32f103_uart_debug_print(const char* str);

/**
 * @brief Send a string with a number via UART1 for debugging
 * @param str String prefix
 * @param num Number to print (decimal)
 */
void stm32f103_uart_debug_print_num(const char* str, uint32_t num);

#endif