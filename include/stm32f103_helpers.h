#ifndef STM32F103_HELPERS_H
#define STM32F103_HELPERS_H

#include <stdint.h>
#include <stdbool.h>
#include "stm32f103_registers.h"

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
 * @brief Clock source selection for STM32F103
 */
typedef enum {
    STM32F103_CLOCK_SOURCE_HSI = 0,     /**< Internal 8MHz RC oscillator */
    STM32F103_CLOCK_SOURCE_HSE,         /**< External crystal oscillator */
    STM32F103_CLOCK_SOURCE_PLL_HSI,     /**< PLL with HSI/2 as source */
    STM32F103_CLOCK_SOURCE_PLL_HSE      /**< PLL with HSE as source */
} stm32f103_clock_source_t;

/**
 * @brief PLL multiplication factor
 */
typedef enum {
    STM32F103_PLL_MUL_2 = 2,
    STM32F103_PLL_MUL_3 = 3,
    STM32F103_PLL_MUL_4 = 4,
    STM32F103_PLL_MUL_5 = 5,
    STM32F103_PLL_MUL_6 = 6,
    STM32F103_PLL_MUL_7 = 7,
    STM32F103_PLL_MUL_8 = 8,
    STM32F103_PLL_MUL_9 = 9,
    STM32F103_PLL_MUL_10 = 10,
    STM32F103_PLL_MUL_11 = 11,
    STM32F103_PLL_MUL_12 = 12,
    STM32F103_PLL_MUL_13 = 13,
    STM32F103_PLL_MUL_14 = 14,
    STM32F103_PLL_MUL_15 = 15,
    STM32F103_PLL_MUL_16 = 16
} stm32f103_pll_mul_t;

/**
 * @brief AHB prescaler values
 */
typedef enum {
    STM32F103_AHB_DIV_1 = 1,
    STM32F103_AHB_DIV_2 = 2,
    STM32F103_AHB_DIV_4 = 4,
    STM32F103_AHB_DIV_8 = 8,
    STM32F103_AHB_DIV_16 = 16,
    STM32F103_AHB_DIV_64 = 64,
    STM32F103_AHB_DIV_128 = 128,
    STM32F103_AHB_DIV_256 = 256,
    STM32F103_AHB_DIV_512 = 512
} stm32f103_ahb_prescaler_t;

/**
 * @brief APB prescaler values
 */
typedef enum {
    STM32F103_APB_DIV_1 = 1,
    STM32F103_APB_DIV_2 = 2,
    STM32F103_APB_DIV_4 = 4,
    STM32F103_APB_DIV_8 = 8,
    STM32F103_APB_DIV_16 = 16
} stm32f103_apb_prescaler_t;

/**
 * @brief ADC prescaler values
 */
typedef enum {
    STM32F103_ADC_DIV_2 = 2,
    STM32F103_ADC_DIV_4 = 4,
    STM32F103_ADC_DIV_6 = 6,
    STM32F103_ADC_DIV_8 = 8
} stm32f103_adc_prescaler_t;

/**
 * @brief Flash latency settings
 */
typedef enum {
    STM32F103_FLASH_LATENCY_0 = 0,      /**< 0 < SYSCLK <= 24 MHz */
    STM32F103_FLASH_LATENCY_1 = 1,      /**< 24 MHz < SYSCLK <= 48 MHz */
    STM32F103_FLASH_LATENCY_2 = 2       /**< 48 MHz < SYSCLK <= 72 MHz */
} stm32f103_flash_latency_t;

/**
 * @brief Comprehensive STM32F103 clock tree configuration
 */
typedef struct {
    /* Main clock source */
    stm32f103_clock_source_t clock_source;     /**< System clock source */
    uint32_t hse_freq_hz;                       /**< HSE frequency (0 = not used) */
    bool hse_bypass;                            /**< HSE bypass mode */

    /* PLL configuration (only used if clock_source is PLL_*) */
    stm32f103_pll_mul_t pll_mul;               /**< PLL multiplication factor */
    bool pll_hse_div2;                         /**< Divide HSE by 2 before PLL (only for PLL_HSE) */

    /* Bus prescalers */
    stm32f103_ahb_prescaler_t ahb_prescaler;   /**< AHB prescaler (HCLK = SYSCLK / prescaler) */
    stm32f103_apb_prescaler_t apb1_prescaler;  /**< APB1 prescaler (PCLK1 = HCLK / prescaler) */
    stm32f103_apb_prescaler_t apb2_prescaler;  /**< APB2 prescaler (PCLK2 = HCLK / prescaler) */
    stm32f103_adc_prescaler_t adc_prescaler;   /**< ADC prescaler (ADCCLK = PCLK2 / prescaler) */

    /* Flash configuration */
    stm32f103_flash_latency_t flash_latency;   /**< Flash wait states */
    bool flash_prefetch_enable;                /**< Enable flash prefetch */

    /* USB configuration */
    bool usb_prescaler_1_5;                    /**< true = PLL/1.5, false = PLL/1 for USB */

    /* System features */
    bool enable_css;                           /**< Enable Clock Security System */
    bool enable_systick;                       /**< Enable SysTick for timing */
    uint32_t systick_freq_hz;                  /**< SysTick frequency (0 = disabled) */
} stm32f103_clock_config_t;

/**
 * @brief Calculated clock frequencies after initialization
 */
typedef struct {
    uint32_t sysclk_hz;                        /**< System clock frequency */
    uint32_t hclk_hz;                          /**< AHB clock frequency */
    uint32_t pclk1_hz;                         /**< APB1 clock frequency */
    uint32_t pclk2_hz;                         /**< APB2 clock frequency */
    uint32_t adcclk_hz;                        /**< ADC clock frequency */
    uint32_t usbclk_hz;                        /**< USB clock frequency */
} stm32f103_clock_frequencies_t;

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
 * @brief Initialize STM32F103 clock system with comprehensive configuration
 * @param config Clock tree configuration
 * @param frequencies Output structure for calculated frequencies (can be NULL)
 * @return 0 on success, negative on error:
 *         -1: Invalid configuration
 *         -2: HSE startup timeout
 *         -3: PLL startup timeout
 *         -4: Clock switch timeout
 *         -5: Invalid frequency combination
 */
int stm32f103_clock_init(const stm32f103_clock_config_t *config,
                         stm32f103_clock_frequencies_t *frequencies);

int stm32f103_default_72mhz_clock_init();
void dwt_init(void);

/**
 * @brief Get default 72MHz configuration using HSE 8MHz crystal
 * @param config Output configuration structure
 */
void stm32f103_get_default_72mhz_config(stm32f103_clock_config_t *config);

/**
 * @brief Get default 48MHz configuration using HSI (no external crystal)
 * @param config Output configuration structure
 */
void stm32f103_get_default_48mhz_hsi_config(stm32f103_clock_config_t *config);

/**
 * @brief Validate clock configuration for feasibility
 * @param config Configuration to validate
 * @param expected_freq Output structure for expected frequencies (can be NULL)
 * @return 0 if valid, negative if invalid
 */
int stm32f103_validate_clock_config(const stm32f103_clock_config_t *config,
                                    stm32f103_clock_frequencies_t *expected_freq);

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

/**
 * @brief Deinitialize UART1 debug interface
 * Resets pins to default state and disables UART
 */
void stm32f103_uart_debug_deinit(void);

/**
 * @brief Get default 8MHz HSI configuration
 * @param config Output configuration structure
 */
void stm32f103_get_default_8mhz_hsi_config(stm32f103_clock_config_t *config);

/**
 * @brief Get current system clock frequency from hardware registers
 * @return System clock frequency in Hz
 */
uint32_t stm32f103_get_sysclk_hz(void);

/**
 * @brief Set the clock frequencies used by timing functions
 *
 * This is the ONLY way to update the internal clock frequency reference
 * used by delay functions. Provides fine-grain control for both HAL
 * implementers and consumers.
 *
 * @param frequencies Pointer to frequency structure to copy
 * @return 0 on success, -1 on invalid input
 */
int stm32f103_set_clock_frequencies(const stm32f103_clock_frequencies_t *frequencies);

/**
 * @brief Get the current clock frequencies used by timing functions
 *
 * Returns a copy of the internal frequency structure used by delay
 * functions. This may differ from hardware registers if frequencies
 * have been manually overridden.
 *
 * @param frequencies Output structure to fill
 * @return 0 on success, -1 on invalid input
 */
int stm32f103_get_clock_frequencies(stm32f103_clock_frequencies_t *frequencies);

/**
 * @brief Auto-detect current clock frequencies from hardware registers
 *
 * Reads hardware registers to determine actual running frequencies
 * and updates the internal frequency reference. More reliable than
 * depending on configuration calculations.
 *
 * @return 0 on success, negative on detection error
 */
int stm32f103_detect_clock_frequencies(void);

/*==============================================================================
 * Clock Status and Utility Functions
 *============================================================================*/

/**
 * @brief Check if PLL is ready and locked
 * @return true if PLL is ready, false otherwise
 */
bool stm32f103_is_pll_ready(void);

/**
 * @brief Check if HSE is ready and stable
 * @return true if HSE is ready, false otherwise
 */
bool stm32f103_is_hse_ready(void);

/**
 * @brief Check if HSI is ready and stable
 * @return true if HSI is ready, false otherwise
 */
bool stm32f103_is_hsi_ready(void);

/**
 * @brief Get the current active system clock source
 * @return Current clock source enumeration
 */
stm32f103_clock_source_t stm32f103_get_current_clock_source(void);

/*==============================================================================
 * Peripheral Management Utilities
 *============================================================================*/

/**
 * @brief Enable peripheral clock
 * @param rcc_register_offset Offset from RCC_BASE to the enable register
 * @param enable_bit Bit mask for the peripheral
 * @return 0 on success
 */
int stm32f103_enable_peripheral_clock(uint32_t rcc_register_offset, uint32_t enable_bit);

/**
 * @brief Disable peripheral clock
 * @param rcc_register_offset Offset from RCC_BASE to the enable register
 * @param enable_bit Bit mask for the peripheral
 * @return 0 on success
 */
int stm32f103_disable_peripheral_clock(uint32_t rcc_register_offset, uint32_t enable_bit);

/**
 * @brief Perform system reset
 * This function does not return
 */
void stm32f103_system_reset(void);

/**
 * @brief Calculate UART BRR value for given clock and baudrate
 * @param peripheral_clock_hz Peripheral clock frequency in Hz
 * @param baudrate Desired baudrate
 * @return BRR register value
 */
uint32_t stm32f103_calculate_uart_brr(uint32_t peripheral_clock_hz, uint32_t baudrate);

/**
 * @brief Configure individual GPIO pin
 * @param gpio_base GPIO port base address
 * @param pin Pin number (0-15)
 * @param mode GPIO mode (0-3)
 * @param cnf GPIO configuration (0-3)
 */
void stm32f103_configure_gpio_pin(uint32_t gpio_base, uint8_t pin, uint8_t mode, uint8_t cnf);

#endif
