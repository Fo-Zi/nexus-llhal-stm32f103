/**
 * @file stm32f103_clock_defs.h
 * @brief STM32F103 Clock System Type Definitions
 */

#ifndef STM32F103_CLOCK_DEFS_H
#define STM32F103_CLOCK_DEFS_H

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief Clock source selection
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
    STM32F103_AHB_DIV_1   = 0,
    STM32F103_AHB_DIV_2   = 8,
    STM32F103_AHB_DIV_4   = 9,
    STM32F103_AHB_DIV_8   = 10,
    STM32F103_AHB_DIV_16  = 11,
    STM32F103_AHB_DIV_64  = 12,
    STM32F103_AHB_DIV_128 = 13,
    STM32F103_AHB_DIV_256 = 14,
    STM32F103_AHB_DIV_512 = 15
} stm32f103_ahb_prescaler_t;

/**
 * @brief APB prescaler values  
 */
typedef enum {
    STM32F103_APB_DIV_1  = 0,
    STM32F103_APB_DIV_2  = 4,
    STM32F103_APB_DIV_4  = 5,
    STM32F103_APB_DIV_8  = 6,
    STM32F103_APB_DIV_16 = 7
} stm32f103_apb_prescaler_t;

/**
 * @brief ADC prescaler values
 */
typedef enum {
    STM32F103_ADC_DIV_2 = 0,
    STM32F103_ADC_DIV_4 = 1,
    STM32F103_ADC_DIV_6 = 2,
    STM32F103_ADC_DIV_8 = 3
} stm32f103_adc_prescaler_t;

/**
 * @brief Flash latency settings
 */
typedef enum {
    STM32F103_FLASH_LATENCY_0 = 0,  /**< 0 wait states for SYSCLK <= 24MHz */
    STM32F103_FLASH_LATENCY_1 = 1,  /**< 1 wait state for 24MHz < SYSCLK <= 48MHz */
    STM32F103_FLASH_LATENCY_2 = 2   /**< 2 wait states for 48MHz < SYSCLK <= 72MHz */
} stm32f103_flash_latency_t;

/**
 * @brief Clock configuration parameters
 */
typedef struct {
    stm32f103_clock_source_t source;
    uint32_t hse_frequency_hz;           /**< External crystal frequency (if using HSE) */
    stm32f103_pll_mul_t pll_mul;        /**< PLL multiplication factor (if using PLL) */
    stm32f103_ahb_prescaler_t ahb_prescaler;
    stm32f103_apb_prescaler_t apb1_prescaler;
    stm32f103_apb_prescaler_t apb2_prescaler;
    stm32f103_adc_prescaler_t adc_prescaler;
    stm32f103_flash_latency_t flash_latency;
    bool enable_css;                     /**< Enable Clock Security System */
    bool enable_hse_bypass;              /**< Enable HSE bypass mode */
    
    // Timeout settings for clock stabilization
    uint32_t hse_timeout_ms;
    uint32_t pll_timeout_ms;
} stm32f103_clock_config_t;

/**
 * @brief System clock frequencies (read-only status)
 */
typedef struct {
    uint32_t sysclk_hz;      /**< System clock frequency */
    uint32_t hclk_hz;        /**< AHB clock frequency */  
    uint32_t pclk1_hz;       /**< APB1 clock frequency */
    uint32_t pclk2_hz;       /**< APB2 clock frequency */
    uint32_t adcclk_hz;      /**< ADC clock frequency */
    stm32f103_clock_source_t active_source;
    bool css_enabled;        /**< Clock Security System status */
    bool pll_ready;          /**< PLL ready status */
    bool hse_ready;          /**< HSE ready status */
    bool hsi_ready;          /**< HSI ready status */
} stm32f103_clock_status_t;

#endif /* STM32F103_CLOCK_DEFS_H */