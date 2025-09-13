#ifndef STM32F103_MCU_H
#define STM32F103_MCU_H

/**
 * @file stm32f103_mcu.h
 * @brief STM32F103 MCU Hardware Package Definitions
 * 
 * This module defines MCU package types for different STM32F103 variants.
 */

/*==============================================================================
 * MCU Package Types
 *============================================================================*/

/**
 * @brief STM32F103 package types
 */
typedef enum {
    STM32F103_PACKAGE_48PIN_LQFP,    // C8T6, C6T6 (Blue Pill)
    STM32F103_PACKAGE_64PIN_LQFP,    // R8T6, R6T6
    STM32F103_PACKAGE_100PIN_LQFP    // V8T6, V6T6
} stm32f103_package_t;

#endif /* STM32F103_MCU_H */