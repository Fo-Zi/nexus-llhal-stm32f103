/**
 * @file stm32f103_mcu_defs.h
 * @brief STM32F103 MCU Package Type Definitions
 */

#ifndef STM32F103_MCU_DEFS_H
#define STM32F103_MCU_DEFS_H

/**
 * @brief STM32F103 package types
 */
typedef enum {
    STM32F103_PACKAGE_48PIN_LQFP,    // C8T6, C6T6 (Blue Pill)
    STM32F103_PACKAGE_64PIN_LQFP,    // R8T6, R6T6
    STM32F103_PACKAGE_100PIN_LQFP    // V8T6, V6T6
} stm32f103_package_t;

#endif /* STM32F103_MCU_DEFS_H */