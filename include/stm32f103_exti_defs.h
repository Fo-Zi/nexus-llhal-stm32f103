/**
 * @file stm32f103_exti_defs.h
 * @brief STM32F103 EXTI and NVIC Type Definitions
 */

#ifndef STM32F103_EXTI_DEFS_H
#define STM32F103_EXTI_DEFS_H

#include <stdint.h>

/**
 * @brief EXTI IRQ Numbers for STM32F103
 */
typedef enum {
    EXTI0_IRQn         = 6,    // EXTI Line 0 interrupt
    EXTI1_IRQn         = 7,    // EXTI Line 1 interrupt
    EXTI2_IRQn         = 8,    // EXTI Line 2 interrupt
    EXTI3_IRQn         = 9,    // EXTI Line 3 interrupt
    EXTI4_IRQn         = 10,   // EXTI Line 4 interrupt
    EXTI9_5_IRQn       = 23,   // EXTI Lines 9:5 interrupts
    EXTI15_10_IRQn     = 40,   // EXTI Lines 15:10 interrupts
} IRQn_Type;

#endif /* STM32F103_EXTI_DEFS_H */