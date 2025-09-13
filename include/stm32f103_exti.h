#ifndef STM32F103_EXTI_H
#define STM32F103_EXTI_H

#include <stdint.h>
#include "stm32f103_registers.h"

// ------------------- NVIC (Nested Vector Interrupt Controller) -------------------

// EXTI IRQ Numbers for STM32F103
typedef enum {
    EXTI0_IRQn         = 6,    // EXTI Line 0 interrupt
    EXTI1_IRQn         = 7,    // EXTI Line 1 interrupt
    EXTI2_IRQn         = 8,    // EXTI Line 2 interrupt
    EXTI3_IRQn         = 9,    // EXTI Line 3 interrupt
    EXTI4_IRQn         = 10,   // EXTI Line 4 interrupt
    EXTI9_5_IRQn       = 23,   // EXTI Lines 9:5 interrupts
    EXTI15_10_IRQn     = 40,   // EXTI Lines 15:10 interrupts
} IRQn_Type;

// NVIC Enable macro for STM32F103
#define NVIC_EnableIRQ(IRQn)  do { \
    volatile uint32_t *nvic_iser = (volatile uint32_t*)0xE000E100UL; \
    nvic_iser[(IRQn) >> 5UL] = (1UL << ((uint32_t)(IRQn) & 0x1FUL)); \
} while(0)

#endif /* STM32F103_EXTI_H */