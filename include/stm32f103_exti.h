#ifndef STM32F103_EXTI_H
#define STM32F103_EXTI_H

#include <stdint.h>

// Base addresses from stm32f103_registers.h
#ifndef APB2PERIPH_BASE
#define APB2PERIPH_BASE       (0x40000000UL + 0x00010000UL)
#endif

// ------------------- AFIO (Alternate Function I/O) -------------------
#define AFIO_BASE         (APB2PERIPH_BASE + 0x0000UL)

// AFIO registers
#define AFIO_EXTICR1      (*(volatile uint32_t*)(AFIO_BASE + 0x08))
#define AFIO_EXTICR2      (*(volatile uint32_t*)(AFIO_BASE + 0x0C))
#define AFIO_EXTICR3      (*(volatile uint32_t*)(AFIO_BASE + 0x10))
#define AFIO_EXTICR4      (*(volatile uint32_t*)(AFIO_BASE + 0x14))

// ------------------- EXTI (External Interrupt) -------------------
#define EXTI_BASE         (APB2PERIPH_BASE + 0x0400UL)

// EXTI registers
#define EXTI_IMR          (*(volatile uint32_t*)(EXTI_BASE + 0x00))  // Interrupt mask register
#define EXTI_RTSR         (*(volatile uint32_t*)(EXTI_BASE + 0x08))  // Rising trigger selection register
#define EXTI_FTSR         (*(volatile uint32_t*)(EXTI_BASE + 0x0C))  // Falling trigger selection register
#define EXTI_PR           (*(volatile uint32_t*)(EXTI_BASE + 0x14))  // Pending register

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
    if ((int32_t)(IRQn) >= 0) { \
        nvic_iser[(IRQn) >> 5UL] = (1UL << ((uint32_t)(IRQn) & 0x1FUL)); \
    } \
} while(0)

#endif /* STM32F103_EXTI_H */