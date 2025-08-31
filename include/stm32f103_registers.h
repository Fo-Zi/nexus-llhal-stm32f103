#ifndef STM32F103_REGISTERS_H
#define STM32F103_REGISTERS_H

#include <stdint.h>

// STM32F103 Memory Map
#define PERIPH_BASE           0x40000000UL
#define APB2PERIPH_BASE       (PERIPH_BASE + 0x00010000UL)
#define AHBPERIPH_BASE        (PERIPH_BASE + 0x00020000UL)

// GPIO Base Addresses
#define GPIOA_BASE            (APB2PERIPH_BASE + 0x0800UL)
#define GPIOB_BASE            (APB2PERIPH_BASE + 0x0C00UL)
#define GPIOC_BASE            (APB2PERIPH_BASE + 0x1000UL)
#define GPIOD_BASE            (APB2PERIPH_BASE + 0x1400UL)
#define GPIOE_BASE            (APB2PERIPH_BASE + 0x1800UL)

// RCC and AFIO Base Addresses
#define RCC_BASE              (AHBPERIPH_BASE + 0x1000UL)
#define AFIO_BASE             (APB2PERIPH_BASE + 0x0000UL)

// RCC Registers
#define RCC_CR                (*(volatile uint32_t*)(RCC_BASE + 0x00))
#define RCC_CFGR              (*(volatile uint32_t*)(RCC_BASE + 0x04))
#define RCC_APB2ENR           (*(volatile uint32_t*)(RCC_BASE + 0x18))

// RCC_APB2ENR bits
#define RCC_APB2ENR_IOPAEN    (1U << 2)
#define RCC_APB2ENR_IOPBEN    (1U << 3)
#define RCC_APB2ENR_IOPCEN    (1U << 4)
#define RCC_APB2ENR_IOPDEN    (1U << 5)
#define RCC_APB2ENR_IOPEEN    (1U << 6)
#define RCC_APB2ENR_AFIOEN    (1U << 0)

// GPIO Register Offsets
#define GPIO_CRL_OFFSET       0x00
#define GPIO_CRH_OFFSET       0x04
#define GPIO_IDR_OFFSET       0x08
#define GPIO_ODR_OFFSET       0x0C
#define GPIO_BSRR_OFFSET      0x10
#define GPIO_BRR_OFFSET       0x14
#define GPIO_LCKR_OFFSET      0x18

// GPIO Register Macros
#define GPIO_CRL(base)        (*(volatile uint32_t*)((base) + GPIO_CRL_OFFSET))
#define GPIO_CRH(base)        (*(volatile uint32_t*)((base) + GPIO_CRH_OFFSET))
#define GPIO_IDR(base)        (*(volatile uint32_t*)((base) + GPIO_IDR_OFFSET))
#define GPIO_ODR(base)        (*(volatile uint32_t*)((base) + GPIO_ODR_OFFSET))
#define GPIO_BSRR(base)       (*(volatile uint32_t*)((base) + GPIO_BSRR_OFFSET))
#define GPIO_BRR(base)        (*(volatile uint32_t*)((base) + GPIO_BRR_OFFSET))

// GPIO Configuration Values
#define GPIO_MODE_INPUT                 0x0
#define GPIO_MODE_OUTPUT_10M            0x1
#define GPIO_MODE_OUTPUT_2M             0x2
#define GPIO_MODE_OUTPUT_50M            0x3

#define GPIO_CNF_INPUT_ANALOG           0x0
#define GPIO_CNF_INPUT_FLOATING         0x1
#define GPIO_CNF_INPUT_PULLUP           0x2
#define GPIO_CNF_OUTPUT_PUSHPULL        0x0
#define GPIO_CNF_OUTPUT_OPENDRAIN       0x1
#define GPIO_CNF_OUTPUT_ALT_PUSHPULL    0x2
#define GPIO_CNF_OUTPUT_ALT_OPENDRAIN   0x3

// SysTick Registers (for timing functions)
#define SCS_BASE            0xE000E000UL
#define SYSTICK_BASE        (SCS_BASE + 0x0010UL)

#define SYSTICK_CSR         (*(volatile uint32_t*)(SYSTICK_BASE + 0x00))
#define SYSTICK_RVR         (*(volatile uint32_t*)(SYSTICK_BASE + 0x04))
#define SYSTICK_CVR         (*(volatile uint32_t*)(SYSTICK_BASE + 0x08))

#define SYSTICK_CSR_ENABLE     (1U << 0)
#define SYSTICK_CSR_TICKINT    (1U << 1)
#define SYSTICK_CSR_CLKSOURCE  (1U << 2)
#define SYSTICK_CSR_COUNTFLAG  (1U << 16)

// UART1 Registers (for debugging)
#define UART1_BASE            (APB2PERIPH_BASE + 0x3800UL)
#define UART1_SR              (*(volatile uint32_t*)(UART1_BASE + 0x00))
#define UART1_DR              (*(volatile uint32_t*)(UART1_BASE + 0x04))
#define UART1_BRR             (*(volatile uint32_t*)(UART1_BASE + 0x08))
#define UART1_CR1             (*(volatile uint32_t*)(UART1_BASE + 0x0C))
#define UART1_CR2             (*(volatile uint32_t*)(UART1_BASE + 0x10))
#define UART1_CR3             (*(volatile uint32_t*)(UART1_BASE + 0x14))

// UART Status Register bits
#define UART_SR_TXE           (1U << 7)  // Transmit data register empty
#define UART_SR_TC            (1U << 6)  // Transmission complete
#define UART_SR_RXNE          (1U << 5)  // Read data register not empty

// UART Control Register 1 bits
#define UART_CR1_UE           (1U << 13) // UART enable
#define UART_CR1_TE           (1U << 3)  // Transmitter enable
#define UART_CR1_RE           (1U << 2)  // Receiver enable

// RCC APB2ENR UART1 enable bit
#define RCC_APB2ENR_USART1EN  (1U << 14)

#endif