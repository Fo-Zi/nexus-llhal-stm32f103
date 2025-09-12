// #ifndef STM32F103_REGISTERS_H
// #define STM32F103_REGISTERS_H

// #include <stdint.h>

// // STM32F103 Memory Map
// #define PERIPH_BASE           0x40000000UL
// #define APB2PERIPH_BASE       (PERIPH_BASE + 0x00010000UL)
// #define AHBPERIPH_BASE        (PERIPH_BASE + 0x00020000UL)

// // GPIO Base Addresses
// #define GPIOA_BASE            (APB2PERIPH_BASE + 0x0800UL)
// #define GPIOB_BASE            (APB2PERIPH_BASE + 0x0C00UL)
// #define GPIOC_BASE            (APB2PERIPH_BASE + 0x1000UL)
// #define GPIOD_BASE            (APB2PERIPH_BASE + 0x1400UL)
// #define GPIOE_BASE            (APB2PERIPH_BASE + 0x1800UL)

// // RCC and AFIO Base Addresses
// #define RCC_BASE              (AHBPERIPH_BASE + 0x1000UL)
// #define AFIO_BASE             (APB2PERIPH_BASE + 0x0000UL)

// // RCC Registers
// #define RCC_CR                (*(volatile uint32_t*)(RCC_BASE + 0x00))
// #define RCC_CFGR              (*(volatile uint32_t*)(RCC_BASE + 0x04))
// #define RCC_CIR               (*(volatile uint32_t*)(RCC_BASE + 0x08))
// #define RCC_APB2RSTR          (*(volatile uint32_t*)(RCC_BASE + 0x0C))
// #define RCC_APB1RSTR          (*(volatile uint32_t*)(RCC_BASE + 0x10))
// #define RCC_AHBENR            (*(volatile uint32_t*)(RCC_BASE + 0x14))
// #define RCC_APB2ENR           (*(volatile uint32_t*)(RCC_BASE + 0x18))
// #define RCC_APB1ENR           (*(volatile uint32_t*)(RCC_BASE + 0x1C))

// // RCC_CR bits
// #define RCC_CR_HSION          (1U << 0)   // HSI oscillator ON
// #define RCC_CR_HSIRDY         (1U << 1)   // HSI oscillator ready
// #define RCC_CR_HSEON          (1U << 16)  // HSE oscillator ON
// #define RCC_CR_HSERDY         (1U << 17)  // HSE oscillator ready
// #define RCC_CR_HSEBYP         (1U << 18)  // HSE oscillator bypassed
// #define RCC_CR_CSSON          (1U << 19)  // Clock security system enable
// #define RCC_CR_PLLON          (1U << 24)  // PLL enable
// #define RCC_CR_PLLRDY         (1U << 25)  // PLL ready

// // RCC_CFGR bits
// #define RCC_CFGR_SW           (0x3U << 0)  // System clock switch mask
// #define RCC_CFGR_SW_HSI       0x0         // HSI selected as system clock
// #define RCC_CFGR_SW_HSE       0x1         // HSE selected as system clock
// #define RCC_CFGR_SW_PLL       0x2         // PLL selected as system clock
// #define RCC_CFGR_SWS          (0x3U << 2)  // System clock switch status mask
// #define RCC_CFGR_SWS_HSI      (0x0 << 2)  // HSI used as system clock
// #define RCC_CFGR_SWS_HSE      (0x1 << 2)  // HSE used as system clock
// #define RCC_CFGR_SWS_PLL      (0x2 << 2)  // PLL used as system clock
// #define RCC_CFGR_HPRE_DIV1    (0x0 << 4)  // SYSCLK not divided
// #define RCC_CFGR_PPRE1_DIV2   (0x4 << 8)  // HCLK divided by 2 for APB1
// #define RCC_CFGR_PPRE2_DIV1   (0x0 << 11) // HCLK not divided for APB2
// #define RCC_CFGR_PLLSRC       (0x1U << 16) // PLL source mask
// #define RCC_CFGR_PLLSRC_HSI   (0x0 << 16) // HSI/2 as PLL input
// #define RCC_CFGR_PLLSRC_HSE   (0x1 << 16) // HSE as PLL input
// #define RCC_CFGR_PLLXTPRE     (0x1 << 17) // HSE divider for PLL entry
// #define RCC_CFGR_PLLMULL      (0xFU << 18) // PLL multiplication factor mask
// #define RCC_CFGR_PLLMUL2      (0x0 << 18) // PLL input * 2
// #define RCC_CFGR_PLLMUL3      (0x1 << 18) // PLL input * 3
// #define RCC_CFGR_PLLMUL4      (0x2 << 18) // PLL input * 4
// #define RCC_CFGR_PLLMUL5      (0x3 << 18) // PLL input * 5
// #define RCC_CFGR_PLLMUL6      (0x4 << 18) // PLL input * 6
// #define RCC_CFGR_PLLMUL7      (0x5 << 18) // PLL input * 7
// #define RCC_CFGR_PLLMUL8      (0x6 << 18) // PLL input * 8
// #define RCC_CFGR_PLLMUL9      (0x7 << 18) // PLL input * 9
// #define RCC_CFGR_PLLMUL10     (0x8 << 18) // PLL input * 10
// #define RCC_CFGR_PLLMUL11     (0x9 << 18) // PLL input * 11
// #define RCC_CFGR_PLLMUL12     (0xA << 18) // PLL input * 12
// #define RCC_CFGR_PLLMUL13     (0xB << 18) // PLL input * 13
// #define RCC_CFGR_PLLMUL14     (0xC << 18) // PLL input * 14
// #define RCC_CFGR_PLLMUL15     (0xD << 18) // PLL input * 15
// #define RCC_CFGR_PLLMUL16     (0xE << 18) // PLL input * 16

// // RCC_APB2ENR bits
// #define RCC_APB2ENR_IOPAEN    (1U << 2)
// #define RCC_APB2ENR_IOPBEN    (1U << 3)
// #define RCC_APB2ENR_IOPCEN    (1U << 4)
// #define RCC_APB2ENR_IOPDEN    (1U << 5)
// #define RCC_APB2ENR_IOPEEN    (1U << 6)
// #define RCC_APB2ENR_AFIOEN    (1U << 0)

// // GPIO Register Offsets
// #define GPIO_CRL_OFFSET       0x00
// #define GPIO_CRH_OFFSET       0x04
// #define GPIO_IDR_OFFSET       0x08
// #define GPIO_ODR_OFFSET       0x0C
// #define GPIO_BSRR_OFFSET      0x10
// #define GPIO_BRR_OFFSET       0x14
// #define GPIO_LCKR_OFFSET      0x18

// // GPIO Register Macros
// #define GPIO_CRL(base)        (*(volatile uint32_t*)((base) + GPIO_CRL_OFFSET))
// #define GPIO_CRH(base)        (*(volatile uint32_t*)((base) + GPIO_CRH_OFFSET))
// #define GPIO_IDR(base)        (*(volatile uint32_t*)((base) + GPIO_IDR_OFFSET))
// #define GPIO_ODR(base)        (*(volatile uint32_t*)((base) + GPIO_ODR_OFFSET))
// #define GPIO_BSRR(base)       (*(volatile uint32_t*)((base) + GPIO_BSRR_OFFSET))
// #define GPIO_BRR(base)        (*(volatile uint32_t*)((base) + GPIO_BRR_OFFSET))

// // GPIO Configuration Values
// #define GPIO_MODE_INPUT                 0x0
// #define GPIO_MODE_OUTPUT_10M            0x1
// #define GPIO_MODE_OUTPUT_2M             0x2
// #define GPIO_MODE_OUTPUT_50M            0x3

// #define GPIO_CNF_INPUT_ANALOG           0x0
// #define GPIO_CNF_INPUT_FLOATING         0x1
// #define GPIO_CNF_INPUT_PULLUP           0x2
// #define GPIO_CNF_OUTPUT_PUSHPULL        0x0
// #define GPIO_CNF_OUTPUT_OPENDRAIN       0x1
// #define GPIO_CNF_OUTPUT_ALT_PUSHPULL    0x2
// #define GPIO_CNF_OUTPUT_ALT_OPENDRAIN   0x3

// // SysTick Registers (for timing functions)
// #define SCS_BASE            0xE000E000UL
// #define SYSTICK_BASE        (SCS_BASE + 0x0010UL)

// #define SYSTICK_CSR         (*(volatile uint32_t*)(SYSTICK_BASE + 0x00))
// #define SYSTICK_RVR         (*(volatile uint32_t*)(SYSTICK_BASE + 0x04))
// #define SYSTICK_CVR         (*(volatile uint32_t*)(SYSTICK_BASE + 0x08))

// #define SYSTICK_CSR_ENABLE     (1U << 0)
// #define SYSTICK_CSR_TICKINT    (1U << 1)
// #define SYSTICK_CSR_CLKSOURCE  (1U << 2)
// #define SYSTICK_CSR_COUNTFLAG  (1U << 16)

// // UART1 Registers (for debugging)
// #define UART1_BASE            (APB2PERIPH_BASE + 0x3800UL)
// #define UART1_SR              (*(volatile uint32_t*)(UART1_BASE + 0x00))
// #define UART1_DR              (*(volatile uint32_t*)(UART1_BASE + 0x04))
// #define UART1_BRR             (*(volatile uint32_t*)(UART1_BASE + 0x08))
// #define UART1_CR1             (*(volatile uint32_t*)(UART1_BASE + 0x0C))
// #define UART1_CR2             (*(volatile uint32_t*)(UART1_BASE + 0x10))
// #define UART1_CR3             (*(volatile uint32_t*)(UART1_BASE + 0x14))

// // UART Status Register bits
// #define UART_SR_TXE           (1U << 7)  // Transmit data register empty
// #define UART_SR_TC            (1U << 6)  // Transmission complete
// #define UART_SR_RXNE          (1U << 5)  // Read data register not empty

// // UART Control Register 1 bits
// #define UART_CR1_UE           (1U << 13) // UART enable
// #define UART_CR1_TE           (1U << 3)  // Transmitter enable
// #define UART_CR1_RE           (1U << 2)  // Receiver enable

// // RCC APB2ENR UART1 enable bit
// #define RCC_APB2ENR_USART1EN  (1U << 14)

// // Flash registers for wait states
// #define FLASH_BASE            (0x40022000UL)
// #define FLASH_ACR             (*(volatile uint32_t*)(FLASH_BASE + 0x00))

// // Flash Access Control Register bits
// #define FLASH_ACR_PRFTBE      (1U << 4)   // Prefetch Buffer Enable
// #define FLASH_ACR_LATENCY     (0x7U << 0) // LATENCY bits
// #define FLASH_ACR_LATENCY_0   (0x0U << 0) // Zero wait state
// #define FLASH_ACR_LATENCY_1   (0x1U << 0) // One wait state
// #define FLASH_ACR_LATENCY_2   (0x2U << 0) // Two wait states

// // HSE timeout constant
// #define HSE_STARTUP_TIMEOUT   (0x500)

// // Clock frequency constants
// #define STM32F103_HSI_FREQ_HZ (8000000UL)  // 8MHz HSI frequency

// #endif


#ifndef STM32F103_REGISTERS_H
#define STM32F103_REGISTERS_H

#include <stdint.h>

/* --- Base addresses --- */
#define PERIPH_BASE           ((uint32_t)0x40000000)
#define APB1PERIPH_BASE       PERIPH_BASE
#define APB2PERIPH_BASE       (PERIPH_BASE + 0x00010000)
#define AHBPERIPH_BASE        (PERIPH_BASE + 0x00020000)

#define RCC_BASE              (AHBPERIPH_BASE + 0x00001000)
#define FLASH_R_BASE          (AHBPERIPH_BASE + 0x00002000)
#define SYSTICK_BASE          ((uint32_t)0xE000E010)

/* --- Register definitions --- */
#define RCC_CR                (*(volatile uint32_t *)(RCC_BASE + 0x00))
#define RCC_CFGR              (*(volatile uint32_t *)(RCC_BASE + 0x04))
#define RCC_CIR               (*(volatile uint32_t *)(RCC_BASE + 0x08))

#define FLASH_ACR             (*(volatile uint32_t *)(FLASH_R_BASE + 0x00))

#define SYSTICK_CSR           (*(volatile uint32_t *)(SYSTICK_BASE + 0x00))
#define SYSTICK_RVR           (*(volatile uint32_t *)(SYSTICK_BASE + 0x04))
#define SYSTICK_CVR           (*(volatile uint32_t *)(SYSTICK_BASE + 0x08))
#define SYSTICK_CALIB         (*(volatile uint32_t *)(SYSTICK_BASE + 0x0C))

/* --- RCC_CR bits --- */
#define RCC_CR_HSION          (1U << 0)
#define RCC_CR_HSIRDY         (1U << 1)
#define RCC_CR_HSEON          (1U << 16)
#define RCC_CR_HSERDY         (1U << 17)
#define RCC_CR_HSEBYP         (1U << 18)
#define RCC_CR_CSSON          (1U << 19)
#define RCC_CR_PLLON          (1U << 24)
#define RCC_CR_PLLRDY         (1U << 25)

/* --- RCC_CFGR bits --- */
#define RCC_CFGR_SW_HSI       (0x0U << 0)
#define RCC_CFGR_SW_HSE       (0x1U << 0)
#define RCC_CFGR_SW_PLL       (0x2U << 0)

#define RCC_CFGR_SWS_HSI      (0x0U << 2)
#define RCC_CFGR_SWS_HSE      (0x1U << 2)
#define RCC_CFGR_SWS_PLL      (0x2U << 2)

#define RCC_CFGR_HPRE_DIV1    (0x0U << 4)
#define RCC_CFGR_PPRE1_DIV1   (0x0U << 8)
#define RCC_CFGR_PPRE1_DIV2   (0x4U << 8)
#define RCC_CFGR_PPRE2_DIV1   (0x0U << 11)

#define RCC_CFGR_PLLSRC_HSI   (0x0U << 16)
#define RCC_CFGR_PLLSRC_HSE   (0x1U << 16)
#define RCC_CFGR_PLLXTPRE_HSE (0x1U << 17)
#define RCC_CFGR_PLLMULL6     (0x14U << 18)
#define RCC_CFGR_PLLMULL9     (0x1CU << 18)

/* --- RCC_CIR bits (clear flags) --- */
#define RCC_CIR_LSIRDYC       (1U << 0)
#define RCC_CIR_LSERDYC       (1U << 1)
#define RCC_CIR_HSIRDYC       (1U << 2)
#define RCC_CIR_HSERDYC       (1U << 3)
#define RCC_CIR_PLLRDYC       (1U << 4)
#define RCC_CIR_CSSC          (1U << 7)

/* --- FLASH_ACR bits --- */
#define FLASH_ACR_LATENCY_0   (0x0U << 0)
#define FLASH_ACR_LATENCY_1   (0x1U << 0)
#define FLASH_ACR_LATENCY_2   (0x2U << 0)
#define FLASH_ACR_PRFTBE      (1U << 4)

/* --- SysTick bits --- */
#define SYSTICK_CSR_ENABLE    (1U << 0)
#define SYSTICK_CSR_TICKINT   (1U << 1)
#define SYSTICK_CSR_CLKSOURCE (1U << 2)
#define SYSTICK_CSR_COUNTFLAG (1U << 16)

// ------------------- Clock / RCC -------------------

// HSE startup timeout
#define HSE_STARTUP_TIMEOUT       0x0500U

// RCC_CFGR register bits and masks
#define RCC_CFGR_SW               0x00000003U
#define RCC_CFGR_HPRE             0x000000F0U
#define RCC_CFGR_PPRE1            0x00000700U
#define RCC_CFGR_PPRE2            0x00003800U
#define RCC_CFGR_ADCPRE           0x0000C000U
#define RCC_CFGR_ADCPRE_DIV6      0x00008000U
#define RCC_CFGR_PLLSRC           0x00010000U
#define RCC_CFGR_PLLXTPRE         0x00020000U
#define RCC_CFGR_PLLMULL           0x003C0000U
#define RCC_CFGR_PLLMUL9          0x001C0000U
#define RCC_CFGR_PLLMUL12         0x00280000U
#define RCC_CFGR_USBPRE           0x00400000U

// ------------------- DWT -------------------
#define DEMCR       (*(volatile uint32_t*)0xE000EDFC)
#define DEMCR_TRCENA    (1 << 24)

#define DWT_CTRL    (*(volatile uint32_t*)0xE0001000)
#define DWT_CYCCNT  (*(volatile uint32_t*)0xE0001004)
#define DWT_CTRL_CYCCNTENA (1 << 0)

// ------------------- Flash -------------------
#define FLASH_ACR_LATENCY         0x00000003U
// #define FLASH_ACR_LATENCY_0       0x00000000U

// Internal high-speed oscillator frequency
#define STM32F103_HSI_FREQ_HZ    8000000U  // 8 MHz


// RCC APB2 enable register bits
#define RCC_APB2ENR      (*(volatile uint32_t*)0x40021018U)
#define RCC_APB2ENR_AFIOEN  (1U << 0)  // AFIO clock enable
#define RCC_APB2ENR_IOPAEN  (1U << 2)  // GPIOA clock enable
#define RCC_APB2ENR_IOPBEN  (1U << 3)  // GPIOB clock enable
#define RCC_APB2ENR_IOPCEN  (1U << 4)  // GPIOC clock enable
#define RCC_APB2ENR_IOPDEN  (1U << 5)  // GPIOD clock enable
#define RCC_APB2ENR_IOPEEN  (1U << 6)  // GPIOE clock enable

// GPIO modes and configuration
#define GPIO_MODE_INPUT        0x0
#define GPIO_MODE_OUTPUT_10M   0x1
#define GPIO_MODE_OUTPUT_2M    0x2
#define GPIO_MODE_OUTPUT_50M   0x3

#define GPIO_CNF_INPUT_ANALOG       0x0
#define GPIO_CNF_INPUT_FLOATING     0x1
#define GPIO_CNF_INPUT_PULLUP       0x2

#define GPIO_CNF_OUTPUT_PUSHPULL    0x0
#define GPIO_CNF_OUTPUT_OPENDRAIN   0x1
#define GPIO_CNF_OUTPUT_AF_PUSHPULL 0x2
#define GPIO_CNF_OUTPUT_AF_OD       0x3

// GPIO registers access (example macros for base pointer)
#define GPIO_CRL(base)   (*(volatile uint32_t*)((base) + 0x00))
#define GPIO_CRH(base)   (*(volatile uint32_t*)((base) + 0x04))
#define GPIO_IDR(base)   (*(volatile uint32_t*)((base) + 0x08))
#define GPIO_ODR(base)   (*(volatile uint32_t*)((base) + 0x0C))
#define GPIO_BSRR(base)  (*(volatile uint32_t*)((base) + 0x10))
#define GPIO_BRR(base)   (*(volatile uint32_t*)((base) + 0x14))
#define GPIO_LCKR(base)  (*(volatile uint32_t*)((base) + 0x18))


#endif /* STM32F103_REGISTERS_H */
