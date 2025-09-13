#ifndef STM32F103_REGISTERS_H
#define STM32F103_REGISTERS_H

#include <stdint.h>

/*==============================================================================
 * Memory Map and Base Addresses
 *============================================================================*/
#define FLASH_BASE            0x08000000UL  /*!< FLASH base address in the alias region */
#define SRAM_BASE             0x20000000UL  /*!< SRAM base address in the alias region */
#define PERIPH_BASE           0x40000000UL  /*!< Peripheral base address in the alias region */

#define APB1PERIPH_BASE       PERIPH_BASE
#define APB2PERIPH_BASE       (PERIPH_BASE + 0x00010000UL)
#define AHBPERIPH_BASE        (PERIPH_BASE + 0x00020000UL)

/*==============================================================================
 * APB1 Peripherals
 *============================================================================*/
#define TIM2_BASE             (APB1PERIPH_BASE + 0x0000UL)
#define TIM3_BASE             (APB1PERIPH_BASE + 0x0400UL)
#define TIM4_BASE             (APB1PERIPH_BASE + 0x0800UL)
#define TIM5_BASE             (APB1PERIPH_BASE + 0x0C00UL)
#define TIM6_BASE             (APB1PERIPH_BASE + 0x1000UL)
#define TIM7_BASE             (APB1PERIPH_BASE + 0x1400UL)
#define RTC_BASE              (APB1PERIPH_BASE + 0x2800UL)
#define WWDG_BASE             (APB1PERIPH_BASE + 0x2C00UL)
#define IWDG_BASE             (APB1PERIPH_BASE + 0x3000UL)
#define SPI2_BASE             (APB1PERIPH_BASE + 0x3800UL)
#define SPI3_BASE             (APB1PERIPH_BASE + 0x3C00UL)
#define USART2_BASE           (APB1PERIPH_BASE + 0x4400UL)
#define USART3_BASE           (APB1PERIPH_BASE + 0x4800UL)
#define UART4_BASE            (APB1PERIPH_BASE + 0x4C00UL)
#define UART5_BASE            (APB1PERIPH_BASE + 0x5000UL)
#define I2C1_BASE             (APB1PERIPH_BASE + 0x5400UL)
#define I2C2_BASE             (APB1PERIPH_BASE + 0x5800UL)
#define USB_BASE              (APB1PERIPH_BASE + 0x5C00UL)
#define USB_PMAADDR           (APB1PERIPH_BASE + 0x6000UL)
#define CAN1_BASE             (APB1PERIPH_BASE + 0x6400UL)
#define BKP_BASE              (APB1PERIPH_BASE + 0x6C00UL)
#define PWR_BASE              (APB1PERIPH_BASE + 0x7000UL)
#define DAC_BASE              (APB1PERIPH_BASE + 0x7400UL)

/*==============================================================================
 * APB2 Peripherals
 *============================================================================*/
#define AFIO_BASE             (APB2PERIPH_BASE + 0x0000UL)
#define EXTI_BASE             (APB2PERIPH_BASE + 0x0400UL)
#define GPIOA_BASE            (APB2PERIPH_BASE + 0x0800UL)
#define GPIOB_BASE            (APB2PERIPH_BASE + 0x0C00UL)
#define GPIOC_BASE            (APB2PERIPH_BASE + 0x1000UL)
#define GPIOD_BASE            (APB2PERIPH_BASE + 0x1400UL)
#define GPIOE_BASE            (APB2PERIPH_BASE + 0x1800UL)
#define GPIOF_BASE            (APB2PERIPH_BASE + 0x1C00UL)
#define GPIOG_BASE            (APB2PERIPH_BASE + 0x2000UL)
#define ADC1_BASE             (APB2PERIPH_BASE + 0x2400UL)
#define ADC2_BASE             (APB2PERIPH_BASE + 0x2800UL)
#define TIM1_BASE             (APB2PERIPH_BASE + 0x2C00UL)
#define SPI1_BASE             (APB2PERIPH_BASE + 0x3000UL)
#define TIM8_BASE             (APB2PERIPH_BASE + 0x3400UL)
#define USART1_BASE           (APB2PERIPH_BASE + 0x3800UL)
#define ADC3_BASE             (APB2PERIPH_BASE + 0x3C00UL)

/*==============================================================================
 * AHB Peripherals
 *============================================================================*/
#define DMA1_BASE             (AHBPERIPH_BASE + 0x0000UL)
#define DMA1_Channel1_BASE    (AHBPERIPH_BASE + 0x0008UL)
#define DMA1_Channel2_BASE    (AHBPERIPH_BASE + 0x001CUL)
#define DMA1_Channel3_BASE    (AHBPERIPH_BASE + 0x0030UL)
#define DMA1_Channel4_BASE    (AHBPERIPH_BASE + 0x0044UL)
#define DMA1_Channel5_BASE    (AHBPERIPH_BASE + 0x0058UL)
#define DMA1_Channel6_BASE    (AHBPERIPH_BASE + 0x006CUL)
#define DMA1_Channel7_BASE    (AHBPERIPH_BASE + 0x0080UL)
#define DMA2_BASE             (AHBPERIPH_BASE + 0x0400UL)
#define DMA2_Channel1_BASE    (AHBPERIPH_BASE + 0x0408UL)
#define DMA2_Channel2_BASE    (AHBPERIPH_BASE + 0x041CUL)
#define DMA2_Channel3_BASE    (AHBPERIPH_BASE + 0x0430UL)
#define DMA2_Channel4_BASE    (AHBPERIPH_BASE + 0x0444UL)
#define DMA2_Channel5_BASE    (AHBPERIPH_BASE + 0x0458UL)
#define RCC_BASE              (AHBPERIPH_BASE + 0x1000UL)
#define CRC_BASE              (AHBPERIPH_BASE + 0x3000UL)
#define FLASH_R_BASE          (AHBPERIPH_BASE + 0x2000UL)
#define FSMC_Bank1_R_BASE     (AHBPERIPH_BASE + 0x4000UL)
#define FSMC_Bank1E_R_BASE    (AHBPERIPH_BASE + 0x4104UL)
#define FSMC_Bank2_R_BASE     (AHBPERIPH_BASE + 0x4060UL)
#define FSMC_Bank3_R_BASE     (AHBPERIPH_BASE + 0x4080UL)
#define FSMC_Bank4_R_BASE     (AHBPERIPH_BASE + 0x40A0UL)
#define SDIO_BASE             (PERIPH_BASE + 0x18000UL)

/*==============================================================================
 * System Control Space
 *============================================================================*/
#define SCS_BASE              0xE000E000UL        /*!< System Control Space Base Address */
#define ITM_BASE              0xE0000000UL        /*!< ITM Base Address */
#define DWT_BASE              0xE0001000UL        /*!< DWT Base Address */
#define TPI_BASE              0xE0040000UL        /*!< TPI Base Address */
#define CoreDebug_BASE        0xE000EDF0UL        /*!< Core Debug Base Address */
#define SysTick_BASE          (SCS_BASE +  0x0010UL)  /*!< SysTick Base Address */
#define NVIC_BASE             (SCS_BASE +  0x0100UL)  /*!< NVIC Base Address */
#define SCB_BASE              (SCS_BASE +  0x0D00UL)  /*!< System Control Block Base Address */

/*==============================================================================
 * RCC (Reset and Clock Control) Registers
 *============================================================================*/
#define RCC_CR                (*(volatile uint32_t *)(RCC_BASE + 0x00))  /*!< RCC clock control register */
#define RCC_CFGR              (*(volatile uint32_t *)(RCC_BASE + 0x04))  /*!< RCC clock configuration register */
#define RCC_CIR               (*(volatile uint32_t *)(RCC_BASE + 0x08))  /*!< RCC clock interrupt register */
#define RCC_APB2RSTR          (*(volatile uint32_t *)(RCC_BASE + 0x0C))  /*!< RCC APB2 peripheral reset register */
#define RCC_APB1RSTR          (*(volatile uint32_t *)(RCC_BASE + 0x10))  /*!< RCC APB1 peripheral reset register */
#define RCC_AHBENR            (*(volatile uint32_t *)(RCC_BASE + 0x14))  /*!< RCC AHB peripheral clock register */
#define RCC_APB2ENR           (*(volatile uint32_t *)(RCC_BASE + 0x18))  /*!< RCC APB2 peripheral clock enable register */
#define RCC_APB1ENR           (*(volatile uint32_t *)(RCC_BASE + 0x1C))  /*!< RCC APB1 peripheral clock enable register */
#define RCC_BDCR              (*(volatile uint32_t *)(RCC_BASE + 0x20))  /*!< RCC Backup domain control register */
#define RCC_CSR               (*(volatile uint32_t *)(RCC_BASE + 0x24))  /*!< RCC clock control & status register */

/*==============================================================================
 * FLASH Registers
 *============================================================================*/
#define FLASH_ACR             (*(volatile uint32_t *)(FLASH_R_BASE + 0x00))  /*!< FLASH access control register */
#define FLASH_KEYR            (*(volatile uint32_t *)(FLASH_R_BASE + 0x04))  /*!< FLASH key register */
#define FLASH_OPTKEYR         (*(volatile uint32_t *)(FLASH_R_BASE + 0x08))  /*!< FLASH option key register */
#define FLASH_SR              (*(volatile uint32_t *)(FLASH_R_BASE + 0x0C))  /*!< FLASH status register */
#define FLASH_CR              (*(volatile uint32_t *)(FLASH_R_BASE + 0x10))  /*!< FLASH control register */
#define FLASH_AR              (*(volatile uint32_t *)(FLASH_R_BASE + 0x14))  /*!< FLASH address register */
#define FLASH_OBR             (*(volatile uint32_t *)(FLASH_R_BASE + 0x1C))  /*!< FLASH Option Bytes register */
#define FLASH_WRPR            (*(volatile uint32_t *)(FLASH_R_BASE + 0x20))  /*!< FLASH Write register */

/*==============================================================================
 * GPIO (General Purpose I/O) Registers
 *============================================================================*/
#define GPIO_CRL(base)        (*(volatile uint32_t*)((base) + 0x00))  /*!< GPIO port configuration register low */
#define GPIO_CRH(base)        (*(volatile uint32_t*)((base) + 0x04))  /*!< GPIO port configuration register high */
#define GPIO_IDR(base)        (*(volatile uint32_t*)((base) + 0x08))  /*!< GPIO port input data register */
#define GPIO_ODR(base)        (*(volatile uint32_t*)((base) + 0x0C))  /*!< GPIO port output data register */
#define GPIO_BSRR(base)       (*(volatile uint32_t*)((base) + 0x10))  /*!< GPIO port bit set/reset register */
#define GPIO_BRR(base)        (*(volatile uint32_t*)((base) + 0x14))  /*!< GPIO port bit reset register */
#define GPIO_LCKR(base)       (*(volatile uint32_t*)((base) + 0x18))  /*!< GPIO port configuration lock register */

/*==============================================================================
 * AFIO (Alternate Function I/O) Registers
 *============================================================================*/
#define AFIO_EVCR             (*(volatile uint32_t*)(AFIO_BASE + 0x00))  /*!< AFIO event control register */
#define AFIO_MAPR             (*(volatile uint32_t*)(AFIO_BASE + 0x04))  /*!< AFIO alternate function remap register */
#define AFIO_EXTICR1          (*(volatile uint32_t*)(AFIO_BASE + 0x08))  /*!< AFIO external interrupt config register 1 */
#define AFIO_EXTICR2          (*(volatile uint32_t*)(AFIO_BASE + 0x0C))  /*!< AFIO external interrupt config register 2 */
#define AFIO_EXTICR3          (*(volatile uint32_t*)(AFIO_BASE + 0x10))  /*!< AFIO external interrupt config register 3 */
#define AFIO_EXTICR4          (*(volatile uint32_t*)(AFIO_BASE + 0x14))  /*!< AFIO external interrupt config register 4 */
#define AFIO_MAPR2            (*(volatile uint32_t*)(AFIO_BASE + 0x1C))  /*!< AFIO alternate function remap register 2 */

/*==============================================================================
 * EXTI (External Interrupt) Registers
 *============================================================================*/
#define EXTI_IMR              (*(volatile uint32_t*)(EXTI_BASE + 0x00))  /*!< EXTI Interrupt mask register */
#define EXTI_EMR              (*(volatile uint32_t*)(EXTI_BASE + 0x04))  /*!< EXTI Event mask register */
#define EXTI_RTSR             (*(volatile uint32_t*)(EXTI_BASE + 0x08))  /*!< EXTI Rising trigger selection register */
#define EXTI_FTSR             (*(volatile uint32_t*)(EXTI_BASE + 0x0C))  /*!< EXTI Falling trigger selection register */
#define EXTI_SWIER            (*(volatile uint32_t*)(EXTI_BASE + 0x10))  /*!< EXTI Software interrupt event register */
#define EXTI_PR               (*(volatile uint32_t*)(EXTI_BASE + 0x14))  /*!< EXTI Pending register */

/*==============================================================================
 * USART (Universal Synchronous/Asynchronous Receiver Transmitter) Registers
 *============================================================================*/
#define USART_SR(base)        (*(volatile uint32_t*)((base) + 0x00))  /*!< USART Status register */
#define USART_DR(base)        (*(volatile uint32_t*)((base) + 0x04))  /*!< USART Data register */
#define USART_BRR(base)       (*(volatile uint32_t*)((base) + 0x08))  /*!< USART Baud rate register */
#define USART_CR1(base)       (*(volatile uint32_t*)((base) + 0x0C))  /*!< USART Control register 1 */
#define USART_CR2(base)       (*(volatile uint32_t*)((base) + 0x10))  /*!< USART Control register 2 */
#define USART_CR3(base)       (*(volatile uint32_t*)((base) + 0x14))  /*!< USART Control register 3 */
#define USART_GTPR(base)      (*(volatile uint32_t*)((base) + 0x18))  /*!< USART Guard time and prescaler register */

/* USART1 specific registers */
#define USART1_SR             USART_SR(USART1_BASE)
#define USART1_DR             USART_DR(USART1_BASE)
#define USART1_BRR            USART_BRR(USART1_BASE)
#define USART1_CR1            USART_CR1(USART1_BASE)
#define USART1_CR2            USART_CR2(USART1_BASE)
#define USART1_CR3            USART_CR3(USART1_BASE)
#define USART1_GTPR           USART_GTPR(USART1_BASE)

/* USART2 specific registers */
#define USART2_SR             USART_SR(USART2_BASE)
#define USART2_DR             USART_DR(USART2_BASE)
#define USART2_BRR            USART_BRR(USART2_BASE)
#define USART2_CR1            USART_CR1(USART2_BASE)
#define USART2_CR2            USART_CR2(USART2_BASE)
#define USART2_CR3            USART_CR3(USART2_BASE)
#define USART2_GTPR           USART_GTPR(USART2_BASE)

/* USART3 specific registers */
#define USART3_SR             USART_SR(USART3_BASE)
#define USART3_DR             USART_DR(USART3_BASE)
#define USART3_BRR            USART_BRR(USART3_BASE)
#define USART3_CR1            USART_CR1(USART3_BASE)
#define USART3_CR2            USART_CR2(USART3_BASE)
#define USART3_CR3            USART_CR3(USART3_BASE)
#define USART3_GTPR           USART_GTPR(USART3_BASE)

/*==============================================================================
 * SysTick Registers
 *============================================================================*/
#define SYSTICK_CSR           (*(volatile uint32_t *)(SysTick_BASE + 0x00))  /*!< SysTick Control and Status Register */
#define SYSTICK_RVR           (*(volatile uint32_t *)(SysTick_BASE + 0x04))  /*!< SysTick Reload Value Register */
#define SYSTICK_CVR           (*(volatile uint32_t *)(SysTick_BASE + 0x08))  /*!< SysTick Current Value Register */
#define SYSTICK_CALIB         (*(volatile uint32_t *)(SysTick_BASE + 0x0C))  /*!< SysTick Calibration Register */

/*==============================================================================
 * DWT (Data Watchpoint and Trace) Registers
 *============================================================================*/
#define DWT_CTRL              (*(volatile uint32_t*)(DWT_BASE + 0x00))   /*!< DWT Control Register */
#define DWT_CYCCNT            (*(volatile uint32_t*)(DWT_BASE + 0x04))   /*!< DWT Current PC Sampler Cycle Count Register */

/*==============================================================================
 * SCB (System Control Block) Registers
 *============================================================================*/
#define SCB_DEMCR             (*(volatile uint32_t*)(CoreDebug_BASE + 0x0C))  /*!< Debug Exception and Monitor Control Register */

/*==============================================================================
 * RCC Register Bit Definitions
 *============================================================================*/

/* RCC_CR register bits */
#define RCC_CR_HSION                         (1U << 0)   /*!< Internal High Speed clock enable */
#define RCC_CR_HSIRDY                        (1U << 1)   /*!< Internal High Speed clock ready flag */
#define RCC_CR_HSITRIM_Pos                   3
#define RCC_CR_HSITRIM_Msk                   (0x1F << RCC_CR_HSITRIM_Pos)
#define RCC_CR_HSICAL_Pos                    8
#define RCC_CR_HSICAL_Msk                    (0xFF << RCC_CR_HSICAL_Pos)
#define RCC_CR_HSEON                         (1U << 16)  /*!< External High Speed clock enable */
#define RCC_CR_HSERDY                        (1U << 17)  /*!< External High Speed clock ready flag */
#define RCC_CR_HSEBYP                        (1U << 18)  /*!< External High Speed clock Bypass */
#define RCC_CR_CSSON                         (1U << 19)  /*!< Clock Security System enable */
#define RCC_CR_PLLON                         (1U << 24)  /*!< PLL enable */
#define RCC_CR_PLLRDY                        (1U << 25)  /*!< PLL clock ready flag */

/* RCC_CFGR register bits */
#define RCC_CFGR_SW_Pos                      0
#define RCC_CFGR_SW_Msk                      (0x3 << RCC_CFGR_SW_Pos)
#define RCC_CFGR_SW_HSI                      (0x0 << RCC_CFGR_SW_Pos)  /*!< HSI selected as system clock */
#define RCC_CFGR_SW_HSE                      (0x1 << RCC_CFGR_SW_Pos)  /*!< HSE selected as system clock */
#define RCC_CFGR_SW_PLL                      (0x2 << RCC_CFGR_SW_Pos)  /*!< PLL selected as system clock */

#define RCC_CFGR_SWS_Pos                     2
#define RCC_CFGR_SWS_Msk                     (0x3 << RCC_CFGR_SWS_Pos)
#define RCC_CFGR_SWS_HSI                     (0x0 << RCC_CFGR_SWS_Pos) /*!< HSI used as system clock */
#define RCC_CFGR_SWS_HSE                     (0x1 << RCC_CFGR_SWS_Pos) /*!< HSE used as system clock */
#define RCC_CFGR_SWS_PLL                     (0x2 << RCC_CFGR_SWS_Pos) /*!< PLL used as system clock */

#define RCC_CFGR_HPRE_Pos                    4
#define RCC_CFGR_HPRE_Msk                    (0xF << RCC_CFGR_HPRE_Pos)
#define RCC_CFGR_HPRE_DIV1                   (0x0 << RCC_CFGR_HPRE_Pos) /*!< SYSCLK not divided */
#define RCC_CFGR_HPRE_DIV2                   (0x8 << RCC_CFGR_HPRE_Pos) /*!< SYSCLK divided by 2 */
#define RCC_CFGR_HPRE_DIV4                   (0x9 << RCC_CFGR_HPRE_Pos) /*!< SYSCLK divided by 4 */
#define RCC_CFGR_HPRE_DIV8                   (0xA << RCC_CFGR_HPRE_Pos) /*!< SYSCLK divided by 8 */
#define RCC_CFGR_HPRE_DIV16                  (0xB << RCC_CFGR_HPRE_Pos) /*!< SYSCLK divided by 16 */
#define RCC_CFGR_HPRE_DIV64                  (0xC << RCC_CFGR_HPRE_Pos) /*!< SYSCLK divided by 64 */
#define RCC_CFGR_HPRE_DIV128                 (0xD << RCC_CFGR_HPRE_Pos) /*!< SYSCLK divided by 128 */
#define RCC_CFGR_HPRE_DIV256                 (0xE << RCC_CFGR_HPRE_Pos) /*!< SYSCLK divided by 256 */
#define RCC_CFGR_HPRE_DIV512                 (0xF << RCC_CFGR_HPRE_Pos) /*!< SYSCLK divided by 512 */

#define RCC_CFGR_PPRE1_Pos                   8
#define RCC_CFGR_PPRE1_Msk                   (0x7 << RCC_CFGR_PPRE1_Pos)
#define RCC_CFGR_PPRE1_DIV1                  (0x0 << RCC_CFGR_PPRE1_Pos) /*!< HCLK not divided */
#define RCC_CFGR_PPRE1_DIV2                  (0x4 << RCC_CFGR_PPRE1_Pos) /*!< HCLK divided by 2 */
#define RCC_CFGR_PPRE1_DIV4                  (0x5 << RCC_CFGR_PPRE1_Pos) /*!< HCLK divided by 4 */
#define RCC_CFGR_PPRE1_DIV8                  (0x6 << RCC_CFGR_PPRE1_Pos) /*!< HCLK divided by 8 */
#define RCC_CFGR_PPRE1_DIV16                 (0x7 << RCC_CFGR_PPRE1_Pos) /*!< HCLK divided by 16 */

#define RCC_CFGR_PPRE2_Pos                   11
#define RCC_CFGR_PPRE2_Msk                   (0x7 << RCC_CFGR_PPRE2_Pos)
#define RCC_CFGR_PPRE2_DIV1                  (0x0 << RCC_CFGR_PPRE2_Pos) /*!< HCLK not divided */
#define RCC_CFGR_PPRE2_DIV2                  (0x4 << RCC_CFGR_PPRE2_Pos) /*!< HCLK divided by 2 */
#define RCC_CFGR_PPRE2_DIV4                  (0x5 << RCC_CFGR_PPRE2_Pos) /*!< HCLK divided by 4 */
#define RCC_CFGR_PPRE2_DIV8                  (0x6 << RCC_CFGR_PPRE2_Pos) /*!< HCLK divided by 8 */
#define RCC_CFGR_PPRE2_DIV16                 (0x7 << RCC_CFGR_PPRE2_Pos) /*!< HCLK divided by 16 */

#define RCC_CFGR_ADCPRE_Pos                  14
#define RCC_CFGR_ADCPRE_Msk                  (0x3 << RCC_CFGR_ADCPRE_Pos)
#define RCC_CFGR_ADCPRE_DIV2                 (0x0 << RCC_CFGR_ADCPRE_Pos) /*!< PCLK2 divided by 2 */
#define RCC_CFGR_ADCPRE_DIV4                 (0x1 << RCC_CFGR_ADCPRE_Pos) /*!< PCLK2 divided by 4 */
#define RCC_CFGR_ADCPRE_DIV6                 (0x2 << RCC_CFGR_ADCPRE_Pos) /*!< PCLK2 divided by 6 */
#define RCC_CFGR_ADCPRE_DIV8                 (0x3 << RCC_CFGR_ADCPRE_Pos) /*!< PCLK2 divided by 8 */

#define RCC_CFGR_PLLSRC                      (1U << 16) /*!< PLL entry clock source */
#define RCC_CFGR_PLLSRC_HSI                  (0U << 16) /*!< HSI/2 selected as PLL input clock */
#define RCC_CFGR_PLLSRC_HSE                  (1U << 16) /*!< HSE selected as PLL input clock */

#define RCC_CFGR_PLLXTPRE                    (1U << 17) /*!< HSE divider for PLL entry */
#define RCC_CFGR_PLLXTPRE_HSE                (0U << 17) /*!< HSE clock not divided for PLL entry */
#define RCC_CFGR_PLLXTPRE_HSE_DIV2           (1U << 17) /*!< HSE clock divided by 2 for PLL entry */

#define RCC_CFGR_PLLMULL_Pos                 18
#define RCC_CFGR_PLLMULL_Msk                 (0xF << RCC_CFGR_PLLMULL_Pos)
#define RCC_CFGR_PLLMULL2                    (0x0 << RCC_CFGR_PLLMULL_Pos) /*!< PLL input clock*2 */
#define RCC_CFGR_PLLMULL3                    (0x1 << RCC_CFGR_PLLMULL_Pos) /*!< PLL input clock*3 */
#define RCC_CFGR_PLLMULL4                    (0x2 << RCC_CFGR_PLLMULL_Pos) /*!< PLL input clock*4 */
#define RCC_CFGR_PLLMULL5                    (0x3 << RCC_CFGR_PLLMULL_Pos) /*!< PLL input clock*5 */
#define RCC_CFGR_PLLMULL6                    (0x4 << RCC_CFGR_PLLMULL_Pos) /*!< PLL input clock*6 */
#define RCC_CFGR_PLLMULL7                    (0x5 << RCC_CFGR_PLLMULL_Pos) /*!< PLL input clock*7 */
#define RCC_CFGR_PLLMULL8                    (0x6 << RCC_CFGR_PLLMULL_Pos) /*!< PLL input clock*8 */
#define RCC_CFGR_PLLMULL9                    (0x7 << RCC_CFGR_PLLMULL_Pos) /*!< PLL input clock*9 */
#define RCC_CFGR_PLLMULL10                   (0x8 << RCC_CFGR_PLLMULL_Pos) /*!< PLL input clock10 */
#define RCC_CFGR_PLLMULL11                   (0x9 << RCC_CFGR_PLLMULL_Pos) /*!< PLL input clock*11 */
#define RCC_CFGR_PLLMULL12                   (0xA << RCC_CFGR_PLLMULL_Pos) /*!< PLL input clock*12 */
#define RCC_CFGR_PLLMULL13                   (0xB << RCC_CFGR_PLLMULL_Pos) /*!< PLL input clock*13 */
#define RCC_CFGR_PLLMULL14                   (0xC << RCC_CFGR_PLLMULL_Pos) /*!< PLL input clock*14 */
#define RCC_CFGR_PLLMULL15                   (0xD << RCC_CFGR_PLLMULL_Pos) /*!< PLL input clock*15 */
#define RCC_CFGR_PLLMULL16                   (0xE << RCC_CFGR_PLLMULL_Pos) /*!< PLL input clock*16 */

#define RCC_CFGR_USBPRE                      (1U << 22) /*!< USB prescaler */

/* RCC_CIR register bits */
#define RCC_CIR_LSIRDYF                      (1U << 0)  /*!< LSI Ready Interrupt flag */
#define RCC_CIR_LSERDYF                      (1U << 1)  /*!< LSE Ready Interrupt flag */
#define RCC_CIR_HSIRDYF                      (1U << 2)  /*!< HSI Ready Interrupt flag */
#define RCC_CIR_HSERDYF                      (1U << 3)  /*!< HSE Ready Interrupt flag */
#define RCC_CIR_PLLRDYF                      (1U << 4)  /*!< PLL Ready Interrupt flag */
#define RCC_CIR_CSSF                         (1U << 7)  /*!< Clock Security System Interrupt flag */
#define RCC_CIR_LSIRDYIE                     (1U << 8)  /*!< LSI Ready Interrupt Enable */
#define RCC_CIR_LSERDYIE                     (1U << 9)  /*!< LSE Ready Interrupt Enable */
#define RCC_CIR_HSIRDYIE                     (1U << 10) /*!< HSI Ready Interrupt Enable */
#define RCC_CIR_HSERDYIE                     (1U << 11) /*!< HSE Ready Interrupt Enable */
#define RCC_CIR_PLLRDYIE                     (1U << 12) /*!< PLL Ready Interrupt Enable */
#define RCC_CIR_LSIRDYC                      (1U << 16) /*!< LSI Ready Interrupt Clear */
#define RCC_CIR_LSERDYC                      (1U << 17) /*!< LSE Ready Interrupt Clear */
#define RCC_CIR_HSIRDYC                      (1U << 18) /*!< HSI Ready Interrupt Clear */
#define RCC_CIR_HSERDYC                      (1U << 19) /*!< HSE Ready Interrupt Clear */
#define RCC_CIR_PLLRDYC                      (1U << 20) /*!< PLL Ready Interrupt Clear */
#define RCC_CIR_CSSC                         (1U << 23) /*!< Clock Security System Interrupt Clear */

/* RCC_APB2ENR register bits */
#define RCC_APB2ENR_AFIOEN                   (1U << 0)  /*!< Alternate Function I/O clock enable */
#define RCC_APB2ENR_IOPAEN                   (1U << 2)  /*!< I/O port A clock enable */
#define RCC_APB2ENR_IOPBEN                   (1U << 3)  /*!< I/O port B clock enable */
#define RCC_APB2ENR_IOPCEN                   (1U << 4)  /*!< I/O port C clock enable */
#define RCC_APB2ENR_IOPDEN                   (1U << 5)  /*!< I/O port D clock enable */
#define RCC_APB2ENR_IOPEEN                   (1U << 6)  /*!< I/O port E clock enable */
#define RCC_APB2ENR_IOPFEN                   (1U << 7)  /*!< I/O port F clock enable */
#define RCC_APB2ENR_IOPGEN                   (1U << 8)  /*!< I/O port G clock enable */
#define RCC_APB2ENR_ADC1EN                   (1U << 9)  /*!< ADC 1 interface clock enable */
#define RCC_APB2ENR_ADC2EN                   (1U << 10) /*!< ADC 2 interface clock enable */
#define RCC_APB2ENR_TIM1EN                   (1U << 11) /*!< TIM1 Timer clock enable */
#define RCC_APB2ENR_SPI1EN                   (1U << 12) /*!< SPI 1 clock enable */
#define RCC_APB2ENR_TIM8EN                   (1U << 13) /*!< TIM8 Timer clock enable */
#define RCC_APB2ENR_USART1EN                 (1U << 14) /*!< USART1 clock enable */
#define RCC_APB2ENR_ADC3EN                   (1U << 15) /*!< ADC3 interface clock enable */

/* RCC_APB1ENR register bits */
#define RCC_APB1ENR_TIM2EN                   (1U << 0)  /*!< TIM2 timer clock enable */
#define RCC_APB1ENR_TIM3EN                   (1U << 1)  /*!< TIM3 timer clock enable */
#define RCC_APB1ENR_TIM4EN                   (1U << 2)  /*!< TIM4 timer clock enable */
#define RCC_APB1ENR_TIM5EN                   (1U << 3)  /*!< TIM5 timer clock enable */
#define RCC_APB1ENR_TIM6EN                   (1U << 4)  /*!< TIM6 timer clock enable */
#define RCC_APB1ENR_TIM7EN                   (1U << 5)  /*!< TIM7 timer clock enable */
#define RCC_APB1ENR_WWDGEN                   (1U << 11) /*!< Window watchdog clock enable */
#define RCC_APB1ENR_SPI2EN                   (1U << 14) /*!< SPI 2 clock enable */
#define RCC_APB1ENR_SPI3EN                   (1U << 15) /*!< SPI 3 clock enable */
#define RCC_APB1ENR_USART2EN                 (1U << 17) /*!< USART 2 clock enable */
#define RCC_APB1ENR_USART3EN                 (1U << 18) /*!< USART 3 clock enable */
#define RCC_APB1ENR_UART4EN                  (1U << 19) /*!< UART 4 clock enable */
#define RCC_APB1ENR_UART5EN                  (1U << 20) /*!< UART 5 clock enable */
#define RCC_APB1ENR_I2C1EN                   (1U << 21) /*!< I2C 1 clock enable */
#define RCC_APB1ENR_I2C2EN                   (1U << 22) /*!< I2C 2 clock enable */
#define RCC_APB1ENR_USBEN                    (1U << 23) /*!< USB device clock enable */
#define RCC_APB1ENR_CANEN                    (1U << 25) /*!< CAN1 clock enable */
#define RCC_APB1ENR_BKPEN                    (1U << 27) /*!< Backup interface clock enable */
#define RCC_APB1ENR_PWREN                    (1U << 28) /*!< Power interface clock enable */
#define RCC_APB1ENR_DACEN                    (1U << 29) /*!< DAC interface clock enable */

/*==============================================================================
 * FLASH Register Bit Definitions
 *============================================================================*/

/* FLASH_ACR register bits */
#define FLASH_ACR_LATENCY_Pos                0
#define FLASH_ACR_LATENCY_Msk                (0x7 << FLASH_ACR_LATENCY_Pos)
#define FLASH_ACR_LATENCY_0                  (0x0 << FLASH_ACR_LATENCY_Pos) /*!< FLASH Zero Latency cycle */
#define FLASH_ACR_LATENCY_1                  (0x1 << FLASH_ACR_LATENCY_Pos) /*!< FLASH One Latency cycle */
#define FLASH_ACR_LATENCY_2                  (0x2 << FLASH_ACR_LATENCY_Pos) /*!< FLASH Two Latency cycles */
#define FLASH_ACR_HLFCYA                     (1U << 3)  /*!< Flash Half Cycle Access Enable */
#define FLASH_ACR_PRFTBE                     (1U << 4)  /*!< Prefetch Buffer Enable */
#define FLASH_ACR_PRFTBS                     (1U << 5)  /*!< Prefetch Buffer Status */

/*==============================================================================
 * USART Register Bit Definitions
 *============================================================================*/

/* USART_SR register bits */
#define USART_SR_PE                          (1U << 0)  /*!< Parity Error */
#define USART_SR_FE                          (1U << 1)  /*!< Framing Error */
#define USART_SR_NE                          (1U << 2)  /*!< Noise Error Flag */
#define USART_SR_ORE                         (1U << 3)  /*!< OverRun Error */
#define USART_SR_IDLE                        (1U << 4)  /*!< IDLE line detected */
#define USART_SR_RXNE                        (1U << 5)  /*!< Read Data Register Not Empty */
#define USART_SR_TC                          (1U << 6)  /*!< Transmission Complete */
#define USART_SR_TXE                         (1U << 7)  /*!< Transmit Data Register Empty */
#define USART_SR_LBD                         (1U << 8)  /*!< LIN Break Detection Flag */
#define USART_SR_CTS                         (1U << 9)  /*!< CTS Flag */

/* USART_CR1 register bits */
#define USART_CR1_SBK                        (1U << 0)  /*!< Send Break */
#define USART_CR1_RWU                        (1U << 1)  /*!< Receiver wakeup */
#define USART_CR1_RE                         (1U << 2)  /*!< Receiver Enable */
#define USART_CR1_TE                         (1U << 3)  /*!< Transmitter Enable */
#define USART_CR1_IDLEIE                     (1U << 4)  /*!< IDLE Interrupt Enable */
#define USART_CR1_RXNEIE                     (1U << 5)  /*!< RXNE Interrupt Enable */
#define USART_CR1_TCIE                       (1U << 6)  /*!< Transmission Complete Interrupt Enable */
#define USART_CR1_TXEIE                      (1U << 7)  /*!< TXE Interrupt Enable */
#define USART_CR1_PEIE                       (1U << 8)  /*!< PE Interrupt Enable */
#define USART_CR1_PS                         (1U << 9)  /*!< Parity Selection */
#define USART_CR1_PCE                        (1U << 10) /*!< Parity Control Enable */
#define USART_CR1_WAKE                       (1U << 11) /*!< Wakeup method */
#define USART_CR1_M                          (1U << 12) /*!< Word length */
#define USART_CR1_UE                         (1U << 13) /*!< USART Enable */

/* USART_CR2 register bits */
#define USART_CR2_ADD_Pos                    0
#define USART_CR2_ADD_Msk                    (0xF << USART_CR2_ADD_Pos)
#define USART_CR2_LBDL                       (1U << 5)  /*!< LIN Break Detection Length */
#define USART_CR2_LBDIE                      (1U << 6)  /*!< LIN Break Detection Interrupt Enable */
#define USART_CR2_LBCL                       (1U << 8)  /*!< Last Bit Clock pulse */
#define USART_CR2_CPHA                       (1U << 9)  /*!< Clock Phase */
#define USART_CR2_CPOL                       (1U << 10) /*!< Clock Polarity */
#define USART_CR2_CLKEN                      (1U << 11) /*!< Clock Enable */
#define USART_CR2_STOP_Pos                   12
#define USART_CR2_STOP_Msk                   (0x3 << USART_CR2_STOP_Pos)
#define USART_CR2_STOP_1                     (0x0 << USART_CR2_STOP_Pos) /*!< 1 Stop bit */
#define USART_CR2_STOP_0_5                   (0x1 << USART_CR2_STOP_Pos) /*!< 0.5 Stop bit */
#define USART_CR2_STOP_2                     (0x2 << USART_CR2_STOP_Pos) /*!< 2 Stop bits */
#define USART_CR2_STOP_1_5                   (0x3 << USART_CR2_STOP_Pos) /*!< 1.5 Stop bit */
#define USART_CR2_LINEN                      (1U << 14) /*!< LIN mode enable */

/* USART_CR3 register bits */
#define USART_CR3_EIE                        (1U << 0)  /*!< Error Interrupt Enable */
#define USART_CR3_IREN                       (1U << 1)  /*!< IrDA mode Enable */
#define USART_CR3_IRLP                       (1U << 2)  /*!< IrDA Low-Power */
#define USART_CR3_HDSEL                      (1U << 3)  /*!< Half-Duplex Selection */
#define USART_CR3_NACK                       (1U << 4)  /*!< Smartcard NACK enable */
#define USART_CR3_SCEN                       (1U << 5)  /*!< Smartcard mode enable */
#define USART_CR3_DMAR                       (1U << 6)  /*!< DMA Enable Receiver */
#define USART_CR3_DMAT                       (1U << 7)  /*!< DMA Enable Transmitter */
#define USART_CR3_RTSE                       (1U << 8)  /*!< RTS Enable */
#define USART_CR3_CTSE                       (1U << 9)  /*!< CTS Enable */
#define USART_CR3_CTSIE                      (1U << 10) /*!< CTS Interrupt Enable */

/*==============================================================================
 * GPIO Register Bit Definitions
 *============================================================================*/

/* GPIO modes */
#define GPIO_MODE_INPUT                      0x0  /*!< Input mode (reset state) */
#define GPIO_MODE_OUTPUT_10M                 0x1  /*!< Output mode, max speed 10 MHz */
#define GPIO_MODE_OUTPUT_2M                  0x2  /*!< Output mode, max speed 2 MHz */
#define GPIO_MODE_OUTPUT_50M                 0x3  /*!< Output mode, max speed 50 MHz */

/* GPIO input configuration */
#define GPIO_CNF_INPUT_ANALOG                0x0  /*!< Analog mode */
#define GPIO_CNF_INPUT_FLOATING              0x1  /*!< Floating input (reset state) */
#define GPIO_CNF_INPUT_PULLUP                0x2  /*!< Input with pull-up / pull-down */

/* GPIO output configuration */
#define GPIO_CNF_OUTPUT_PUSHPULL             0x0  /*!< General purpose output push-pull */
#define GPIO_CNF_OUTPUT_OPENDRAIN            0x1  /*!< General purpose output Open-drain */
#define GPIO_CNF_OUTPUT_AF_PUSHPULL          0x2  /*!< Alternate function output Push-pull */
#define GPIO_CNF_OUTPUT_AF_OD                0x3  /*!< Alternate function output Open-drain */

/*==============================================================================
 * SysTick Register Bit Definitions
 *============================================================================*/

/* SysTick_CSR register bits */
#define SYSTICK_CSR_ENABLE                   (1U << 0)  /*!< Counter enable */
#define SYSTICK_CSR_TICKINT                  (1U << 1)  /*!< Counting down to zero to assert SysTick exception request */
#define SYSTICK_CSR_CLKSOURCE                (1U << 2)  /*!< Clock source selection */
#define SYSTICK_CSR_COUNTFLAG                (1U << 16) /*!< Returns 1 if timer counted to 0 since last read */

/*==============================================================================
 * DWT Register Bit Definitions
 *============================================================================*/

/* DWT_CTRL register bits */
#define DWT_CTRL_CYCCNTENA                   (1U << 0)  /*!< Cycle counter enable */

/* SCB_DEMCR register bits */
#define SCB_DEMCR_TRCENA                     (1U << 24) /*!< Trace enable */

/*==============================================================================
 * System Constants
 *============================================================================*/

/* HSE startup timeout */
#define HSE_STARTUP_TIMEOUT                  0x0500U

/* STM32F103 system clock frequencies */
#define STM32F103_HSI_FREQ_HZ                8000000UL  /*!< Internal High Speed oscillator in Hz */
#define STM32F103_MAX_SYSCLK_HZ              72000000UL /*!< Maximum System Clock frequency in Hz */
#define STM32F103_MAX_AHB_HZ                 72000000UL /*!< Maximum AHB Clock frequency in Hz */
#define STM32F103_MAX_APB1_HZ                36000000UL /*!< Maximum APB1 Clock frequency in Hz */
#define STM32F103_MAX_APB2_HZ                72000000UL /*!< Maximum APB2 Clock frequency in Hz */

#endif /* STM32F103_REGISTERS_H */