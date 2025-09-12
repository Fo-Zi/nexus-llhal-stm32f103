#include "stm32f103_helpers.h"
#include <stddef.h>

static stm32f103_clock_frequencies_t g_clock_frequencies = {0};
static bool g_clock_frequencies_valid = false;

static uint32_t get_ahb_prescaler_value(stm32f103_ahb_prescaler_t prescaler) {
    switch (prescaler) {
        case STM32F103_AHB_DIV_1:   return 0x0;
        case STM32F103_AHB_DIV_2:   return 0x8;
        case STM32F103_AHB_DIV_4:   return 0x9;
        case STM32F103_AHB_DIV_8:   return 0xA;
        case STM32F103_AHB_DIV_16:  return 0xB;
        case STM32F103_AHB_DIV_64:  return 0xC;
        case STM32F103_AHB_DIV_128: return 0xD;
        case STM32F103_AHB_DIV_256: return 0xE;
        case STM32F103_AHB_DIV_512: return 0xF;
        default: return 0x0;
    }
}

static uint32_t get_apb_prescaler_value(stm32f103_apb_prescaler_t prescaler) {
    switch (prescaler) {
        case STM32F103_APB_DIV_1:  return 0x0;
        case STM32F103_APB_DIV_2:  return 0x4;
        case STM32F103_APB_DIV_4:  return 0x5;
        case STM32F103_APB_DIV_8:  return 0x6;
        case STM32F103_APB_DIV_16: return 0x7;
        default: return 0x0;
    }
}

static uint32_t get_adc_prescaler_value(stm32f103_adc_prescaler_t prescaler) {
    switch (prescaler) {
        case STM32F103_ADC_DIV_2: return 0x0;
        case STM32F103_ADC_DIV_4: return 0x1;
        case STM32F103_ADC_DIV_6: return 0x2;
        case STM32F103_ADC_DIV_8: return 0x3;
        default: return 0x0;
    }
}

static int wait_for_flag_set(volatile uint32_t *reg, uint32_t mask, uint32_t timeout_us) {
    uint32_t countdown = timeout_us;
    while (countdown-- && !(*reg & mask)) {
        __asm("nop");
    }
    return (*reg & mask) ? 0 : -1;
}

static int wait_for_flag_match(volatile uint32_t *reg, uint32_t mask, uint32_t expected, uint32_t timeout_us) {
    uint32_t countdown = timeout_us;
    while (countdown-- && ((*reg & mask) != expected)) {
        __asm("nop");
    }
    return ((*reg & mask) == expected) ? 0 : -1;
}

static void calculate_frequencies(const stm32f103_clock_config_t *config,
                                 stm32f103_clock_frequencies_t *freq) {
    uint32_t sysclk = 0;

    switch (config->clock_source) {
        case STM32F103_CLOCK_SOURCE_HSI:
            sysclk = STM32F103_HSI_FREQ_HZ;
            break;

        case STM32F103_CLOCK_SOURCE_HSE:
            sysclk = config->hse_freq_hz;
            break;

        case STM32F103_CLOCK_SOURCE_PLL_HSI: {
            uint32_t pll_input = STM32F103_HSI_FREQ_HZ / 2;
            sysclk = pll_input * config->pll_mul;
            break;
        }

        case STM32F103_CLOCK_SOURCE_PLL_HSE: {
            uint32_t pll_input = config->hse_freq_hz;
            if (config->pll_hse_div2) {
                pll_input /= 2;
            }
            sysclk = pll_input * config->pll_mul;
            break;
        }
    }

    freq->sysclk_hz = sysclk;
    freq->hclk_hz = sysclk / config->ahb_prescaler;
    freq->pclk1_hz = freq->hclk_hz / config->apb1_prescaler;
    freq->pclk2_hz = freq->hclk_hz / config->apb2_prescaler;
    freq->adcclk_hz = freq->pclk2_hz / config->adc_prescaler;

    if (config->clock_source == STM32F103_CLOCK_SOURCE_PLL_HSI ||
        config->clock_source == STM32F103_CLOCK_SOURCE_PLL_HSE) {
        if (config->usb_prescaler_1_5) {
            freq->usbclk_hz = (sysclk * 2) / 3;
        } else {
            freq->usbclk_hz = sysclk;
        }
    } else {
        freq->usbclk_hz = 0;
    }
}

int stm32f103_validate_clock_config(const stm32f103_clock_config_t *config,
                                    stm32f103_clock_frequencies_t *expected_freq) {
    if (!config) return -1;

    stm32f103_clock_frequencies_t freq;
    calculate_frequencies(config, &freq);

    if (freq.sysclk_hz > 72000000) return -1;
    if (freq.hclk_hz > 72000000) return -1;
    if (freq.pclk1_hz > 36000000) return -1;
    if (freq.pclk2_hz > 72000000) return -1;
    if (freq.adcclk_hz > 14000000) return -1;

    if (freq.usbclk_hz != 0 && freq.usbclk_hz != 48000000) return -1;

    if (config->pll_mul < STM32F103_PLL_MUL_2 || config->pll_mul > STM32F103_PLL_MUL_16) return -1;

    if ((config->clock_source == STM32F103_CLOCK_SOURCE_HSE ||
         config->clock_source == STM32F103_CLOCK_SOURCE_PLL_HSE) &&
        config->hse_freq_hz == 0) return -1;

    if (expected_freq) {
        *expected_freq = freq;
    }

    return 0;
}

int stm32f103_default_72mhz_clock_init(){

    uint32_t tmp;

    /* 1) RCC_CR |= RCC_CR_HSEBYP;  // only if using external clock signal (not crystal) */

    /* 2) Enable HSE and wait for HSERDY */
    RCC_CR |= RCC_CR_HSEON;
    if (wait_for(&RCC_CR, RCC_CR_HSERDY, RCC_CR_HSERDY, 0x1FFFFF)) {
        /* HSE failed to start — handle error */
        goto fallback_hsi;
    }

    /* 3) Set Flash latency = 2WS and enable prefetch (needed for 72 MHz) */
    tmp = FLASH_ACR & ~FLASH_ACR_LATENCY_MASK;
    tmp |= 2U;                    /* 2 wait states */
    tmp |= FLASH_ACR_PRFTBE;      /* enable prefetch buffer */
    FLASH_ACR = tmp;

    /* 4) Configure prescalers: AHB = SYSCLK, APB1 = HCLK/2 (<=36MHz), APB2 = HCLK */
    /* Clear HPRE[7:4], PPRE1[10:8], PPRE2[13:11], then set PPRE1 = 0b100 (div2) */
    tmp = RCC_CFGR & ~((0xFU << 4) | (0x7U << 8) | (0x7U << 11));
    tmp |= (0x4U << 8); /* PPRE1 = div2 */
    RCC_CFGR = tmp;

    /* 5) Configure PLL: PLLSRC = HSE (1), PLLXTPRE = 0 (HSE not divided), PLLMUL = x9 (code 7) */
    /* Clear PLLSRC, PLLXTPRE, PLLMUL[21:18] then set them */
    tmp = RCC_CFGR & ~((1U<<16) | (1U<<17) | (0xFU << 18));
    tmp |= (1U << 16);         /* PLLSRC = HSE */
    tmp |= (7U << 18);         /* PLLMUL code = 7 => x9 */
    RCC_CFGR = tmp;

    /* 6) Enable PLL, wait for PLLRDY */
    RCC_CR |= RCC_CR_PLLON;
    if (wait_for(&RCC_CR, RCC_CR_PLLRDY, RCC_CR_PLLRDY, 0x1FFFFF)) {
        /* PLL failed to lock — handle error */
        goto fallback_hsi;
    }

    /* 7) Select PLL as system clock (SW = 10) and wait for switch (SWS = 10) */
    tmp = RCC_CFGR & ~(3U << 0);
    tmp |= (2U << 0);          /* SW = PLL */
    RCC_CFGR = tmp;
    /* wait: SWS bits are bits 3:2 */
    while (((RCC_CFGR >> 2) & 0x3U) != 0x2U) { /* spin */ }

    /* 8) (Optional) turn off HSI to save power */
    RCC_CR &= ~RCC_CR_HSION;

    g_sysclk_hz = 72000000;
    g_clock_frequencies_valid = true;

    return 0;

fallback_hsi:
    /* Safe fallback path: ensure HSI is running and select it as SYSCLK */
    RCC_CR |= RCC_CR_HSION;
    timeout = 0x10000U;
    /* Wait until HSIRDY (HSI is expected to be available quickly) */
    wait_for_mask_set(&RCC_CR, RCC_CR_HSIRDY, timeout);

    /* Disable PLL (if it was on) and HSE to clean state */
    RCC_CR &= ~RCC_CR_PLLON;
    RCC_CR &= ~RCC_CR_HSEON;
    /* Switch system clock to HSI */
    RCC_CFGR = (RCC_CFGR & ~RCC_CFGR_SW) | RCC_CFGR_SW_HSI;
    /* Wait for SWS to indicate HSI */
    timeout = 0x10000U;
    while (((RCC_CFGR & RCC_CFGR_SWS_HSI) != RCC_CFGR_SWS_HSI) && timeout--) {
        __asm("nop");
    }

    g_sysclk_hz = 72000000;
    g_clock_frequencies_valid = true;

    return 0;

};

void stm32f103_get_default_72mhz_config(stm32f103_clock_config_t *config) {
    if (!config) return;

    config->clock_source = STM32F103_CLOCK_SOURCE_PLL_HSE;
    config->hse_freq_hz = 8000000;
    config->hse_bypass = false;
    config->pll_mul = STM32F103_PLL_MUL_9;
    config->pll_hse_div2 = false;
    config->ahb_prescaler = STM32F103_AHB_DIV_1;
    config->apb1_prescaler = STM32F103_APB_DIV_2;
    config->apb2_prescaler = STM32F103_APB_DIV_1;
    config->adc_prescaler = STM32F103_ADC_DIV_6;
    config->flash_latency = STM32F103_FLASH_LATENCY_2;
    config->flash_prefetch_enable = true;
    config->usb_prescaler_1_5 = true;
    config->enable_css = true;
    config->enable_systick = true;
    config->systick_freq_hz = 1000;
}

void stm32f103_get_default_48mhz_hsi_config(stm32f103_clock_config_t *config) {
    if (!config) return;

    config->clock_source = STM32F103_CLOCK_SOURCE_PLL_HSI;
    config->hse_freq_hz = 0;
    config->hse_bypass = false;
    config->pll_mul = STM32F103_PLL_MUL_12;
    config->pll_hse_div2 = false;
    config->ahb_prescaler = STM32F103_AHB_DIV_1;
    config->apb1_prescaler = STM32F103_APB_DIV_2;
    config->apb2_prescaler = STM32F103_APB_DIV_1;
    config->adc_prescaler = STM32F103_ADC_DIV_6;
    config->flash_latency = STM32F103_FLASH_LATENCY_1;
    config->flash_prefetch_enable = true;
    config->usb_prescaler_1_5 = false;
    config->enable_css = false;
    config->enable_systick = true;
    config->systick_freq_hz = 1000;
}

uint32_t stm32f103_get_sysclk_hz(void) {
    return g_clock_frequencies.sysclk_hz;
}

uint32_t stm32f103_get_gpio_base(stm32f103_gpio_port_t port) {
    switch (port) {
        case STM32F103_GPIO_PORT_A: return 0x40010800;
        case STM32F103_GPIO_PORT_B: return 0x40010C00;
        case STM32F103_GPIO_PORT_C: return 0x40011000;
        case STM32F103_GPIO_PORT_D: return 0x40011400;
        case STM32F103_GPIO_PORT_E: return 0x40011800;
        default: return 0;
    }
}

uint32_t stm32f103_get_gpio_clk_mask(stm32f103_gpio_port_t port) {
    switch (port) {
        case STM32F103_GPIO_PORT_A: return RCC_APB2ENR_IOPAEN;
        case STM32F103_GPIO_PORT_B: return RCC_APB2ENR_IOPBEN;
        case STM32F103_GPIO_PORT_C: return RCC_APB2ENR_IOPCEN;
        case STM32F103_GPIO_PORT_D: return RCC_APB2ENR_IOPDEN;
        case STM32F103_GPIO_PORT_E: return RCC_APB2ENR_IOPEEN;
        default: return 0;
    }
}

static uint32_t g_sysclk_hz = 8000000; // default after reset (HSI=8 MHz)

static inline void dwt_init(void) {
    DEMCR |= DEMCR_TRCENA;          // enable trace
    DWT_CYCCNT = 0;                 // reset cycle counter
    DWT_CTRL |= DWT_CTRL_CYCCNTENA; // enable cycle counter
}

void stm32f103_delay_us(uint32_t microseconds) {
    uint32_t cycles = (g_sysclk_hz / 1000000U) * us;
    uint32_t start = DWT_CYCCNT;
    while ((DWT_CYCCNT - start) < cycles) {
        __asm__ volatile ("nop");
    }
}

void stm32f103_delay_ms(uint32_t milliseconds) {
    while (ms--) {
        delay_us(1000);
    }
}

int stm32f103_set_clock_frequencies(const stm32f103_clock_frequencies_t *frequencies) {
    if (!frequencies) {
        return -1;
    }

    g_clock_frequencies = *frequencies;
    g_clock_frequencies_valid = true;
    return 0;
}

int stm32f103_get_clock_frequencies(stm32f103_clock_frequencies_t *frequencies) {
    if (!frequencies) {
        return -1;
    }

    *frequencies = g_clock_frequencies;
    return 0;
}

int stm32f103_detect_clock_frequencies(void) {
    stm32f103_clock_frequencies_t detected = {0};

    // Read current clock source from RCC_CFGR SWS bits
    uint32_t sws = (RCC_CFGR >> 2) & 0x3;

    switch (sws) {
        case 0x0: // HSI
            detected.sysclk_hz = STM32F103_HSI_FREQ_HZ;
            break;

        case 0x1: // HSE
            // Need to detect HSE frequency - assume 8MHz for Blue Pill
            detected.sysclk_hz = 8000000;
            break;

        case 0x2: { // PLL
            // Read PLL configuration
            uint32_t pllmul = ((RCC_CFGR >> 18) & 0xF) + 2;
            uint32_t pllsrc = (RCC_CFGR >> 16) & 0x1;
            uint32_t pllxtpre = (RCC_CFGR >> 17) & 0x1;

            uint32_t pll_input;
            if (pllsrc == 0) {
                // HSI/2 as PLL source
                pll_input = STM32F103_HSI_FREQ_HZ / 2;
            } else {
                // HSE as PLL source
                pll_input = 8000000; // Assume 8MHz HSE for Blue Pill
                if (pllxtpre) {
                    pll_input /= 2;
                }
            }

            detected.sysclk_hz = pll_input * pllmul;
            break;
        }

        default:
            return -1; // Unknown clock source
    }

    // Calculate other frequencies from prescalers
    uint32_t hpre = (RCC_CFGR >> 4) & 0xF;
    uint32_t ppre1 = (RCC_CFGR >> 8) & 0x7;
    uint32_t ppre2 = (RCC_CFGR >> 11) & 0x7;
    uint32_t adcpre = (RCC_CFGR >> 14) & 0x3;

    // AHB prescaler
    if (hpre < 8) {
        detected.hclk_hz = detected.sysclk_hz;
    } else {
        uint32_t ahb_div = 1 << (hpre - 7);
        detected.hclk_hz = detected.sysclk_hz / ahb_div;
    }

    // APB1 prescaler
    if (ppre1 < 4) {
        detected.pclk1_hz = detected.hclk_hz;
    } else {
        uint32_t apb1_div = 1 << (ppre1 - 3);
        detected.pclk1_hz = detected.hclk_hz / apb1_div;
    }

    // APB2 prescaler
    if (ppre2 < 4) {
        detected.pclk2_hz = detected.hclk_hz;
    } else {
        uint32_t apb2_div = 1 << (ppre2 - 3);
        detected.pclk2_hz = detected.hclk_hz / apb2_div;
    }

    // ADC prescaler
    uint32_t adc_divs[] = {2, 4, 6, 8};
    detected.adcclk_hz = detected.pclk2_hz / adc_divs[adcpre];

    // USB clock (only valid for PLL source)
    if (sws == 0x2) {
        uint32_t usbpre = (RCC_CFGR >> 22) & 0x1;
        if (usbpre == 0) {
            detected.usbclk_hz = (detected.sysclk_hz * 2) / 3; // PLL / 1.5
        } else {
            detected.usbclk_hz = detected.sysclk_hz; // PLL / 1
        }
    }

    g_clock_frequencies = detected;
    g_clock_frequencies_valid = true;

    return 0;
}

int stm32f103_uart_debug_init(void) {
    return -1;
}

void stm32f103_uart_debug_print(const char* str) {
    (void)str;
}

void stm32f103_uart_debug_print_num(const char* str, uint32_t num) {
    (void)str;
    (void)num;
}
