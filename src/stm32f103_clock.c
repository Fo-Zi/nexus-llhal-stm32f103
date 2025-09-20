#include "stm32f103_clock.h"
#include "stm32f103_registers.h"
#include <stddef.h>

/**
 * @file stm32f103_clock.c
 * @brief STM32F103 Clock System Management Implementation
 */

/*==============================================================================
 * Private Variables
 *============================================================================*/

static stm32f103_clock_frequencies_t g_clock_frequencies = {0};
static bool g_clock_frequencies_valid = false;

/*==============================================================================
 * Private Function Declarations
 *============================================================================*/

static int wait_for_flag_set(volatile uint32_t *reg, uint32_t mask, uint32_t timeout_us);
static int wait_for_flag_match(volatile uint32_t *reg, uint32_t mask, uint32_t expected, uint32_t timeout_us);


static uint32_t get_ahb_prescaler_value(stm32f103_ahb_prescaler_t prescaler);
static uint32_t get_apb_prescaler_value(stm32f103_apb_prescaler_t prescaler);
static uint32_t get_adc_prescaler_value(stm32f103_adc_prescaler_t prescaler);
static uint32_t get_flash_latency_value(stm32f103_flash_latency_t latency);
static uint32_t get_pll_mul_value(stm32f103_pll_mul_t pll_mul);

static int configure_hse(bool enable, bool bypass, uint32_t timeout_us);
static int configure_pll(const stm32f103_clock_config_t *config);
static int configure_flash_latency(const stm32f103_clock_config_t *config);
static int configure_bus_prescalers(const stm32f103_clock_config_t *config);
static int switch_system_clock(stm32f103_clock_source_t source);
static int configure_css(bool enable);
static int configure_systick(const stm32f103_clock_config_t *config, uint32_t sysclk_hz);
static void calculate_frequencies(const stm32f103_clock_config_t *config,
                                  stm32f103_clock_frequencies_t *frequencies);

/*==============================================================================
 * Private Helper Functions
 *============================================================================*/

static int wait_for_flag_set(volatile uint32_t *reg, uint32_t mask, uint32_t timeout_us) {
    uint32_t count = 0;
    const uint32_t max_count = timeout_us;

    while ((*reg & mask) == 0) {
        if (++count > max_count) {
            return -1; // Timeout
        }
        // Simple delay loop (not accurate but sufficient for timeouts)
        for (volatile int i = 0; i < 100; i++){
            __asm("nop");
        };
    }
    return 0;
}

static int wait_for_flag_match(volatile uint32_t *reg, uint32_t mask, uint32_t expected, uint32_t timeout_us) {
    uint32_t count = 0;
    const uint32_t max_count = timeout_us;

    while ((*reg & mask) != expected) {
        if (++count > max_count) {
            return -1; // Timeout
        }
        // Simple delay loop (not accurate but sufficient for timeouts)
        for (volatile int i = 0; i < 100; i++){
            __asm volatile ("nop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\n");
        };
    }
    return 0;
}



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

static uint32_t get_flash_latency_value(stm32f103_flash_latency_t latency) {
    switch (latency) {
        case STM32F103_FLASH_LATENCY_0: return FLASH_ACR_LATENCY_0;
        case STM32F103_FLASH_LATENCY_1: return FLASH_ACR_LATENCY_1;
        case STM32F103_FLASH_LATENCY_2: return FLASH_ACR_LATENCY_2;
        default: return FLASH_ACR_LATENCY_0;
    }
}

static inline __attribute__((always_inline)) uint32_t get_pll_mul_value(stm32f103_pll_mul_t pll_mul) {
    if (pll_mul < STM32F103_PLL_MUL_2 || pll_mul > STM32F103_PLL_MUL_16) {
        return 0; // Invalid
    }
    return (pll_mul - 2) << RCC_CFGR_PLLMULL_Pos; // PLL_MUL - 2 shifted to position
}

static inline __attribute__((always_inline)) int configure_hse(bool enable, bool bypass, uint32_t timeout_us) {
    if (enable) {
        // Configure HSE bypass if requested
        if (bypass) {
            RCC_CR |= RCC_CR_HSEBYP;
        } else {
            RCC_CR &= ~RCC_CR_HSEBYP;
        }

        // Enable HSE and wait for ready
        RCC_CR |= RCC_CR_HSEON;
        __asm volatile ("dsb" ::: "memory");
        return wait_for_flag_set(&RCC_CR, RCC_CR_HSERDY, timeout_us);
    } else {
        // Disable HSE
        RCC_CR &= ~(RCC_CR_HSEON | RCC_CR_HSEBYP);
        return 0;
    }
}

static inline __attribute__((always_inline)) int configure_pll(const stm32f103_clock_config_t *config) {
    // Disable PLL first
    RCC_CR &= ~RCC_CR_PLLON;

    // Wait for PLL to be disabled with timeout
    if (wait_for_flag_match(&RCC_CR, RCC_CR_PLLRDY, 0, HSE_STARTUP_TIMEOUT) != 0) {
        return -2; // PLL disable timeout - will trigger error_recovery fallback to HSI
    }

    // Configure PLL source and parameters
    uint32_t cfgr = RCC_CFGR;
    cfgr &= ~(RCC_CFGR_PLLSRC | RCC_CFGR_PLLXTPRE | RCC_CFGR_PLLMULL_Msk);

    if (config->clock_source == STM32F103_CLOCK_SOURCE_PLL_HSE) {
        cfgr |= RCC_CFGR_PLLSRC_HSE;
        if (config->pll_hse_div2) {
            cfgr |= RCC_CFGR_PLLXTPRE;
        }
    } else {
        cfgr |= RCC_CFGR_PLLSRC_HSI;  // HSI/2 as PLL source
    }

    cfgr |= get_pll_mul_value(config->pll_mul);
    RCC_CFGR = cfgr;
    __asm volatile ("dsb" ::: "memory");

    // Enable PLL and wait for ready
    RCC_CR |= RCC_CR_PLLON;
    __asm volatile ("dsb" ::: "memory");
    return wait_for_flag_set(&RCC_CR, RCC_CR_PLLRDY, HSE_STARTUP_TIMEOUT);
}

static inline __attribute__((always_inline)) int configure_flash_latency(const stm32f103_clock_config_t *config) {
    uint32_t acr = FLASH_ACR;
    acr &= ~FLASH_ACR_LATENCY_Msk;
    acr |= get_flash_latency_value(config->flash_latency);

    if (config->flash_prefetch_enable) {
        acr |= FLASH_ACR_PRFTBE;
    } else {
        acr &= ~FLASH_ACR_PRFTBE;
    }

    FLASH_ACR = acr;
    __asm volatile ("dsb" ::: "memory");
    return 0;
}

static inline __attribute__((always_inline)) int configure_bus_prescalers(const stm32f103_clock_config_t *config) {
    uint32_t cfgr = RCC_CFGR;

    // Clear prescaler fields
    cfgr &= ~(RCC_CFGR_HPRE_Msk | RCC_CFGR_PPRE1_Msk | RCC_CFGR_PPRE2_Msk | RCC_CFGR_ADCPRE_Msk);

    // Set prescalers
    cfgr |= get_ahb_prescaler_value(config->ahb_prescaler) << RCC_CFGR_HPRE_Pos;
    cfgr |= get_apb_prescaler_value(config->apb1_prescaler) << RCC_CFGR_PPRE1_Pos;
    cfgr |= get_apb_prescaler_value(config->apb2_prescaler) << RCC_CFGR_PPRE2_Pos;
    cfgr |= get_adc_prescaler_value(config->adc_prescaler) << RCC_CFGR_ADCPRE_Pos;

    // Configure USB prescaler if using PLL
    if (config->clock_source == STM32F103_CLOCK_SOURCE_PLL_HSE ||
        config->clock_source == STM32F103_CLOCK_SOURCE_PLL_HSI) {
        if (config->usb_prescaler_1_5) {
            cfgr &= ~RCC_CFGR_USBPRE;  // PLL/1.5
        } else {
            cfgr |= RCC_CFGR_USBPRE;   // PLL/1
        }
    }

    RCC_CFGR = cfgr;
    __asm volatile ("dsb" ::: "memory");
    return 0;
}

static inline __attribute__((always_inline)) int switch_system_clock(stm32f103_clock_source_t source) {
    uint32_t cfgr = RCC_CFGR;
    uint32_t target_sw, target_sws;

    cfgr &= ~RCC_CFGR_SW_Msk;

    switch (source) {
        case STM32F103_CLOCK_SOURCE_HSI:
            target_sw = RCC_CFGR_SW_HSI;
            target_sws = RCC_CFGR_SWS_HSI;
            break;
        case STM32F103_CLOCK_SOURCE_HSE:
            target_sw = RCC_CFGR_SW_HSE;
            target_sws = RCC_CFGR_SWS_HSE;
            break;
        case STM32F103_CLOCK_SOURCE_PLL_HSI:
        case STM32F103_CLOCK_SOURCE_PLL_HSE:
            target_sw = RCC_CFGR_SW_PLL;
            target_sws = RCC_CFGR_SWS_PLL;
            break;
        default:
            return -4; // Invalid clock source
    }

    cfgr |= target_sw;
    RCC_CFGR = cfgr;
    __asm volatile ("dsb" ::: "memory");

    // Wait for system clock switch
    return wait_for_flag_match(&RCC_CFGR, RCC_CFGR_SWS_Msk, target_sws, HSE_STARTUP_TIMEOUT);
}

static inline __attribute__((always_inline)) int configure_css(bool enable) {
    if (enable) {
        RCC_CR |= RCC_CR_CSSON;
    } else {
        RCC_CR &= ~RCC_CR_CSSON;
    }
    return 0;
}

static inline __attribute__((always_inline)) int configure_systick(const stm32f103_clock_config_t *config, uint32_t sysclk_hz) {
    if (!config->enable_systick || config->systick_freq_hz == 0) {
        // Disable SysTick
        SYSTICK_CSR &= ~SYSTICK_CSR_ENABLE;
        return 0;
    }

    // Calculate SysTick reload value
    uint32_t reload = (sysclk_hz / config->systick_freq_hz) - 1;
    if (reload > 0xFFFFFF) {
        return -1; // Reload value too large
    }

    // Configure SysTick
    SYSTICK_RVR = reload;
    SYSTICK_CVR = 0; // Clear current value
    SYSTICK_CSR = SYSTICK_CSR_CLKSOURCE | SYSTICK_CSR_ENABLE;
    __asm volatile ("dsb" ::: "memory");

    return 0;
}

static inline __attribute__((always_inline)) void calculate_frequencies(const stm32f103_clock_config_t *config,
                                  stm32f103_clock_frequencies_t *frequencies) {
    uint32_t sysclk_hz = 0;

    // Calculate SYSCLK based on source
    switch (config->clock_source) {
        case STM32F103_CLOCK_SOURCE_HSI:
            sysclk_hz = 8000000; // HSI is 8MHz
            break;

        case STM32F103_CLOCK_SOURCE_HSE:
            sysclk_hz = config->hse_freq_hz;
            break;

        case STM32F103_CLOCK_SOURCE_PLL_HSI: {
            uint32_t pll_input = 8000000 / 2; // HSI/2 = 4MHz
            sysclk_hz = pll_input * config->pll_mul;
            break;
        }

        case STM32F103_CLOCK_SOURCE_PLL_HSE: {
            uint32_t pll_input = config->hse_freq_hz;
            if (config->pll_hse_div2) {
                pll_input /= 2;
            }
            sysclk_hz = pll_input * config->pll_mul;
            break;
        }
    }

    // Calculate bus clocks
    frequencies->sysclk_hz = sysclk_hz;
    frequencies->hclk_hz = sysclk_hz / config->ahb_prescaler;
    frequencies->pclk1_hz = frequencies->hclk_hz / config->apb1_prescaler;
    frequencies->pclk2_hz = frequencies->hclk_hz / config->apb2_prescaler;
    frequencies->adcclk_hz = frequencies->pclk2_hz / config->adc_prescaler;

    // Calculate USB clock (only if using PLL)
    if (config->clock_source == STM32F103_CLOCK_SOURCE_PLL_HSE ||
        config->clock_source == STM32F103_CLOCK_SOURCE_PLL_HSI) {
        if (config->usb_prescaler_1_5) {
            frequencies->usbclk_hz = (sysclk_hz * 2) / 3; // PLL/1.5
        } else {
            frequencies->usbclk_hz = sysclk_hz; // PLL/1
        }
    } else {
        frequencies->usbclk_hz = 0; // USB not available without PLL
    }
}

/*==============================================================================
 * Public Functions - Clock System Initialization
 *============================================================================*/

int stm32f103_clock_init(const stm32f103_clock_config_t *config,
                         stm32f103_clock_frequencies_t *frequencies) {
    if (!config) {
        return -1;
    }

    // Validate configuration
    stm32f103_clock_frequencies_t calc_freq;
    int result = stm32f103_validate_clock_config(config, &calc_freq);
    if (result != 0) {
        return result;
    }

    // Save current system state for error recovery
    uint32_t original_cr = RCC_CR;
    uint32_t original_cfgr = RCC_CFGR;
    uint32_t original_acr = FLASH_ACR;

    // Enable HSI as a safe fallback first
    RCC_CR |= RCC_CR_HSION;
    if (wait_for_flag_set(&RCC_CR, RCC_CR_HSIRDY, HSE_STARTUP_TIMEOUT) != 0) {
        return -1; // HSI failed to start
    }

    // Configure flash latency first (before increasing frequency)
    result = configure_flash_latency(config);
    if (result != 0) {
        goto error_recovery;
    }

    // Configure HSE if needed
    if (config->clock_source == STM32F103_CLOCK_SOURCE_HSE ||
        config->clock_source == STM32F103_CLOCK_SOURCE_PLL_HSE) {
        result = configure_hse(true, config->hse_bypass, HSE_STARTUP_TIMEOUT);
        if (result != 0) {
            goto error_recovery; // HSE startup timeout
        }
    }

    // Configure PLL if needed
    if (config->clock_source == STM32F103_CLOCK_SOURCE_PLL_HSI ||
        config->clock_source == STM32F103_CLOCK_SOURCE_PLL_HSE) {
        result = configure_pll(config);
        if (result != 0) {
            goto error_recovery; // PLL startup timeout
        }
    }

    // Configure bus prescalers
    result = configure_bus_prescalers(config);
    if (result != 0) {
        goto error_recovery;
    }

    // Switch system clock
    result = switch_system_clock(config->clock_source);
    if (result != 0) {
        goto error_recovery;
    }

    // Configure CSS if enabled
    result = configure_css(config->enable_css);
    if (result != 0) {
        goto error_recovery;
    }

    // Configure SysTick if enabled
    result = configure_systick(config, calc_freq.sysclk_hz);
    if (result != 0) {
        goto error_recovery;
    }

    // Update internal frequency reference
    g_clock_frequencies = calc_freq;
    g_clock_frequencies_valid = true;

    // Return calculated frequencies if requested
    if (frequencies) {
        *frequencies = calc_freq;
    }

    return 0;

error_recovery:
    // Restore original state on error
    RCC_CR = original_cr;
    RCC_CFGR = original_cfgr;
    FLASH_ACR = original_acr;

    // Try to ensure HSI is working
    RCC_CR |= RCC_CR_HSION;
    wait_for_flag_set(&RCC_CR, RCC_CR_HSIRDY, HSE_STARTUP_TIMEOUT);
    switch_system_clock(STM32F103_CLOCK_SOURCE_HSI);

    // Update frequency tracking for HSI fallback
    g_clock_frequencies.sysclk_hz = 8000000;
    g_clock_frequencies_valid = true;

    return -3; // Configuration failed, system restored to safe state
}

int stm32f103_default_72mhz_clock_init(void) {
    stm32f103_clock_config_t config;
    stm32f103_get_default_72mhz_config(&config);
    return stm32f103_clock_init(&config, NULL);
}

/*==============================================================================
 * Public Functions - Predefined Configurations
 *============================================================================*/

void stm32f103_get_default_72mhz_config(stm32f103_clock_config_t *config) {
    if (!config) return;

    config->clock_source = STM32F103_CLOCK_SOURCE_PLL_HSE;
    config->hse_freq_hz = 8000000;
    config->hse_bypass = false;
    config->pll_mul = STM32F103_PLL_MUL_9; // 8MHz * 9 = 72MHz
    config->pll_hse_div2 = false;
    config->ahb_prescaler = STM32F103_AHB_DIV_1;
    config->apb1_prescaler = STM32F103_APB_DIV_2;
    config->apb2_prescaler = STM32F103_APB_DIV_1;
    config->adc_prescaler = STM32F103_ADC_DIV_6;
    config->flash_latency = STM32F103_FLASH_LATENCY_2;
    config->flash_prefetch_enable = true;
    config->usb_prescaler_1_5 = true; // 72MHz/1.5 = 48MHz for USB
    config->enable_css = false;
    config->enable_systick = true;
    config->systick_freq_hz = 1000; // 1ms tick
}

void stm32f103_get_default_48mhz_hsi_config(stm32f103_clock_config_t *config) {
    if (!config) return;

    config->clock_source = STM32F103_CLOCK_SOURCE_PLL_HSI;
    config->hse_freq_hz = 0;
    config->hse_bypass = false;
    config->pll_mul = STM32F103_PLL_MUL_12; // (8MHz/2) * 12 = 48MHz
    config->pll_hse_div2 = false;
    config->ahb_prescaler = STM32F103_AHB_DIV_1;
    config->apb1_prescaler = STM32F103_APB_DIV_2;
    config->apb2_prescaler = STM32F103_APB_DIV_1;
    config->adc_prescaler = STM32F103_ADC_DIV_6;
    config->flash_latency = STM32F103_FLASH_LATENCY_1;
    config->flash_prefetch_enable = true;
    config->usb_prescaler_1_5 = false; // 48MHz/1 = 48MHz for USB
    config->enable_css = false;
    config->enable_systick = true;
    config->systick_freq_hz = 1000; // 1ms tick
}

void stm32f103_get_default_8mhz_hsi_config(stm32f103_clock_config_t *config) {
    if (!config) return;

    config->clock_source = STM32F103_CLOCK_SOURCE_HSI;
    config->hse_freq_hz = 0;
    config->hse_bypass = false;
    config->pll_mul = STM32F103_PLL_MUL_2; // Not used
    config->pll_hse_div2 = false;
    config->ahb_prescaler = STM32F103_AHB_DIV_1;
    config->apb1_prescaler = STM32F103_APB_DIV_1;
    config->apb2_prescaler = STM32F103_APB_DIV_1;
    config->adc_prescaler = STM32F103_ADC_DIV_2;
    config->flash_latency = STM32F103_FLASH_LATENCY_0;
    config->flash_prefetch_enable = false;
    config->usb_prescaler_1_5 = false; // USB not available
    config->enable_css = false;
    config->enable_systick = true;
    config->systick_freq_hz = 1000; // 1ms tick
}

/*==============================================================================
 * Public Functions - Clock Frequency Management
 *============================================================================*/

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

    if (!g_clock_frequencies_valid) {
        // Auto-detect if not set
        stm32f103_detect_clock_frequencies();
    }

    *frequencies = g_clock_frequencies;
    return 0;
}

int stm32f103_detect_clock_frequencies(void) {
    stm32f103_clock_frequencies_t freq;

    // Get SYSCLK from hardware registers
    freq.sysclk_hz = stm32f103_get_sysclk_hz();

    // Calculate other frequencies based on register values
    uint32_t cfgr = RCC_CFGR;

    // AHB prescaler
    uint32_t hpre = (cfgr & RCC_CFGR_HPRE_Msk) >> RCC_CFGR_HPRE_Pos;
    uint32_t ahb_div = (hpre < 8) ? 1 : (2 << (hpre - 7));
    freq.hclk_hz = freq.sysclk_hz / ahb_div;

    // APB1 prescaler
    uint32_t ppre1 = (cfgr & RCC_CFGR_PPRE1_Msk) >> RCC_CFGR_PPRE1_Pos;
    uint32_t apb1_div = (ppre1 < 4) ? 1 : (2 << (ppre1 - 3));
    freq.pclk1_hz = freq.hclk_hz / apb1_div;

    // APB2 prescaler
    uint32_t ppre2 = (cfgr & RCC_CFGR_PPRE2_Msk) >> RCC_CFGR_PPRE2_Pos;
    uint32_t apb2_div = (ppre2 < 4) ? 1 : (2 << (ppre2 - 3));
    freq.pclk2_hz = freq.hclk_hz / apb2_div;

    // ADC prescaler
    uint32_t adcpre = (cfgr & RCC_CFGR_ADCPRE_Msk) >> RCC_CFGR_ADCPRE_Pos;
    uint32_t adc_div = (adcpre == 0) ? 2 : ((adcpre == 1) ? 4 : ((adcpre == 2) ? 6 : 8));
    freq.adcclk_hz = freq.pclk2_hz / adc_div;

    // USB clock (only available with PLL)
    uint32_t sws = (cfgr & RCC_CFGR_SWS_Msk) >> RCC_CFGR_SWS_Pos;
    if (sws == 2) { // PLL as system clock
        bool usb_div = (cfgr & RCC_CFGR_USBPRE) ? false : true; // Inverted logic
        freq.usbclk_hz = usb_div ? ((freq.sysclk_hz * 2) / 3) : freq.sysclk_hz;
    } else {
        freq.usbclk_hz = 0;
    }

    // Update internal reference
    g_clock_frequencies = freq;
    g_clock_frequencies_valid = true;

    return 0;
}

uint32_t stm32f103_get_sysclk_hz(void) {
    uint32_t cfgr = RCC_CFGR;
    uint32_t sws = (cfgr & RCC_CFGR_SWS_Msk) >> RCC_CFGR_SWS_Pos;

    switch (sws) {
        case 0: // HSI
            return 8000000;

        case 1: // HSE
            // We can't determine HSE frequency from registers alone
            // Use stored value if available, otherwise assume 8MHz
            return g_clock_frequencies_valid ?
                   g_clock_frequencies.sysclk_hz : 8000000;

        case 2: { // PLL
            // This is complex - would need to read all PLL configuration
            // Use stored value if available, otherwise assume 72MHz
            return g_clock_frequencies_valid ?
                   g_clock_frequencies.sysclk_hz : 72000000;
        }

        default:
            return 8000000; // Default to HSI
    }
}

/*==============================================================================
 * Public Functions - Clock Status and Validation
 *============================================================================*/

int stm32f103_validate_clock_config(const stm32f103_clock_config_t *config,
                                    stm32f103_clock_frequencies_t *expected_freq) {
    if (!config) {
        return -1;
    }

    // Calculate expected frequencies
    stm32f103_clock_frequencies_t freq;
    calculate_frequencies(config, &freq);

    // Validate frequency ranges
    if (freq.sysclk_hz > 72000000) {
        return -5; // SYSCLK too high
    }

    if (freq.pclk1_hz > 36000000) {
        return -5; // PCLK1 too high
    }

    if (freq.pclk2_hz > 72000000) {
        return -5; // PCLK2 too high
    }

    if (freq.adcclk_hz > 14000000) {
        return -5; // ADC clock too high
    }

    // Validate USB clock (must be 48MHz if enabled)
    if (freq.usbclk_hz != 0 && freq.usbclk_hz != 48000000) {
        return -5; // Invalid USB clock
    }

    // Return expected frequencies if requested
    if (expected_freq) {
        *expected_freq = freq;
    }

    return 0;
}

bool stm32f103_is_pll_ready(void) {
    return (RCC_CR & RCC_CR_PLLRDY) != 0;
}

bool stm32f103_is_hse_ready(void) {
    return (RCC_CR & RCC_CR_HSERDY) != 0;
}

bool stm32f103_is_hsi_ready(void) {
    return (RCC_CR & RCC_CR_HSIRDY) != 0;
}

stm32f103_clock_source_t stm32f103_get_current_clock_source(void) {
    uint32_t sws = (RCC_CFGR & RCC_CFGR_SWS_Msk) >> RCC_CFGR_SWS_Pos;

    switch (sws) {
        case 0: return STM32F103_CLOCK_SOURCE_HSI;
        case 1: return STM32F103_CLOCK_SOURCE_HSE;
        case 2: {
            // Determine if PLL source is HSI or HSE
            bool pll_src = (RCC_CFGR & RCC_CFGR_PLLSRC) != 0;
            return pll_src ? STM32F103_CLOCK_SOURCE_PLL_HSE : STM32F103_CLOCK_SOURCE_PLL_HSI;
        }
        default: return STM32F103_CLOCK_SOURCE_HSI;
    }
}

/*==============================================================================
 * Public Functions - Peripheral Clock Management
 *============================================================================*/

int stm32f103_enable_peripheral_clock(uint32_t rcc_register_offset, uint32_t enable_bit) {
    volatile uint32_t *reg = (volatile uint32_t *)(RCC_BASE + rcc_register_offset);
    *reg |= enable_bit;
    __asm volatile ("dsb" ::: "memory");
    return 0;
}

int stm32f103_disable_peripheral_clock(uint32_t rcc_register_offset, uint32_t enable_bit) {
    volatile uint32_t *reg = (volatile uint32_t *)(RCC_BASE + rcc_register_offset);
    *reg &= ~enable_bit;
    return 0;
}

/*==============================================================================
 * Public Functions - System Control
 *============================================================================*/

void stm32f103_system_reset(void) {
    // Trigger system reset via AIRCR
    SCB_AIRCR = (0x5FA << 16) | (1 << 2); // SYSRESETREQ

    // Wait for reset (should not return)
    while (1) {
        __asm volatile ("nop");
    }
}
