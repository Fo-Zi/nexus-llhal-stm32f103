#include "stm32f103_helpers.h"
#include <stddef.h>

static stm32f103_clock_frequencies_t g_clock_frequencies = {0};
static uint32_t g_sysclk_hz = 8000000; // default after reset (HSI=8 MHz)
static bool g_clock_frequencies_valid = false;

/*==============================================================================
 * Forward Declarations for Static Helper Functions
 *============================================================================*/
static int wait_for_flag_set(volatile uint32_t *reg, uint32_t mask, uint32_t timeout_us);
static int wait_for_flag_match(volatile uint32_t *reg, uint32_t mask, uint32_t expected, uint32_t timeout_us);
static int wait_for(volatile uint32_t *reg, uint32_t mask, uint32_t expect, uint32_t timeout);

/*==============================================================================
 * Static Helper Functions for Clock Configuration
 *============================================================================*/

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

static uint32_t get_pll_mul_value(stm32f103_pll_mul_t pll_mul) {
    if (pll_mul < STM32F103_PLL_MUL_2 || pll_mul > STM32F103_PLL_MUL_16) {
        return 0; // Invalid
    }
    return (pll_mul - 2) << RCC_CFGR_PLLMULL_Pos; // PLL_MUL - 2 shifted to position
}

static int configure_hse(bool enable, bool bypass, uint32_t timeout_us) {
    if (enable) {
        // Configure HSE bypass if requested
        if (bypass) {
            RCC_CR |= RCC_CR_HSEBYP;
        } else {
            RCC_CR &= ~RCC_CR_HSEBYP;
        }
        
        // Enable HSE and wait for ready
        RCC_CR |= RCC_CR_HSEON;
        return wait_for_flag_set(&RCC_CR, RCC_CR_HSERDY, timeout_us);
    } else {
        // Disable HSE
        RCC_CR &= ~(RCC_CR_HSEON | RCC_CR_HSEBYP);
        return 0;
    }
}

static int configure_pll(const stm32f103_clock_config_t *config) {
    // Disable PLL first
    RCC_CR &= ~RCC_CR_PLLON;
    
    // Wait for PLL to be disabled
    while (RCC_CR & RCC_CR_PLLRDY) { /* spin */ }
    
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
    
    // Enable PLL and wait for ready
    RCC_CR |= RCC_CR_PLLON;
    return wait_for_flag_set(&RCC_CR, RCC_CR_PLLRDY, HSE_STARTUP_TIMEOUT);
}

static int configure_flash_latency(const stm32f103_clock_config_t *config) {
    uint32_t acr = FLASH_ACR;
    acr &= ~FLASH_ACR_LATENCY_Msk;
    acr |= get_flash_latency_value(config->flash_latency);
    
    if (config->flash_prefetch_enable) {
        acr |= FLASH_ACR_PRFTBE;
    } else {
        acr &= ~FLASH_ACR_PRFTBE;
    }
    
    FLASH_ACR = acr;
    return 0;
}

static int configure_bus_prescalers(const stm32f103_clock_config_t *config) {
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
    return 0;
}

static int switch_system_clock(stm32f103_clock_source_t source) {
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
    
    // Wait for system clock switch
    return wait_for_flag_match(&RCC_CFGR, RCC_CFGR_SWS_Msk, target_sws, HSE_STARTUP_TIMEOUT);
}

static int configure_css(bool enable) {
    if (enable) {
        RCC_CR |= RCC_CR_CSSON;
    } else {
        RCC_CR &= ~RCC_CR_CSSON;
    }
    return 0;
}

static int configure_systick(const stm32f103_clock_config_t *config, uint32_t sysclk_hz) {
    if (!config->enable_systick || config->systick_freq_hz == 0) {
        // Disable SysTick
        SYSTICK_CSR &= ~SYSTICK_CSR_ENABLE;
        return 0;
    }
    
    // Calculate reload value
    uint32_t reload = (sysclk_hz / config->systick_freq_hz) - 1;
    if (reload > 0xFFFFFF) {
        return -5; // SysTick reload value too large
    }
    
    // Configure SysTick
    SYSTICK_RVR = reload;
    SYSTICK_CVR = 0; // Clear current value
    SYSTICK_CSR = SYSTICK_CSR_CLKSOURCE | SYSTICK_CSR_ENABLE;
    
    return 0;
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

static int wait_for(volatile uint32_t *reg, uint32_t mask, uint32_t expect, uint32_t timeout) {
    while (timeout--) {
        if (((*reg) & mask) == expect) return 0;
    }
    return -1;
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

/*==============================================================================
 * Comprehensive Clock Configuration Implementation
 *============================================================================*/

int stm32f103_clock_init(const stm32f103_clock_config_t *config,
                         stm32f103_clock_frequencies_t *frequencies) {
    if (!config) {
        return -1; // Invalid configuration
    }
    
    // Validate configuration first
    stm32f103_clock_frequencies_t expected_freq;
    int validation_result = stm32f103_validate_clock_config(config, &expected_freq);
    if (validation_result != 0) {
        return validation_result;
    }
    
    // Save current system state for error recovery
    uint32_t original_cr = RCC_CR;
    uint32_t original_cfgr = RCC_CFGR;
    uint32_t original_acr = FLASH_ACR;
    
    // Step 1: Enable HSI as a safe fallback
    RCC_CR |= RCC_CR_HSION;
    if (wait_for_flag_set(&RCC_CR, RCC_CR_HSIRDY, HSE_STARTUP_TIMEOUT) != 0) {
        return -1; // HSI failed to start
    }
    
    // Step 2: Switch to HSI temporarily for safe configuration
    if (switch_system_clock(STM32F103_CLOCK_SOURCE_HSI) != 0) {
        return -4; // Failed to switch to HSI
    }
    
    // Step 3: Configure Flash latency BEFORE increasing frequency
    if (configure_flash_latency(config) != 0) {
        goto error_recovery;
    }
    
    // Step 4: Configure HSE if needed
    bool need_hse = (config->clock_source == STM32F103_CLOCK_SOURCE_HSE ||
                     config->clock_source == STM32F103_CLOCK_SOURCE_PLL_HSE);
    
    if (need_hse) {
        if (configure_hse(true, config->hse_bypass, HSE_STARTUP_TIMEOUT) != 0) {
            goto error_recovery; // HSE failed to start
        }
    }
    
    // Step 5: Configure bus prescalers
    if (configure_bus_prescalers(config) != 0) {
        goto error_recovery;
    }
    
    // Step 6: Configure PLL if needed
    bool need_pll = (config->clock_source == STM32F103_CLOCK_SOURCE_PLL_HSI ||
                     config->clock_source == STM32F103_CLOCK_SOURCE_PLL_HSE);
    
    if (need_pll) {
        if (configure_pll(config) != 0) {
            goto error_recovery; // PLL failed to start
        }
    }
    
    // Step 7: Switch to final system clock
    if (switch_system_clock(config->clock_source) != 0) {
        goto error_recovery; // Failed to switch to target clock
    }
    
    // Step 8: Configure Clock Security System
    if (configure_css(config->enable_css) != 0) {
        goto error_recovery;
    }
    
    // Step 9: Configure SysTick
    if (configure_systick(config, expected_freq.sysclk_hz) != 0) {
        goto error_recovery;
    }
    
    // Step 10: Disable HSI if not needed (power saving)
    if (config->clock_source != STM32F103_CLOCK_SOURCE_HSI &&
        config->clock_source != STM32F103_CLOCK_SOURCE_PLL_HSI) {
        RCC_CR &= ~RCC_CR_HSION;
    }
    
    // Step 11: Update global frequency tracking
    g_clock_frequencies = expected_freq;
    g_sysclk_hz = expected_freq.sysclk_hz;
    g_clock_frequencies_valid = true;
    
    // Step 12: Initialize DWT for accurate delays
    dwt_init();
    
    // Return calculated frequencies to caller
    if (frequencies) {
        *frequencies = expected_freq;
    }
    
    return 0; // Success
    
error_recovery:
    // Restore original state on error
    RCC_CR = original_cr;
    RCC_CFGR = original_cfgr;
    FLASH_ACR = original_acr;
    
    // Try to ensure HSI is working
    RCC_CR |= RCC_CR_HSION;
    wait_for_flag_set(&RCC_CR, RCC_CR_HSIRDY, HSE_STARTUP_TIMEOUT);
    switch_system_clock(STM32F103_CLOCK_SOURCE_HSI);
    
    return -3; // Configuration failed, system restored to safe state
}

/*==============================================================================
 * Legacy Clock Configuration Functions (for backward compatibility)
 *============================================================================*/

int stm32f103_default_8mhz_internal_clock_init(){

    /* Safe fallback path: ensure HSI is running and select it as SYSCLK */
    RCC_CR |= RCC_CR_HSION;
    /* Wait until HSIRDY (HSI is expected to be available quickly) */
    wait_for(&RCC_CR,RCC_CR_HSIRDY, RCC_CR_HSIRDY, 0x1FFFFF);

    /* Disable PLL (if it was on) and HSE to clean state */
    RCC_CR &= ~RCC_CR_PLLON;
    RCC_CR &= ~RCC_CR_HSEON;
    /* Switch system clock to HSI */
    RCC_CFGR = (RCC_CFGR & ~RCC_CFGR_SW_Msk) | RCC_CFGR_SW_HSI;
    /* Wait for SWS to indicate HSI */

    wait_for(&RCC_CFGR, RCC_CFGR_SWS_Msk, RCC_CFGR_SWS_HSI, 0x1FFFFF);
    // while (((RCC_CFGR & RCC_CFGR_SWS_HSI) != RCC_CFGR_SWS_HSI) && timeout--) {
    //     __asm("nop");
    // }

    g_sysclk_hz = 8000000;
    g_clock_frequencies_valid = true;

    return 0;

}

int stm32f103_default_72mhz_clock_init(){

    uint32_t tmp;

    /* 1) RCC_CR |= RCC_CR_HSEBYP;  // only if using external clock signal (not crystal) */

    /* 2) Enable HSE and wait for HSERDY */
    RCC_CR |= RCC_CR_HSEON;
    if (wait_for(&RCC_CR, RCC_CR_HSERDY, RCC_CR_HSERDY, 0x1FFFFF)) {
        /* HSE failed to start — handle error */
        stm32f103_default_8mhz_internal_clock_init();
        return 0;
    }

    /* 3) Set Flash latency = 2WS and enable prefetch (needed for 72 MHz) */
    tmp = FLASH_ACR & ~FLASH_ACR_LATENCY_Msk;
    tmp |= FLASH_ACR_LATENCY_2;   /* 2 wait states */
    tmp |= FLASH_ACR_PRFTBE;      /* enable prefetch buffer */
    FLASH_ACR = tmp;

    /* 4) Configure prescalers: AHB = SYSCLK, APB1 = HCLK/2 (<=36MHz), APB2 = HCLK */
    tmp = RCC_CFGR & ~(RCC_CFGR_HPRE_Msk | RCC_CFGR_PPRE1_Msk | RCC_CFGR_PPRE2_Msk);
    tmp |= RCC_CFGR_PPRE1_DIV2; /* APB1 = HCLK/2 */
    RCC_CFGR = tmp;

    /* 5) Configure PLL: PLLSRC = HSE, PLLXTPRE = 0 (HSE not divided), PLLMUL = x9 */
    tmp = RCC_CFGR & ~(RCC_CFGR_PLLSRC | RCC_CFGR_PLLXTPRE | RCC_CFGR_PLLMULL_Msk);
    tmp |= RCC_CFGR_PLLSRC_HSE;    /* PLL source = HSE */
    tmp |= RCC_CFGR_PLLMULL9;      /* PLL multiplication = 9 */
    RCC_CFGR = tmp;

    /* 6) Enable PLL, wait for PLLRDY */
    RCC_CR |= RCC_CR_PLLON;
    if (wait_for(&RCC_CR, RCC_CR_PLLRDY, RCC_CR_PLLRDY, 0x1FFFFF)) {
        /* PLL failed to lock — handle error */
        stm32f103_default_8mhz_internal_clock_init();
        return 0;
    }

    /* 7) Select PLL as system clock and wait for switch */
    tmp = RCC_CFGR & ~RCC_CFGR_SW_Msk;
    tmp |= RCC_CFGR_SW_PLL;    /* SW = PLL */
    RCC_CFGR = tmp;
    /* Wait for system clock switch to complete */
    while ((RCC_CFGR & RCC_CFGR_SWS_Msk) != RCC_CFGR_SWS_PLL) { /* spin */ }

    /* 8) (Optional) turn off HSI to save power */
    RCC_CR &= ~RCC_CR_HSION;

    g_sysclk_hz = 72000000;
    g_clock_frequencies_valid = true;

    return 0;

}

/*==============================================================================
 * Pre-configured Clock Settings (Common Configurations)
 *============================================================================*/

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

void stm32f103_get_default_8mhz_hsi_config(stm32f103_clock_config_t *config) {
    if (!config) return;

    config->clock_source = STM32F103_CLOCK_SOURCE_HSI;
    config->hse_freq_hz = 0;
    config->hse_bypass = false;
    config->pll_mul = STM32F103_PLL_MUL_2; // Not used for HSI direct
    config->pll_hse_div2 = false;
    config->ahb_prescaler = STM32F103_AHB_DIV_1;
    config->apb1_prescaler = STM32F103_APB_DIV_1;
    config->apb2_prescaler = STM32F103_APB_DIV_1;
    config->adc_prescaler = STM32F103_ADC_DIV_2;
    config->flash_latency = STM32F103_FLASH_LATENCY_0;
    config->flash_prefetch_enable = false;
    config->usb_prescaler_1_5 = false;
    config->enable_css = false;
    config->enable_systick = true;
    config->systick_freq_hz = 1000;
}

/*==============================================================================
 * GPIO Helper Functions
 *============================================================================*/

uint32_t stm32f103_get_gpio_base(stm32f103_gpio_port_t port) {
    switch (port) {
        case STM32F103_GPIO_PORT_A: return GPIOA_BASE;
        case STM32F103_GPIO_PORT_B: return GPIOB_BASE;
        case STM32F103_GPIO_PORT_C: return GPIOC_BASE;
        case STM32F103_GPIO_PORT_D: return GPIOD_BASE;
        case STM32F103_GPIO_PORT_E: return GPIOE_BASE;
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

/*==============================================================================
 * Timing and Delay Functions
 *============================================================================*/

void dwt_init(void) {
    SCB_DEMCR |= SCB_DEMCR_TRCENA;          // enable trace
    DWT_CYCCNT = 0;                         // reset cycle counter
    DWT_CTRL |= DWT_CTRL_CYCCNTENA;         // enable cycle counter
}

void stm32f103_delay_us(uint32_t microseconds) {
    uint32_t cycles = (g_sysclk_hz / 1000000U) * microseconds;
    uint32_t start = DWT_CYCCNT;
    while ((DWT_CYCCNT - start) < cycles) {
        __asm__ volatile ("nop");
    }
}

void stm32f103_delay_ms(uint32_t milliseconds) {
    while (milliseconds--) {
        stm32f103_delay_us(1000);
    }
}

/*==============================================================================
 * Clock Information and Status Functions
 *============================================================================*/

uint32_t stm32f103_get_sysclk_hz(void) {
    return g_clock_frequencies.sysclk_hz;
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
    uint32_t sws = (RCC_CFGR & RCC_CFGR_SWS_Msk) >> RCC_CFGR_SWS_Pos;

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
            uint32_t pllmul = ((RCC_CFGR & RCC_CFGR_PLLMULL_Msk) >> RCC_CFGR_PLLMULL_Pos) + 2;
            uint32_t pllsrc = (RCC_CFGR & RCC_CFGR_PLLSRC) ? 1 : 0;
            uint32_t pllxtpre = (RCC_CFGR & RCC_CFGR_PLLXTPRE) ? 1 : 0;

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
    uint32_t hpre = (RCC_CFGR & RCC_CFGR_HPRE_Msk) >> RCC_CFGR_HPRE_Pos;
    uint32_t ppre1 = (RCC_CFGR & RCC_CFGR_PPRE1_Msk) >> RCC_CFGR_PPRE1_Pos;
    uint32_t ppre2 = (RCC_CFGR & RCC_CFGR_PPRE2_Msk) >> RCC_CFGR_PPRE2_Pos;
    uint32_t adcpre = (RCC_CFGR & RCC_CFGR_ADCPRE_Msk) >> RCC_CFGR_ADCPRE_Pos;

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
        bool usbpre = (RCC_CFGR & RCC_CFGR_USBPRE);
        if (!usbpre) {
            detected.usbclk_hz = (detected.sysclk_hz * 2) / 3; // PLL / 1.5
        } else {
            detected.usbclk_hz = detected.sysclk_hz; // PLL / 1
        }
    }

    g_clock_frequencies = detected;
    g_clock_frequencies_valid = true;

    return 0;
}

/*==============================================================================
 * Debug UART Functions (UART1 - PA9/PA10)
 *============================================================================*/

static volatile bool uart_debug_initialized = false;

int stm32f103_uart_debug_init(void) {
    if (uart_debug_initialized) return 0;

    // Enable UART1 and GPIOA clocks using utility function
    stm32f103_enable_peripheral_clock(0x18, RCC_APB2ENR_USART1EN); // APB2ENR offset
    stm32f103_enable_peripheral_clock(0x18, RCC_APB2ENR_IOPAEN);

    // Configure PA9 (TX) as alternate function push-pull, 50MHz
    stm32f103_configure_gpio_pin(GPIOA_BASE, 9, GPIO_MODE_OUTPUT_50M, GPIO_CNF_OUTPUT_AF_PUSHPULL);
    
    // Configure PA10 (RX) as input floating
    stm32f103_configure_gpio_pin(GPIOA_BASE, 10, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOATING);

    // Configure UART1: 115200 baud using utility function
    uint32_t pclk2_hz = g_clock_frequencies_valid ? g_clock_frequencies.pclk2_hz : STM32F103_HSI_FREQ_HZ;
    USART1_BRR = stm32f103_calculate_uart_brr(pclk2_hz, 115200);

    // Enable TX, RX, and UART
    USART1_CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;

    uart_debug_initialized = true;
    return 0;
}

void stm32f103_uart_debug_print(const char* str) {
    if (!uart_debug_initialized) return;

    while (*str) {
        // Wait for transmit register empty
        while (!(USART1_SR & USART_SR_TXE));
        USART1_DR = *str++;
    }
}

void stm32f103_uart_debug_print_num(const char* str, uint32_t num) {
    if (!uart_debug_initialized) return;

    stm32f103_uart_debug_print(str);

    // Simple hex conversion
    char hex_str[13];  // "0x" + 8 hex digits + "\r\n" + '\0'
    char hex_chars[] = "0123456789ABCDEF";

    hex_str[0] = '0';
    hex_str[1] = 'x';

    for (int i = 7; i >= 0; i--) {
        hex_str[2 + (7-i)] = hex_chars[(num >> (i*4)) & 0xF];
    }
    hex_str[10] = '\r';
    hex_str[11] = '\n';
    hex_str[12] = '\0';

    stm32f103_uart_debug_print(hex_str);
}

void stm32f103_uart_debug_deinit(void) {
    if (!uart_debug_initialized) return;
    
    // Disable UART
    USART1_CR1 = 0;
    
    // Reset GPIO pins to input floating (reset state)
    GPIO_CRH(GPIOA_BASE) &= ~(0xFF << 4); // Clear PA9 and PA10 config
    GPIO_CRH(GPIOA_BASE) |= (0x44 << 4);  // Set both to input floating
    
    uart_debug_initialized = false;
}

/*==============================================================================
 * Additional Utility Functions
 *============================================================================*/

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
        case 0x0: return STM32F103_CLOCK_SOURCE_HSI;
        case 0x1: return STM32F103_CLOCK_SOURCE_HSE;
        case 0x2: {
            // PLL is active, determine if it's HSI or HSE based
            bool pll_src_hse = (RCC_CFGR & RCC_CFGR_PLLSRC);
            return pll_src_hse ? STM32F103_CLOCK_SOURCE_PLL_HSE : STM32F103_CLOCK_SOURCE_PLL_HSI;
        }
        default: return STM32F103_CLOCK_SOURCE_HSI; // Fallback
    }
}

int stm32f103_enable_peripheral_clock(uint32_t rcc_register_offset, uint32_t enable_bit) {
    volatile uint32_t *rcc_reg = (volatile uint32_t *)(RCC_BASE + rcc_register_offset);
    *rcc_reg |= enable_bit;
    
    // Small delay to ensure clock is stable
    for (volatile int i = 0; i < 10; i++) { __asm("nop"); }
    
    return 0;
}

int stm32f103_disable_peripheral_clock(uint32_t rcc_register_offset, uint32_t enable_bit) {
    volatile uint32_t *rcc_reg = (volatile uint32_t *)(RCC_BASE + rcc_register_offset);
    *rcc_reg &= ~enable_bit;
    return 0;
}

void stm32f103_system_reset(void) {
    // Request system reset through AIRCR register
    volatile uint32_t *AIRCR = (volatile uint32_t *)0xE000ED0C;
    *AIRCR = (0x5FA << 16) | (1 << 2); // VECTKEY + SYSRESETREQ
    
    // Should not reach here, but just in case
    while (1) { __asm("nop"); }
}

uint32_t stm32f103_calculate_uart_brr(uint32_t peripheral_clock_hz, uint32_t baudrate) {
    return (peripheral_clock_hz + (baudrate / 2)) / baudrate; // Round to nearest
}

void stm32f103_configure_gpio_pin(uint32_t gpio_base, uint8_t pin, uint8_t mode, uint8_t cnf) {
    if (pin > 15) return; // Invalid pin
    
    volatile uint32_t *cr_reg = (pin < 8) ? 
        (volatile uint32_t *)(gpio_base + 0x00) : // CRL
        (volatile uint32_t *)(gpio_base + 0x04);  // CRH
    
    uint8_t shift = (pin % 8) * 4;
    uint32_t mask = 0xF << shift;
    uint32_t config = ((cnf << 2) | mode) << shift;
    
    *cr_reg = (*cr_reg & ~mask) | config;
}
