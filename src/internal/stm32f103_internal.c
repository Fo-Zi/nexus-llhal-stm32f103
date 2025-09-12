#include "stm32f103_internal.h"

// Cache for calculated clock frequencies to avoid repeated register reads
static struct {
    uint32_t sysclk_hz;
    uint32_t ahb_hz;
    uint32_t apb1_hz;
    uint32_t apb2_hz;
    bool cached;
} clock_cache = {0};

static uint32_t stm32f103_calculate_pll_freq(void) {
    // Read PLL configuration from RCC_CFGR
    uint32_t cfgr = RCC_CFGR;
    
    // PLL source (bit 16)
    bool pll_src_hse = (cfgr >> 16) & 0x1;
    
    // PLL multiplication factor (bits 21:18)
    uint32_t pllmul = ((cfgr >> 18) & 0xF) + 2; // PLLMUL[3:0] + 2
    if (pllmul > 16) pllmul = 16; // Maximum is 16
    
    uint32_t pll_input;
    if (pll_src_hse) {
        pll_input = STM32F103_HSI_FREQ_HZ; // Assume 8MHz external crystal
        // Check HSE prescaler (bit 17)
        if ((cfgr >> 17) & 0x1) {
            pll_input = pll_input / 2; // HSE/2 when PLLXTPRE=1
        }
    } else {
        pll_input = STM32F103_HSI_FREQ_HZ / 2; // HSI/2
    }
    
    return pll_input * pllmul;
}

// Use external implementation from stm32f103_helpers.c
extern uint32_t stm32f103_get_sysclk_hz(void);

uint32_t stm32f103_get_ahb_hz(void) {
    if (clock_cache.cached) {
        return clock_cache.ahb_hz;
    }
    
    uint32_t sysclk = stm32f103_get_sysclk_hz();
    
    // Read AHB prescaler from RCC_CFGR (bits 7:4) 
    uint32_t hpre = (RCC_CFGR >> 4) & 0xF;
    
    uint32_t ahb_div;
    if (hpre < 8) {
        ahb_div = 1; // No division
    } else {
        ahb_div = 2 << (hpre - 7); // 2^(hpre-7) when hpre >= 8
    }
    
    uint32_t ahb_clk = sysclk / ahb_div;
    clock_cache.ahb_hz = ahb_clk;
    return ahb_clk;
}

uint32_t stm32f103_get_apb1_hz(void) {
    if (clock_cache.cached) {
        return clock_cache.apb1_hz;
    }
    
    uint32_t ahb_clk = stm32f103_get_ahb_hz();
    
    // Read APB1 prescaler from RCC_CFGR (bits 10:8)
    uint32_t ppre1 = (RCC_CFGR >> 8) & 0x7;
    
    uint32_t apb1_div;
    if (ppre1 < 4) {
        apb1_div = 1; // No division
    } else {
        apb1_div = 2 << (ppre1 - 3); // 2^(ppre1-3) when ppre1 >= 4
    }
    
    uint32_t apb1_clk = ahb_clk / apb1_div;
    clock_cache.apb1_hz = apb1_clk;
    return apb1_clk;
}

uint32_t stm32f103_get_apb2_hz(void) {
    if (clock_cache.cached) {
        return clock_cache.apb2_hz;
    }
    
    uint32_t ahb_clk = stm32f103_get_ahb_hz();
    
    // Read APB2 prescaler from RCC_CFGR (bits 13:11)
    uint32_t ppre2 = (RCC_CFGR >> 11) & 0x7;
    
    uint32_t apb2_div;
    if (ppre2 < 4) {
        apb2_div = 1; // No division
    } else {
        apb2_div = 2 << (ppre2 - 3); // 2^(ppre2-3) when ppre2 >= 4
    }
    
    uint32_t apb2_clk = ahb_clk / apb2_div;
    clock_cache.apb2_hz = apb2_clk;
    clock_cache.cached = true; // Mark cache as complete
    return apb2_clk;
}

bool stm32f103_is_pin_available(stm32f103_package_t package, stm32f103_pin_t pin) {
    switch (package) {
        case STM32F103_PACKAGE_48PIN_LQFP:
            // 48-pin package limitations (Blue Pill - STM32F103C8T6)
            switch (pin.port) {
                case STM32F103_GPIO_PORT_A:
                    return true; // PA0-PA15 all available
                    
                case STM32F103_GPIO_PORT_B:
                    return true; // PB0-PB15 all available
                    
                case STM32F103_GPIO_PORT_C:
                    // Only PC13, PC14, PC15 available in 48-pin
                    return (pin.pin >= STM32F103_GPIO_PIN_13);
                    
                case STM32F103_GPIO_PORT_D:
                    // Only PD0, PD1 available (OSC pins)
                    return (pin.pin <= STM32F103_GPIO_PIN_1);
                    
                case STM32F103_GPIO_PORT_E:
                    return false; // Port E not available in 48-pin
                    
                default:
                    return false;
            }
            
        case STM32F103_PACKAGE_64PIN_LQFP:
            // 64-pin package has more pins available
            switch (pin.port) {
                case STM32F103_GPIO_PORT_A:
                case STM32F103_GPIO_PORT_B:
                    return true; // All pins available
                    
                case STM32F103_GPIO_PORT_C:
                    return true; // PC0-PC15 all available
                    
                case STM32F103_GPIO_PORT_D:
                    return (pin.pin <= STM32F103_GPIO_PIN_2); // PD0-PD2
                    
                case STM32F103_GPIO_PORT_E:
                    return false; // Still not available in 64-pin
                    
                default:
                    return false;
            }
            
        case STM32F103_PACKAGE_100PIN_LQFP:
            // 100-pin package has all pins available
            switch (pin.port) {
                case STM32F103_GPIO_PORT_A:
                case STM32F103_GPIO_PORT_B:
                case STM32F103_GPIO_PORT_C:
                case STM32F103_GPIO_PORT_D:
                    return true; // All pins available
                    
                case STM32F103_GPIO_PORT_E:
                    return true; // PE0-PE15 available
                    
                default:
                    return false;
            }
            
        default:
            return false;
    }
}