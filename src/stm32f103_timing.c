#include "stm32f103_timing.h"
#include "stm32f103_clock.h"
#include "stm32f103_registers.h"

/**
 * @file stm32f103_timing.c
 * @brief STM32F103 Timing and Delay Functions Implementation
 */

/*==============================================================================
 * Private Variables
 *============================================================================*/

static bool timing_initialized = false;
static volatile uint32_t systick_counter = 0;

/*==============================================================================
 * Private Functions
 *============================================================================*/



/*==============================================================================
 * Public Functions - Initialization
 *============================================================================*/

int stm32f103_timing_init(void) {
    // Initialize DWT for cycle counting
    stm32f103_dwt_init();
    
    // Get current system frequency to configure SysTick
    stm32f103_clock_frequencies_t freq;
    int result = stm32f103_get_clock_frequencies(&freq);
    if (result != 0) {
        return result;
    }
    
    // Configure SysTick for 1ms interrupts
    uint32_t reload = (freq.sysclk_hz / 1000) - 1; // 1ms tick
    if (reload > 0xFFFFFF) {
        return -1; // Reload value too large
    }
    
    SYSTICK_RVR = reload;
    SYSTICK_CVR = 0; // Clear current value
    SYSTICK_CSR = SYSTICK_CSR_CLKSOURCE | SYSTICK_CSR_TICKINT | SYSTICK_CSR_ENABLE;
    
    timing_initialized = true;
    return 0;
}

void stm32f103_dwt_init(void) {
    SCB_DEMCR |= SCB_DEMCR_TRCENA;          // Enable trace
    DWT_CYCCNT = 0;                         // Reset cycle counter
    DWT_CTRL |= DWT_CTRL_CYCCNTENA;         // Enable cycle counter
}

/*==============================================================================
 * Public Functions - Delay Functions
 *============================================================================*/

void stm32f103_delay_us(uint32_t microseconds) {
    if (microseconds == 0) return;
    
    // Get current system clock frequency
    stm32f103_clock_frequencies_t freq;
    int result = stm32f103_get_clock_frequencies(&freq);
    if (result != 0) {
        // Fallback: assume 72MHz
        freq.sysclk_hz = 72000000;
    }
    
    // Calculate required cycles for the delay
    uint64_t cycles_64 = (uint64_t)freq.sysclk_hz * microseconds;
    uint32_t cycles = (uint32_t)(cycles_64 / 1000000ULL);
    
    uint32_t start = DWT_CYCCNT;
    
    while ((DWT_CYCCNT - start) < cycles) {
        __asm__ volatile ("nop");
    }
}

void stm32f103_delay_ms(uint32_t milliseconds) {
    if (!timing_initialized) {
        // Fallback: use microsecond delay
        while (milliseconds--) {
            stm32f103_delay_us(1000);
        }
        return;
    }
    
    // Use SysTick-based delay
    uint32_t start_tick = systick_counter;
    while ((systick_counter - start_tick) < milliseconds) {
        __asm__ volatile ("wfi"); // Wait for interrupt (power saving)
    }
}

/*==============================================================================
 * Public Functions - Time Measurement
 *============================================================================*/

uint32_t stm32f103_get_cycles(void) {
    return DWT_CYCCNT;
}

uint32_t stm32f103_cycles_to_us(uint32_t cycles) {
    stm32f103_clock_frequencies_t freq;
    int result = stm32f103_get_clock_frequencies(&freq);
    if (result != 0) {
        // Fallback: assume 72MHz
        freq.sysclk_hz = 72000000;
    }
    
    return (uint64_t)cycles * 1000000ULL / freq.sysclk_hz;
}

uint32_t stm32f103_us_to_cycles(uint32_t microseconds) {
    stm32f103_clock_frequencies_t freq;
    int result = stm32f103_get_clock_frequencies(&freq);
    if (result != 0) {
        // Fallback: assume 72MHz
        freq.sysclk_hz = 72000000;
    }
    
    return (uint64_t)freq.sysclk_hz * microseconds / 1000000ULL;
}

/*==============================================================================
 * Public Functions - SysTick Functions
 *============================================================================*/

uint32_t stm32f103_get_tick(void) {
    return systick_counter;
}

uint32_t stm32f103_elapsed_ms(uint32_t start_tick) {
    return systick_counter - start_tick;
}

bool stm32f103_timeout_occurred(uint32_t start_tick, uint32_t timeout_ms) {
    return (systick_counter - start_tick) >= timeout_ms;
}