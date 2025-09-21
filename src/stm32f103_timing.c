#include "stm32f103_timing.h"
#include "stm32f103_clock.h"
#include "stm32f103_registers.h"
#include <stdint.h>

/**
 * @file stm32f103_timing.c
 * @brief STM32F103 Timing and Delay Functions Implementation
 */

/*==============================================================================
 * Private Variables
 *============================================================================*/

static bool timing_initialized = false;
static volatile uint32_t systick_counter = 0;
static uint32_t stored_sysclk_hz = 8000000; // Default to 8MHz HSI

/*==============================================================================
 * Private Functions
 *============================================================================*/
 // SysTick interrupt handler - called every 1ms
 void SysTick_Handler(void) {
     systick_counter += 1000; // Increment by 1ms (1000 microseconds)
 }

 void stm32f103_systick_init(void);

/*==============================================================================
 * Public Functions - Initialization
 *============================================================================*/

int stm32f103_timing_init(void) {
    // Get current system frequency and store it locally
    stm32f103_clock_frequencies_t freq;
    int result = stm32f103_get_clock_frequencies(&freq);
    if (result != 0) {
        return result;
    }

    // Store frequency for delay functions
    stored_sysclk_hz = freq.sysclk_hz;

    // Initialize DWT for cycle counting
    stm32f103_dwt_init();
    stm32f103_systick_init();

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
    // Use 64-bit arithmetic to prevent integer overflow
    uint64_t cycles = ((uint64_t)stored_sysclk_hz * microseconds) / 1000000ULL;
    uint32_t start = DWT_CYCCNT;
    while ((DWT_CYCCNT - start) < (uint32_t)cycles) {
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
    while ((systick_counter - start_tick) < (milliseconds * 1000)) {
        __asm__ volatile ("wfi"); // Wait for interrupt
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

    return (uint32_t)(((uint64_t)cycles * 1000000ULL) / freq.sysclk_hz);
}

uint32_t stm32f103_us_to_cycles(uint32_t microseconds) {
    stm32f103_clock_frequencies_t freq;
    int result = stm32f103_get_clock_frequencies(&freq);
    if (result != 0) {
        // Fallback: assume 72MHz
        freq.sysclk_hz = 72000000;
    }

    return (uint32_t)(((uint64_t)freq.sysclk_hz * microseconds) / 1000000ULL);
}

/*==============================================================================
 * Public Functions - SysTick Functions
 *============================================================================*/

 // Initialize SysTick for timestamp tracking (should be called during system init)
 void stm32f103_systick_init(void) {
     // Configure SysTick for 1ms interrupts
     SYSTICK_RVR = (stored_sysclk_hz / 1000) - 1;
     SYSTICK_CVR = 0;
     SYSTICK_CSR = SYSTICK_CSR_ENABLE | SYSTICK_CSR_CLKSOURCE | SYSTICK_CSR_TICKINT;
 }


uint32_t stm32f103_get_tick(void) {
    return systick_counter;
}

uint32_t stm32f103_elapsed_ms(uint32_t start_tick) {
    return systick_counter - start_tick;
}

bool stm32f103_timeout_occurred(uint32_t start_tick, uint32_t timeout_ms) {
    return (systick_counter - start_tick) >= timeout_ms;
}
