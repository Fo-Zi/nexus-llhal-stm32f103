#include "nhal_common.h"
#include "stm32f103_timing.h"
#include "stm32f103_clock.h"
#include "stm32f103_registers.h"

// Static variables for timestamp tracking
static volatile uint64_t system_uptime_us = 0;
static volatile uint32_t last_systick = 0;

void nhal_delay_microseconds(uint32_t microseconds) {
    stm32f103_delay_us(microseconds);
}

void nhal_delay_milliseconds(uint32_t milliseconds) {
    stm32f103_delay_ms(milliseconds);
}

uint64_t nhal_get_timestamp_microseconds(void) {
    // Use DWT cycle counter for microsecond precision
    // This provides much better resolution than SysTick for DHT11 timing
    static uint64_t base_time_us = 0;
    static uint32_t last_cycle_count = 0;
    static bool first_call = true;
    
    uint32_t current_cycles = DWT_CYCCNT;
    uint32_t sysclk_hz = stm32f103_get_sysclk_hz();
    
    if (first_call) {
        first_call = false;
        last_cycle_count = current_cycles;
        return base_time_us;
    }
    
    // Calculate elapsed cycles (handle 32-bit overflow)
    uint32_t elapsed_cycles = current_cycles - last_cycle_count;
    
    // Convert cycles to microseconds
    uint64_t elapsed_us = ((uint64_t)elapsed_cycles * 1000000ULL) / sysclk_hz;
    
    base_time_us += elapsed_us;
    last_cycle_count = current_cycles;
    
    return base_time_us;
}

uint32_t nhal_get_timestamp_milliseconds(void) {
    return (uint32_t)(nhal_get_timestamp_microseconds() / 1000);
}

// Initialize SysTick for timestamp tracking (should be called during system init)
void nhal_systick_init(void) {
    uint32_t sysclk = stm32f103_get_sysclk_hz();
    
    // Configure SysTick for 1ms interrupts
    SYSTICK_RVR = (sysclk / 1000) - 1;
    SYSTICK_CVR = 0;
    SYSTICK_CSR = SYSTICK_CSR_ENABLE | SYSTICK_CSR_CLKSOURCE | SYSTICK_CSR_TICKINT;
}

// SysTick interrupt handler - called every 1ms
void SysTick_Handler(void) {
    system_uptime_us += 1000; // Increment by 1ms (1000 microseconds)
}