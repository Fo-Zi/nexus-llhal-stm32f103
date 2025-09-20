#include "nhal_common.h"
#include "stm32f103_timing.h"
#include "stm32f103_clock.h"
#include "stm32f103_registers.h"

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
