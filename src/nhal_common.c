#include "nhal_common.h"
#include "stm32f103_helpers.h"
#include "internal/stm32f103_internal.h"

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
    // Simple implementation - could be enhanced with SysTick interrupt
    // For now, return a basic timestamp
    return system_uptime_us;
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