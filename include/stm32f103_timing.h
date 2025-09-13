#ifndef STM32F103_TIMING_H
#define STM32F103_TIMING_H

#include <stdint.h>
#include <stdbool.h>

/**
 * @file stm32f103_timing.h
 * @brief STM32F103 Timing and Delay Functions
 * 
 * This module provides precise timing and delay functions for STM32F103.
 * It depends on the clock module for frequency information and uses 
 * SysTick and DWT for accurate timing.
 */

/*==============================================================================
 * Initialization
 *============================================================================*/

/**
 * @brief Initialize timing subsystem
 * 
 * Initializes SysTick and DWT (Data Watchpoint and Trace) for precise timing.
 * This function should be called after clock initialization.
 * 
 * @return 0 on success, negative on error
 */
int stm32f103_timing_init(void);

/**
 * @brief Initialize DWT (Data Watchpoint and Trace) for cycle counting
 * 
 * Enables DWT cycle counter for microsecond precision delays.
 * Called automatically by stm32f103_timing_init().
 */
void stm32f103_dwt_init(void);

/*==============================================================================
 * Delay Functions
 *============================================================================*/

/**
 * @brief Microsecond delay using DWT cycle counter
 * 
 * Provides accurate microsecond delays using the DWT cycle counter.
 * Requires DWT to be initialized first.
 * 
 * @param microseconds Number of microseconds to delay
 */
void stm32f103_delay_us(uint32_t microseconds);

/**
 * @brief Millisecond delay using SysTick
 * 
 * Provides millisecond delays using SysTick timer.
 * 
 * @param milliseconds Number of milliseconds to delay
 */
void stm32f103_delay_ms(uint32_t milliseconds);

/*==============================================================================
 * Time Measurement
 *============================================================================*/

/**
 * @brief Get current DWT cycle count
 * @return Current cycle count value
 */
uint32_t stm32f103_get_cycles(void);

/**
 * @brief Convert cycles to microseconds
 * @param cycles Number of CPU cycles
 * @return Equivalent time in microseconds
 */
uint32_t stm32f103_cycles_to_us(uint32_t cycles);

/**
 * @brief Convert microseconds to cycles
 * @param microseconds Time in microseconds
 * @return Equivalent number of CPU cycles
 */
uint32_t stm32f103_us_to_cycles(uint32_t microseconds);

/*==============================================================================
 * SysTick Functions
 *============================================================================*/

/**
 * @brief Get current SysTick tick count
 * @return Current tick count (increments every millisecond)
 */
uint32_t stm32f103_get_tick(void);

/**
 * @brief Calculate elapsed time in milliseconds
 * @param start_tick Starting tick value
 * @return Elapsed time in milliseconds
 */
uint32_t stm32f103_elapsed_ms(uint32_t start_tick);

/**
 * @brief Check if timeout has occurred
 * @param start_tick Starting tick value
 * @param timeout_ms Timeout period in milliseconds
 * @return true if timeout occurred, false otherwise
 */
bool stm32f103_timeout_occurred(uint32_t start_tick, uint32_t timeout_ms);

/**
 * @brief Initialize SysTick timer for timing functions
 * STM32-specific initialization function
 */
void nhal_systick_init(void);

#endif /* STM32F103_TIMING_H */