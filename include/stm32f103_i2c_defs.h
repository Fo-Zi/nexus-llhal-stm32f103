#ifndef STM32F103_I2C_DEFS_H
#define STM32F103_I2C_DEFS_H

#include <stdint.h>
#include <stdbool.h>
#include "nhal_i2c_types.h"
#include "nhal_pin_types.h"
#include "nhal_pin_stm32f103.h"

// STM32F103 I2C peripheral identifiers
typedef enum {
    STM32F103_I2C_1 = 1,
    STM32F103_I2C_2 = 2
} stm32f103_i2c_id_t;

// STM32F103 I2C duty cycle (for fast mode timing calculation)
typedef enum {
    STM32F103_I2C_DUTY_2 = 0,       // 2:1 duty cycle
    STM32F103_I2C_DUTY_16_9 = 1     // 16:9 duty cycle
} stm32f103_i2c_duty_t;

/**
 * @file stm32f103_i2c_defs.h
 * @brief STM32F103-specific I2C definitions and structures
 */

/**
 * @brief STM32F103 I2C context implementation
 * 
 * Contains only the truly stateful information needed for I2C operations.
 * Static information like base address is derived from i2c_id as needed.
 */
struct nhal_i2c_context {
    stm32f103_i2c_id_t i2c_id;       // Which STM32F103 I2C peripheral (1 or 2)
    bool is_initialized;             // Whether I2C hardware is initialized
    bool is_configured;              // Whether I2C is configured and ready
};

/**
 * @brief STM32F103 I2C implementation-specific configuration
 */
struct nhal_i2c_impl_config {
    bool use_remap;         // Use alternate pin mapping
    uint32_t clock_freq_hz;
    stm32f103_i2c_duty_t duty_cycle;
    uint32_t timeout_ms;
};

#endif /* STM32F103_I2C_DEFS_H */