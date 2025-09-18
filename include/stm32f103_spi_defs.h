#ifndef STM32F103_SPI_DEFS_H
#define STM32F103_SPI_DEFS_H

#include <stdint.h>
#include <stdbool.h>
#include "nhal_spi_types.h"
#include "nhal_pin_types.h"

// Forward declaration for STM32F103 SPI types
typedef enum {
    STM32F103_SPI_1 = 1,
    STM32F103_SPI_2 = 2,
    STM32F103_SPI_3 = 3
} stm32f103_spi_id_t;

/**
 * @brief STM32F103 SPI clock prescaler (hardware-specific)
 */
typedef enum {
    STM32F103_SPI_PRESCALER_2 = 0,
    STM32F103_SPI_PRESCALER_4 = 1,
    STM32F103_SPI_PRESCALER_8 = 2,
    STM32F103_SPI_PRESCALER_16 = 3,
    STM32F103_SPI_PRESCALER_32 = 4,
    STM32F103_SPI_PRESCALER_64 = 5,
    STM32F103_SPI_PRESCALER_128 = 6,
    STM32F103_SPI_PRESCALER_256 = 7
} stm32f103_spi_prescaler_t;

/**
 * @file stm32f103_spi_defs.h
 * @brief STM32F103-specific SPI definitions and structures
 */

/**
 * @brief STM32F103 SPI context implementation
 * 
 * Contains only the truly stateful information needed for SPI operations.
 * Static information like base address is derived from spi_id as needed.
 */
struct nhal_spi_context {
    stm32f103_spi_id_t spi_id;       // Which STM32F103 SPI peripheral (1, 2, or 3)
    bool is_initialized;             // Whether SPI hardware is initialized
    bool is_configured;              // Whether SPI is configured and ready
};

/**
 * @brief STM32F103 SPI implementation-specific configuration
 */
struct nhal_spi_impl_config {
    bool use_remap;         // Use alternate pin mapping
    uint32_t clock_freq_hz;
    bool use_hardware_cs;   // Use hardware CS control
    uint32_t timeout_ms;
};

#endif /* STM32F103_SPI_DEFS_H */