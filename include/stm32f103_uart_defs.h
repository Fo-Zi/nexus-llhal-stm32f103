/**
 * @file stm32f103_uart_defs.h
 * @brief STM32F103-specific UART definitions and structures
 */

#ifndef STM32F103_UART_DEFS_H
#define STM32F103_UART_DEFS_H

#include <stdint.h>
#include <stdbool.h>
#include "stm32f103_gpio_defs.h"

/**
 * @brief STM32F103 UART peripheral identifiers
 */
typedef enum {
    STM32F103_UART_1 = 1,
    STM32F103_UART_2 = 2,
    STM32F103_UART_3 = 3
} stm32f103_uart_id_t;

/**
 * @brief STM32F103 UART flow control options
 */
typedef enum {
    STM32F103_UART_FLOWCTRL_NONE = 0,
    STM32F103_UART_FLOWCTRL_RTS = 1,
    STM32F103_UART_FLOWCTRL_CTS = 2,
    STM32F103_UART_FLOWCTRL_RTS_CTS = 3
} stm32f103_uart_flowctrl_t;

/**
 * @brief STM32F103 UART context implementation
 * 
 * Contains only the truly stateful information needed for UART operations.
 * Static information like base address is derived from uart_id as needed.
 */
struct nhal_uart_context {
    stm32f103_uart_id_t uart_id;      // Which STM32F103 UART peripheral (1, 2, or 3)
    bool is_initialized;              // Whether UART hardware is initialized
    bool is_configured;               // Whether UART is configured and ready
};

/**
 * @brief STM32F103 UART implementation-specific configuration
 * 
 * This contains only STM32-specific settings that can't be derived from 
 * generic NHAL parameters. The NHAL layer provides generic settings 
 * (baudrate, parity, data bits, stop bits) which the implementation 
 * translates to STM32 register values.
 */
struct nhal_uart_impl_config {
    bool use_remap;         // Use alternate pin mapping for the UART
    uint32_t timeout_ms;    // Operation timeout
};



#endif /* STM32F103_UART_DEFS_H */