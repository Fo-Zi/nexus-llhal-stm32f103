#ifndef STM32F103_UART_H
#define STM32F103_UART_H

#include <stdint.h>
#include <stdbool.h>
#include "nhal_uart_types.h"
#include "stm32f103_uart_defs.h"
#include "nhal_pin_types.h"
#include "stm32f103_gpio.h"

/**
 * @file stm32f103_uart.h
 * @brief STM32F103 UART hardware definitions and peripheral control
 */

#ifdef __cplusplus
extern "C" {
#endif

/*==============================================================================
 * STM32F103 UART Hardware Definitions
 *============================================================================*/

// UART peripheral identifiers are defined in stm32f103_uart_defs.h

// All UART type definitions (enums, structs) are now in stm32f103_uart_defs.h

/*==============================================================================
 * STM32F103 UART Context and Implementation Config (for NHAL)
 *============================================================================*/

// Context and config structures are defined in stm32f103_uart_defs.h

/*==============================================================================
 * STM32F103 UART Functions
 *============================================================================*/

// Low-level STM32F103 UART functions (used internally by NHAL implementation)
uint32_t stm32f103_uart_get_base_address(stm32f103_uart_id_t uart_id);
uint32_t stm32f103_uart_calculate_brr(uint32_t peripheral_clock_hz, uint32_t baudrate);

// Simple NHAL helper functions (for NHAL implementation use only)
int stm32f103_uart_init_simple(stm32f103_uart_id_t uart_id, const struct nhal_uart_config *nhal_config);
int stm32f103_uart_send_byte_timeout(stm32f103_uart_id_t uart_id, uint8_t data, uint32_t timeout_ms);
int stm32f103_uart_receive_byte_timeout(stm32f103_uart_id_t uart_id, uint8_t *data, uint32_t timeout_ms);
bool stm32f103_uart_is_tx_ready(stm32f103_uart_id_t uart_id);
bool stm32f103_uart_is_rx_data_available(stm32f103_uart_id_t uart_id);

#ifdef __cplusplus
}
#endif

#endif /* STM32F103_UART_H */