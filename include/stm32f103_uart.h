#ifndef STM32F103_UART_H
#define STM32F103_UART_H

#include <stdint.h>
#include <stdbool.h>
#include "stm32f103_gpio.h"

/**
 * @file stm32f103_uart.h
 * @brief STM32F103 UART Interface Module
 * 
 * This module provides configurable UART functionality for STM32F103.
 * Unlike the previous hardcoded implementation, this allows flexible
 * pin assignment and UART peripheral selection.
 */

/*==============================================================================
 * UART Types and Definitions
 *============================================================================*/

/**
 * @brief UART peripheral selection
 */
typedef enum {
    STM32F103_UART_1 = 1,
    STM32F103_UART_2 = 2,
    STM32F103_UART_3 = 3
} stm32f103_uart_peripheral_t;

/**
 * @brief UART word length
 */
typedef enum {
    STM32F103_UART_WORDLEN_8B = 0,      /**< 8 data bits */
    STM32F103_UART_WORDLEN_9B = 1       /**< 9 data bits */
} stm32f103_uart_wordlen_t;

/**
 * @brief UART stop bits
 */
typedef enum {
    STM32F103_UART_STOPBITS_1 = 0,      /**< 1 stop bit */
    STM32F103_UART_STOPBITS_0_5 = 1,    /**< 0.5 stop bits */
    STM32F103_UART_STOPBITS_2 = 2,      /**< 2 stop bits */
    STM32F103_UART_STOPBITS_1_5 = 3     /**< 1.5 stop bits */
} stm32f103_uart_stopbits_t;

/**
 * @brief UART parity
 */
typedef enum {
    STM32F103_UART_PARITY_NONE = 0,     /**< No parity */
    STM32F103_UART_PARITY_EVEN = 1,     /**< Even parity */
    STM32F103_UART_PARITY_ODD = 2       /**< Odd parity */
} stm32f103_uart_parity_t;

/**
 * @brief UART flow control
 */
typedef enum {
    STM32F103_UART_FLOWCTRL_NONE = 0,   /**< No flow control */
    STM32F103_UART_FLOWCTRL_RTS = 1,    /**< RTS flow control */
    STM32F103_UART_FLOWCTRL_CTS = 2,    /**< CTS flow control */
    STM32F103_UART_FLOWCTRL_RTS_CTS = 3 /**< RTS and CTS flow control */
} stm32f103_uart_flowctrl_t;

/**
 * @brief UART pin configuration
 */
typedef struct {
    stm32f103_gpio_port_t tx_port;      /**< TX pin port */
    stm32f103_gpio_pin_t tx_pin;        /**< TX pin number */
    stm32f103_gpio_port_t rx_port;      /**< RX pin port */
    stm32f103_gpio_pin_t rx_pin;        /**< RX pin number */
    
    /* Optional flow control pins */
    bool use_flow_control;               /**< Enable flow control pins */
    stm32f103_gpio_port_t rts_port;     /**< RTS pin port (if used) */
    stm32f103_gpio_pin_t rts_pin;       /**< RTS pin number (if used) */
    stm32f103_gpio_port_t cts_port;     /**< CTS pin port (if used) */
    stm32f103_gpio_pin_t cts_pin;       /**< CTS pin number (if used) */
} stm32f103_uart_pins_t;

/**
 * @brief Comprehensive UART configuration
 */
typedef struct {
    stm32f103_uart_peripheral_t peripheral;    /**< UART peripheral to use */
    stm32f103_uart_pins_t pins;                /**< Pin configuration */
    
    /* Communication parameters */
    uint32_t baudrate;                          /**< Baudrate (e.g., 115200) */
    stm32f103_uart_wordlen_t word_length;      /**< Word length */
    stm32f103_uart_stopbits_t stop_bits;       /**< Stop bits */
    stm32f103_uart_parity_t parity;            /**< Parity */
    stm32f103_uart_flowctrl_t flow_control;    /**< Flow control */
    
    /* Optional features */
    bool enable_tx;                             /**< Enable transmitter */
    bool enable_rx;                             /**< Enable receiver */
    bool enable_interrupts;                     /**< Enable RX/TX interrupts */
} stm32f103_uart_config_t;

/**
 * @brief UART handle for runtime operations
 */
typedef struct {
    stm32f103_uart_peripheral_t peripheral;    /**< UART peripheral */
    uint32_t base_address;                      /**< UART base address */
    stm32f103_uart_config_t config;            /**< Current configuration */
    bool initialized;                           /**< Initialization status */
} stm32f103_uart_handle_t;

/*==============================================================================
 * UART Initialization and Configuration
 *============================================================================*/

/**
 * @brief Initialize UART with given configuration
 * @param handle UART handle to initialize
 * @param config UART configuration
 * @return 0 on success, negative on error
 */
int stm32f103_uart_init(stm32f103_uart_handle_t *handle, 
                        const stm32f103_uart_config_t *config);

/**
 * @brief Deinitialize UART
 * @param handle UART handle to deinitialize
 * @return 0 on success, negative on error
 */
int stm32f103_uart_deinit(stm32f103_uart_handle_t *handle);

/**
 * @brief Reconfigure UART parameters (baudrate, parity, etc.)
 * @param handle UART handle
 * @param config New configuration
 * @return 0 on success, negative on error
 */
int stm32f103_uart_reconfigure(stm32f103_uart_handle_t *handle,
                               const stm32f103_uart_config_t *config);

/*==============================================================================
 * Predefined Configurations
 *============================================================================*/

/**
 * @brief Get default debug UART configuration (UART1, PA9/PA10, 115200 baud)
 * @param config Output configuration structure
 */
void stm32f103_uart_get_default_debug_config(stm32f103_uart_config_t *config);

/**
 * @brief Get configuration for UART1 on PA9/PA10
 * @param config Output configuration structure
 * @param baudrate Desired baudrate
 */
void stm32f103_uart_get_uart1_pa9_pa10_config(stm32f103_uart_config_t *config, uint32_t baudrate);

/**
 * @brief Get configuration for UART1 on PB6/PB7 (alternate pins)
 * @param config Output configuration structure
 * @param baudrate Desired baudrate
 */
void stm32f103_uart_get_uart1_pb6_pb7_config(stm32f103_uart_config_t *config, uint32_t baudrate);

/**
 * @brief Get configuration for UART2 on PA2/PA3
 * @param config Output configuration structure
 * @param baudrate Desired baudrate
 */
void stm32f103_uart_get_uart2_pa2_pa3_config(stm32f103_uart_config_t *config, uint32_t baudrate);

/**
 * @brief Get configuration for UART3 on PB10/PB11
 * @param config Output configuration structure
 * @param baudrate Desired baudrate
 */
void stm32f103_uart_get_uart3_pb10_pb11_config(stm32f103_uart_config_t *config, uint32_t baudrate);

/*==============================================================================
 * UART Transmission
 *============================================================================*/

/**
 * @brief Send a single byte via UART
 * @param handle UART handle
 * @param byte Byte to send
 * @return 0 on success, negative on error
 */
int stm32f103_uart_send_byte(stm32f103_uart_handle_t *handle, uint8_t byte);

/**
 * @brief Send data buffer via UART
 * @param handle UART handle
 * @param data Data buffer to send
 * @param length Number of bytes to send
 * @return Number of bytes sent, negative on error
 */
int stm32f103_uart_send_data(stm32f103_uart_handle_t *handle, 
                             const uint8_t *data, uint32_t length);

/**
 * @brief Send null-terminated string via UART
 * @param handle UART handle
 * @param str String to send
 * @return Number of bytes sent, negative on error
 */
int stm32f103_uart_send_string(stm32f103_uart_handle_t *handle, const char *str);

/**
 * @brief Send formatted string via UART (simple version)
 * @param handle UART handle
 * @param str String prefix
 * @param num Number to append (decimal)
 * @return Number of bytes sent, negative on error
 */
int stm32f103_uart_send_string_num(stm32f103_uart_handle_t *handle, 
                                   const char *str, uint32_t num);

/*==============================================================================
 * UART Reception
 *============================================================================*/

/**
 * @brief Receive a single byte from UART (blocking)
 * @param handle UART handle
 * @param byte Output byte
 * @param timeout_ms Timeout in milliseconds (0 = no timeout)
 * @return 0 on success, negative on error/timeout
 */
int stm32f103_uart_receive_byte(stm32f103_uart_handle_t *handle, 
                                uint8_t *byte, uint32_t timeout_ms);

/**
 * @brief Check if data is available for reception
 * @param handle UART handle
 * @return true if data available, false otherwise
 */
bool stm32f103_uart_data_available(stm32f103_uart_handle_t *handle);

/*==============================================================================
 * Utility Functions
 *============================================================================*/

/**
 * @brief Calculate UART BRR value for given clock and baudrate
 * @param peripheral_clock_hz Peripheral clock frequency in Hz
 * @param baudrate Desired baudrate
 * @return BRR register value
 */
uint32_t stm32f103_uart_calculate_brr(uint32_t peripheral_clock_hz, uint32_t baudrate);

/**
 * @brief Validate UART configuration
 * @param config Configuration to validate
 * @return 0 if valid, negative if invalid
 */
int stm32f103_uart_validate_config(const stm32f103_uart_config_t *config);

/**
 * @brief Get UART peripheral base address
 * @param peripheral UART peripheral
 * @return Base address or 0 if invalid
 */
uint32_t stm32f103_uart_get_base_address(stm32f103_uart_peripheral_t peripheral);

/*==============================================================================
 * Debug UART Convenience Functions
 *============================================================================*/

/**
 * @brief Initialize default debug UART (UART1 on PA9/PA10, 115200 baud)
 * @return 0 on success, negative on error
 */
int stm32f103_uart_debug_init(void);

/**
 * @brief Send debug string
 * @param str String to send
 */
void stm32f103_uart_debug_print(const char *str);

/**
 * @brief Send debug string with number
 * @param str String prefix
 * @param num Number to append
 */
void stm32f103_uart_debug_print_num(const char *str, uint32_t num);

/**
 * @brief Deinitialize debug UART
 */
void stm32f103_uart_debug_deinit(void);

#endif /* STM32F103_UART_H */