#include "nhal_uart.h"
#include "stm32f103_uart.h"
#include "stm32f103_uart_defs.h"
#include "stm32f103_gpio.h"
#include "stm32f103_registers.h"

/**
 * @file nhal_uart_stm32f103.c
 * @brief STM32F103 implementation of NHAL UART interface
 */

// Base address function is now provided by stm32f103_uart.c

static nhal_result_t configure_uart_pins(stm32f103_uart_id_t uart_id, bool use_remap) {
    stm32f103_gpio_config_t pin_config;
    
    // Select pins based on UART ID and remap setting
    switch (uart_id) {
        case STM32F103_UART_1:
            if (use_remap) {
                // UART1 remap: PB6(TX), PB7(RX)
                pin_config.port = STM32F103_GPIO_PORT_B;
                pin_config.pin = STM32F103_GPIO_PIN_6;
                pin_config.mode = STM32F103_GPIO_MODE_OUTPUT_50MHz;
                pin_config.cnf = STM32F103_GPIO_CNF_ALTFN_PUSH_PULL;
                if (stm32f103_gpio_configure_pin(&pin_config) != 0) {
                    return NHAL_ERR_HW_FAILURE;
                }
                
                pin_config.pin = STM32F103_GPIO_PIN_7;
                pin_config.mode = STM32F103_GPIO_MODE_INPUT;
                pin_config.cnf = STM32F103_GPIO_CNF_INPUT_FLOATING;
                if (stm32f103_gpio_configure_pin(&pin_config) != 0) {
                    return NHAL_ERR_HW_FAILURE;
                }
            } else {
                // UART1 default: PA9(TX), PA10(RX)
                pin_config.port = STM32F103_GPIO_PORT_A;
                pin_config.pin = STM32F103_GPIO_PIN_9;
                pin_config.mode = STM32F103_GPIO_MODE_OUTPUT_50MHz;
                pin_config.cnf = STM32F103_GPIO_CNF_ALTFN_PUSH_PULL;
                if (stm32f103_gpio_configure_pin(&pin_config) != 0) {
                    return NHAL_ERR_HW_FAILURE;
                }
                
                pin_config.pin = STM32F103_GPIO_PIN_10;
                pin_config.mode = STM32F103_GPIO_MODE_INPUT;
                pin_config.cnf = STM32F103_GPIO_CNF_INPUT_FLOATING;
                if (stm32f103_gpio_configure_pin(&pin_config) != 0) {
                    return NHAL_ERR_HW_FAILURE;
                }
            }
            break;
            
        case STM32F103_UART_2:
            // UART2: PA2(TX), PA3(RX)
            pin_config.port = STM32F103_GPIO_PORT_A;
            pin_config.pin = STM32F103_GPIO_PIN_2;
            pin_config.mode = STM32F103_GPIO_MODE_OUTPUT_50MHz;
            pin_config.cnf = STM32F103_GPIO_CNF_ALTFN_PUSH_PULL;
            if (stm32f103_gpio_configure_pin(&pin_config) != 0) {
                return NHAL_ERR_HW_FAILURE;
            }
            
            pin_config.pin = STM32F103_GPIO_PIN_3;
            pin_config.mode = STM32F103_GPIO_MODE_INPUT;
            pin_config.cnf = STM32F103_GPIO_CNF_INPUT_FLOATING;
            if (stm32f103_gpio_configure_pin(&pin_config) != 0) {
                return NHAL_ERR_HW_FAILURE;
            }
            break;
            
        case STM32F103_UART_3:
            // UART3: PB10(TX), PB11(RX)
            pin_config.port = STM32F103_GPIO_PORT_B;
            pin_config.pin = STM32F103_GPIO_PIN_10;
            pin_config.mode = STM32F103_GPIO_MODE_OUTPUT_50MHz;
            pin_config.cnf = STM32F103_GPIO_CNF_ALTFN_PUSH_PULL;
            if (stm32f103_gpio_configure_pin(&pin_config) != 0) {
                return NHAL_ERR_HW_FAILURE;
            }
            
            pin_config.pin = STM32F103_GPIO_PIN_11;
            pin_config.mode = STM32F103_GPIO_MODE_INPUT;
            pin_config.cnf = STM32F103_GPIO_CNF_INPUT_FLOATING;
            if (stm32f103_gpio_configure_pin(&pin_config) != 0) {
                return NHAL_ERR_HW_FAILURE;
            }
            break;
            
        default:
            return NHAL_ERR_INVALID_ARG;
    }
    
    return NHAL_OK;
}

/*==============================================================================
 * Debug UART Printf-like Functions (operates over NHAL interface)
 *============================================================================*/

#include <stdarg.h>

// Static debug UART context pointer - set by user
static struct nhal_uart_context *debug_uart_ctx = NULL;

/**
 * @brief Initialize debug UART with provided context
 * @param uart_ctx UART context to use for debug output
 * @return 0 on success, -1 on error
 */
int nhal_uart_debug_init(struct nhal_uart_context *uart_ctx) {
    if (!uart_ctx) {
        return -1;
    }
    
    debug_uart_ctx = uart_ctx;
    return 0;
}

/**
 * @brief Simple implementation of printf-like functionality
 * @param format Printf-style format string
 * @param ... Variable arguments
 * @return Number of characters written, or -1 on error
 */
int nhal_uart_debug_printf(const char* format, ...) {
    if (!debug_uart_ctx || !format) {
        return -1;
    }
    
    va_list args;
    va_start(args, format);
    
    const char* ptr = format;
    int count = 0;
    
    while (*ptr) {
        if (*ptr == '%' && *(ptr + 1)) {
            ptr++; // Skip '%'
            
            switch (*ptr) {
                case 'd': {
                    int num = va_arg(args, int);
                    char buffer[12];
                    int len = 0;
                    
                    if (num == 0) {
                        buffer[len++] = '0';
                    } else {
                        int temp = num;
                        bool negative = false;
                        if (temp < 0) {
                            negative = true;
                            temp = -temp;
                        }
                        
                        // Convert to string (reversed)
                        while (temp > 0) {
                            buffer[len++] = '0' + (temp % 10);
                            temp /= 10;
                        }
                        
                        if (negative) {
                            buffer[len++] = '-';
                        }
                        
                        // Reverse string
                        for (int i = 0; i < len / 2; i++) {
                            char temp_char = buffer[i];
                            buffer[i] = buffer[len - 1 - i];
                            buffer[len - 1 - i] = temp_char;
                        }
                    }
                    
                    // Send the number string
                    nhal_uart_write(debug_uart_ctx, (uint8_t*)buffer, len);
                    count += len;
                    break;
                }
                
                case 'u': {
                    unsigned int num = va_arg(args, unsigned int);
                    char buffer[12];
                    int len = 0;
                    
                    if (num == 0) {
                        buffer[len++] = '0';
                    } else {
                        // Convert to string (reversed)
                        while (num > 0) {
                            buffer[len++] = '0' + (num % 10);
                            num /= 10;
                        }
                        
                        // Reverse string
                        for (int i = 0; i < len / 2; i++) {
                            char temp_char = buffer[i];
                            buffer[i] = buffer[len - 1 - i];
                            buffer[len - 1 - i] = temp_char;
                        }
                    }
                    
                    // Send the number string
                    nhal_uart_write(debug_uart_ctx, (uint8_t*)buffer, len);
                    count += len;
                    break;
                }
                
                case 'x': {
                    unsigned int num = va_arg(args, unsigned int);
                    char buffer[10];
                    int len = 0;
                    const char hex_chars[] = "0123456789abcdef";
                    
                    if (num == 0) {
                        buffer[len++] = '0';
                    } else {
                        // Convert to hex string (reversed)
                        while (num > 0) {
                            buffer[len++] = hex_chars[num & 0xF];
                            num >>= 4;
                        }
                        
                        // Reverse string
                        for (int i = 0; i < len / 2; i++) {
                            char temp_char = buffer[i];
                            buffer[i] = buffer[len - 1 - i];
                            buffer[len - 1 - i] = temp_char;
                        }
                    }
                    
                    // Send the hex string
                    nhal_uart_write(debug_uart_ctx, (uint8_t*)buffer, len);
                    count += len;
                    break;
                }
                
                case 's': {
                    const char* str = va_arg(args, const char*);
                    if (str) {
                        int len = 0;
                        while (str[len]) len++; // Calculate length
                        nhal_uart_write(debug_uart_ctx, (const uint8_t*)str, len);
                        count += len;
                    }
                    break;
                }
                
                case 'c': {
                    char c = (char)va_arg(args, int);
                    nhal_uart_write(debug_uart_ctx, (uint8_t*)&c, 1);
                    count++;
                    break;
                }
                
                case '%': {
                    uint8_t percent = '%';
                    nhal_uart_write(debug_uart_ctx, &percent, 1);
                    count++;
                    break;
                }
                
                default: {
                    // Unknown format specifier, just print it
                    uint8_t percent = '%';
                    nhal_uart_write(debug_uart_ctx, &percent, 1);
                    nhal_uart_write(debug_uart_ctx, (uint8_t*)ptr, 1);
                    count += 2;
                    break;
                }
            }
        } else {
            // Regular character
            nhal_uart_write(debug_uart_ctx, (uint8_t*)ptr, 1);
            count++;
        }
        ptr++;
    }
    
    va_end(args);
    return count;
}



nhal_result_t nhal_uart_init(struct nhal_uart_context *ctx) {
    if (!ctx) {
        return NHAL_ERR_INVALID_ARG;
    }

    // Validate UART ID (base address derivation will be done as needed)
    if (stm32f103_uart_get_base_address(ctx->uart_id) == 0) {
        return NHAL_ERR_INVALID_ARG;
    }

    // Configure pins automatically based on UART ID (no remap for now)
    nhal_result_t result = configure_uart_pins(ctx->uart_id, false);
    if (result != NHAL_OK) {
        return result;
    }

    ctx->is_initialized = true;
    ctx->is_configured = false; // Will be set true when configured
    return NHAL_OK;
}

nhal_result_t nhal_uart_deinit(struct nhal_uart_context *ctx) {
    if (!ctx) {
        return NHAL_ERR_INVALID_ARG;
    }
    
    ctx->is_initialized = false;
    ctx->is_configured = false;
    return NHAL_OK;
}

nhal_result_t nhal_uart_set_config(struct nhal_uart_context *ctx, struct nhal_uart_config *cfg) {
    if (!ctx || !cfg || !ctx->is_initialized) {
        return NHAL_ERR_INVALID_ARG;
    }

    if (stm32f103_uart_init_simple(ctx->uart_id, cfg) != 0) {
        return NHAL_ERR_HW_FAILURE;
    }

    ctx->is_configured = true;
    return NHAL_OK;
}

nhal_result_t nhal_uart_get_config(struct nhal_uart_context *ctx, struct nhal_uart_config *cfg) {
    if (!ctx || !cfg || !ctx->is_initialized) {
        return NHAL_ERR_INVALID_ARG;
    }
    
    // Implementation would read current hardware config
    // For now, return not implemented
    return NHAL_ERR_UNSUPPORTED;
}

nhal_result_t nhal_uart_write(struct nhal_uart_context *ctx, const uint8_t *data, size_t len) {
    if (!ctx || !data || !ctx->is_initialized) {
        return NHAL_ERR_INVALID_ARG;
    }

    uint32_t timeout_ms = 1000; // Default timeout

    for (size_t i = 0; i < len; i++) {
        if (stm32f103_uart_send_byte_timeout(ctx->uart_id, data[i], timeout_ms) != 0) {
            return NHAL_ERR_TIMEOUT;
        }
    }

    return NHAL_OK;
}

nhal_result_t nhal_uart_read(struct nhal_uart_context *ctx, uint8_t *data, size_t len) {
    if (!ctx || !data || !ctx->is_initialized) {
        return NHAL_ERR_INVALID_ARG;
    }

    uint32_t timeout_ms = 1000; // Default timeout

    for (size_t i = 0; i < len; i++) {
        if (stm32f103_uart_receive_byte_timeout(ctx->uart_id, &data[i], timeout_ms) != 0) {
            return NHAL_ERR_TIMEOUT;
        }
    }

    return NHAL_OK;
}