#include "stm32f103_uart.h"
#include "stm32f103_uart_defs.h"
#include "stm32f103_clock.h"
#include "stm32f103_gpio.h"
#include "stm32f103_timing.h"
#include "stm32f103_registers.h"

/**
 * @file stm32f103_uart.c
 * @brief STM32F103 UART Interface Implementation
 */

/*==============================================================================
 * UART Register Offset Definitions (use with base addresses from registers.h)
 *============================================================================*/

#define USART_SR_OFFSET    0x00
#define USART_DR_OFFSET    0x04
#define USART_BRR_OFFSET   0x08
#define USART_CR1_OFFSET   0x0C
#define USART_CR2_OFFSET   0x10
#define USART_CR3_OFFSET   0x14

// Note: All base addresses and register bit definitions are taken from stm32f103_registers.h

/*==============================================================================
 * Private Functions
 *============================================================================*/

/**
 * @brief Get UART register base address from peripheral ID
 */
uint32_t stm32f103_uart_get_base_address(stm32f103_uart_id_t uart_id) {
    switch (uart_id) {
        case STM32F103_UART_1: return USART1_BASE;
        case STM32F103_UART_2: return USART2_BASE;
        case STM32F103_UART_3: return USART3_BASE;
        default: return 0;
    }
}

/**
 * @brief Enable UART peripheral clock
 */
static int enable_uart_clock(stm32f103_uart_id_t uart_id) {
    switch (uart_id) {
        case STM32F103_UART_1:
            return stm32f103_enable_peripheral_clock(RCC_APB2ENR_OFFSET, RCC_APB2ENR_USART1EN);
        case STM32F103_UART_2:
            return stm32f103_enable_peripheral_clock(RCC_APB1ENR_OFFSET, RCC_APB1ENR_USART2EN);
        case STM32F103_UART_3:
            return stm32f103_enable_peripheral_clock(RCC_APB1ENR_OFFSET, RCC_APB1ENR_USART3EN);
        default:
            return -1;
    }
}

/**
 * @brief Configure UART pins based on peripheral ID and remap setting
 */
static int configure_uart_pins(stm32f103_uart_id_t uart_id, bool use_remap) {
    stm32f103_gpio_config_t gpio_config;
    
    switch (uart_id) {
        case STM32F103_UART_1:
            if (use_remap) {
                // UART1 remap: PB6(TX), PB7(RX)
                stm32f103_gpio_enable_port_clock(STM32F103_GPIO_PORT_B);
                
                // TX pin: PB6 as alternate function push-pull
                gpio_config.port = STM32F103_GPIO_PORT_B;
                gpio_config.pin = STM32F103_GPIO_PIN_6;
                gpio_config.mode = STM32F103_GPIO_MODE_OUTPUT_50MHz;
                gpio_config.cnf = STM32F103_GPIO_CNF_ALTFN_PUSH_PULL;
                stm32f103_gpio_configure_pin(&gpio_config);
                
                // RX pin: PB7 as input floating
                gpio_config.pin = STM32F103_GPIO_PIN_7;
                gpio_config.mode = STM32F103_GPIO_MODE_INPUT;
                gpio_config.cnf = STM32F103_GPIO_CNF_INPUT_FLOATING;
                stm32f103_gpio_configure_pin(&gpio_config);
            } else {
                // UART1 default: PA9(TX), PA10(RX)
                stm32f103_gpio_enable_port_clock(STM32F103_GPIO_PORT_A);
                
                // TX pin: PA9 as alternate function push-pull
                gpio_config.port = STM32F103_GPIO_PORT_A;
                gpio_config.pin = STM32F103_GPIO_PIN_9;
                gpio_config.mode = STM32F103_GPIO_MODE_OUTPUT_50MHz;
                gpio_config.cnf = STM32F103_GPIO_CNF_ALTFN_PUSH_PULL;
                stm32f103_gpio_configure_pin(&gpio_config);
                
                // RX pin: PA10 as input floating
                gpio_config.pin = STM32F103_GPIO_PIN_10;
                gpio_config.mode = STM32F103_GPIO_MODE_INPUT;
                gpio_config.cnf = STM32F103_GPIO_CNF_INPUT_FLOATING;
                stm32f103_gpio_configure_pin(&gpio_config);
            }
            break;
            
        case STM32F103_UART_2:
            // UART2: PA2(TX), PA3(RX) - no remap option
            stm32f103_gpio_enable_port_clock(STM32F103_GPIO_PORT_A);
            
            // TX pin: PA2 as alternate function push-pull
            gpio_config.port = STM32F103_GPIO_PORT_A;
            gpio_config.pin = STM32F103_GPIO_PIN_2;
            gpio_config.mode = STM32F103_GPIO_MODE_OUTPUT_50MHz;
            gpio_config.cnf = STM32F103_GPIO_CNF_ALTFN_PUSH_PULL;
            stm32f103_gpio_configure_pin(&gpio_config);
            
            // RX pin: PA3 as input floating
            gpio_config.pin = STM32F103_GPIO_PIN_3;
            gpio_config.mode = STM32F103_GPIO_MODE_INPUT;
            gpio_config.cnf = STM32F103_GPIO_CNF_INPUT_FLOATING;
            stm32f103_gpio_configure_pin(&gpio_config);
            break;
            
        case STM32F103_UART_3:
            if (use_remap) {
                // UART3 remap: PC10(TX), PC11(RX)
                stm32f103_gpio_enable_port_clock(STM32F103_GPIO_PORT_C);
                
                // TX pin: PC10 as alternate function push-pull
                gpio_config.port = STM32F103_GPIO_PORT_C;
                gpio_config.pin = STM32F103_GPIO_PIN_10;
                gpio_config.mode = STM32F103_GPIO_MODE_OUTPUT_50MHz;
                gpio_config.cnf = STM32F103_GPIO_CNF_ALTFN_PUSH_PULL;
                stm32f103_gpio_configure_pin(&gpio_config);
                
                // RX pin: PC11 as input floating
                gpio_config.pin = STM32F103_GPIO_PIN_11;
                gpio_config.mode = STM32F103_GPIO_MODE_INPUT;
                gpio_config.cnf = STM32F103_GPIO_CNF_INPUT_FLOATING;
                stm32f103_gpio_configure_pin(&gpio_config);
            } else {
                // UART3 default: PB10(TX), PB11(RX)
                stm32f103_gpio_enable_port_clock(STM32F103_GPIO_PORT_B);
                
                // TX pin: PB10 as alternate function push-pull
                gpio_config.port = STM32F103_GPIO_PORT_B;
                gpio_config.pin = STM32F103_GPIO_PIN_10;
                gpio_config.mode = STM32F103_GPIO_MODE_OUTPUT_50MHz;
                gpio_config.cnf = STM32F103_GPIO_CNF_ALTFN_PUSH_PULL;
                stm32f103_gpio_configure_pin(&gpio_config);
                
                // RX pin: PB11 as input floating
                gpio_config.pin = STM32F103_GPIO_PIN_11;
                gpio_config.mode = STM32F103_GPIO_MODE_INPUT;
                gpio_config.cnf = STM32F103_GPIO_CNF_INPUT_FLOATING;
                stm32f103_gpio_configure_pin(&gpio_config);
            }
            break;
            
        default:
            return -1;
    }
    
    return 0;
}

/*==============================================================================
 * NHAL-Compatible Simple UART Implementation
 *============================================================================*/

/**
 * @brief Initialize UART with NHAL configuration
 */
int stm32f103_uart_init_simple(stm32f103_uart_id_t uart_id, const struct nhal_uart_config *nhal_config) {
    if (!nhal_config) {
        return -1;
    }
    
    // Get implementation config if provided
    const struct nhal_uart_impl_config *impl = (const struct nhal_uart_impl_config *)nhal_config->impl_config;
    bool use_remap = impl ? impl->use_remap : false;
    
    // Configure pins
    int pin_result = configure_uart_pins(uart_id, use_remap);
    if (pin_result != 0) {
        return pin_result;
    }
    
    // Enable UART clock
    int clock_result = enable_uart_clock(uart_id);
    if (clock_result != 0) {
        return clock_result;
    }
    
    // Get UART base address
    uint32_t base = stm32f103_uart_get_base_address(uart_id);
    if (base == 0) {
        return -1;
    }
    
    // Configure UART parameters
    uint32_t cr1 = 0;
    uint32_t cr2 = 0;
    
    // Configure data bits (word length)
    switch (nhal_config->data_bits) {
        case NHAL_UART_DATA_BITS_7:
            // STM32F103 doesn't support 7-bit directly, would need parity for 8-bit frame
            // For now, treat as 8-bit
        case NHAL_UART_DATA_BITS_8:
        default:
            // 8-bit is default (M=0), STM32F103 supports 8 or 9 bit
            break;
    }
    
    // Configure parity
    switch (nhal_config->parity) {
        case NHAL_UART_PARITY_EVEN:
            cr1 |= USART_CR1_PCE;  // Enable parity, even is default
            break;
        case NHAL_UART_PARITY_ODD:
            cr1 |= USART_CR1_PCE | USART_CR1_PS;  // Enable parity + odd
            break;
        case NHAL_UART_PARITY_NONE:
        default:
            // No parity (default)
            break;
    }
    
    // Configure stop bits
    switch (nhal_config->stop_bits) {
        case NHAL_UART_STOP_BITS_2:
            cr2 |= (2 << 12);  // STOP[1:0] = 10 for 2 stop bits
            break;
        case NHAL_UART_STOP_BITS_1:
        default:
            // 1 stop bit is default (STOP[1:0] = 00)
            break;
    }
    
    // Calculate and set baud rate
    uint32_t peripheral_clock;
    if (uart_id == STM32F103_UART_1) {
        peripheral_clock = stm32f103_get_pclk2_hz();
    } else {
        peripheral_clock = stm32f103_get_pclk1_hz();
    }
    
    uint32_t brr = peripheral_clock / nhal_config->baudrate;
    *(volatile uint32_t *)(base + USART_BRR_OFFSET) = brr;
    
    // Write CR2 register
    *(volatile uint32_t *)(base + USART_CR2_OFFSET) = cr2;
    
    // Enable UART, TX, RX
    cr1 |= USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;
    *(volatile uint32_t *)(base + USART_CR1_OFFSET) = cr1;
    
    return 0;
}

/*==============================================================================
 * NHAL Helper Functions for Simple Operations
 *============================================================================*/

/**
 * @brief Send a single byte with timeout
 */
int stm32f103_uart_send_byte_timeout(stm32f103_uart_id_t uart_id, uint8_t data, uint32_t timeout_ms) {
    uint32_t base = stm32f103_uart_get_base_address(uart_id);
    if (base == 0) {
        return -1;
    }
    
    uint32_t start_tick = stm32f103_get_tick();
    
    // Wait for TX register to be empty
    while (!(*(volatile uint32_t *)(base + USART_SR_OFFSET) & USART_SR_TXE)) {
        if (stm32f103_timeout_occurred(start_tick, timeout_ms)) {
            return -2;  // Timeout
        }
    }
    
    // Send data
    *(volatile uint32_t *)(base + USART_DR_OFFSET) = data;
    
    return 0;
}

/**
 * @brief Receive a single byte with timeout
 */
int stm32f103_uart_receive_byte_timeout(stm32f103_uart_id_t uart_id, uint8_t *data, uint32_t timeout_ms) {
    if (!data) {
        return -1;
    }
    
    uint32_t base = stm32f103_uart_get_base_address(uart_id);
    if (base == 0) {
        return -1;
    }
    
    uint32_t start_tick = stm32f103_get_tick();
    
    // Wait for data to be available
    while (!(*(volatile uint32_t *)(base + USART_SR_OFFSET) & USART_SR_RXNE)) {
        if (stm32f103_timeout_occurred(start_tick, timeout_ms)) {
            return -2;  // Timeout
        }
    }
    
    // Read data
    *data = (uint8_t)(*(volatile uint32_t *)(base + USART_DR_OFFSET) & 0xFF);
    
    return 0;
}

/**
 * @brief Check if UART TX is ready
 */
bool stm32f103_uart_is_tx_ready(stm32f103_uart_id_t uart_id) {
    uint32_t base = stm32f103_uart_get_base_address(uart_id);
    if (base == 0) {
        return false;
    }
    
    return (*(volatile uint32_t *)(base + USART_SR_OFFSET) & USART_SR_TXE) != 0;
}

/**
 * @brief Check if UART has received data
 */
bool stm32f103_uart_is_rx_data_available(stm32f103_uart_id_t uart_id) {
    uint32_t base = stm32f103_uart_get_base_address(uart_id);
    if (base == 0) {
        return false;
    }
    
    return (*(volatile uint32_t *)(base + USART_SR_OFFSET) & USART_SR_RXNE) != 0;
}