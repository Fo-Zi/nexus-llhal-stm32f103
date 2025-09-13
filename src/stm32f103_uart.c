#include "stm32f103_uart.h"
#include "stm32f103_clock.h"
#include "stm32f103_gpio.h"
#include "stm32f103_timing.h"
#include "stm32f103_registers.h"

/**
 * @file stm32f103_uart.c
 * @brief STM32F103 UART Interface Implementation
 */

/*==============================================================================
 * Private Variables
 *============================================================================*/

static stm32f103_uart_handle_t debug_uart_handle;
static bool debug_uart_initialized = false;

/*==============================================================================
 * Private Functions
 *============================================================================*/

/**
 * @brief Get UART register base address from peripheral enum
 */
static uint32_t get_uart_base(stm32f103_uart_peripheral_t peripheral) {
    switch (peripheral) {
        case STM32F103_UART_1: return USART1_BASE;
        case STM32F103_UART_2: return USART2_BASE;
        case STM32F103_UART_3: return USART3_BASE;
        default: return 0;
    }
}

/**
 * @brief Enable UART peripheral clock
 */
static int enable_uart_clock(stm32f103_uart_peripheral_t peripheral) {
    switch (peripheral) {
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
 * @brief Configure UART pins
 */
static int configure_uart_pins(const stm32f103_uart_config_t *config) {
    stm32f103_gpio_config_t gpio_config;
    
    // Enable GPIO port clocks
    stm32f103_gpio_enable_port_clock(config->pins.tx_port);
    stm32f103_gpio_enable_port_clock(config->pins.rx_port);
    
    if (config->enable_tx) {
        // Configure TX pin as alternate function push-pull
        gpio_config.port = config->pins.tx_port;
        gpio_config.pin = config->pins.tx_pin;
        gpio_config.mode = STM32F103_GPIO_MODE_OUTPUT_50MHz;
        gpio_config.cnf = STM32F103_GPIO_CNF_ALTFN_PUSH_PULL;
        stm32f103_gpio_configure_pin(&gpio_config);
    }
    
    if (config->enable_rx) {
        // Configure RX pin as input floating
        gpio_config.port = config->pins.rx_port;
        gpio_config.pin = config->pins.rx_pin;
        gpio_config.mode = STM32F103_GPIO_MODE_INPUT;
        gpio_config.cnf = STM32F103_GPIO_CNF_INPUT_FLOATING;
        stm32f103_gpio_configure_pin(&gpio_config);
    }
    
    // Configure flow control pins if enabled
    if (config->pins.use_flow_control) {
        if (config->flow_control == STM32F103_UART_FLOWCTRL_RTS || 
            config->flow_control == STM32F103_UART_FLOWCTRL_RTS_CTS) {
            gpio_config.port = config->pins.rts_port;
            gpio_config.pin = config->pins.rts_pin;
            gpio_config.mode = STM32F103_GPIO_MODE_OUTPUT_50MHz;
            gpio_config.cnf = STM32F103_GPIO_CNF_ALTFN_PUSH_PULL;
            stm32f103_gpio_configure_pin(&gpio_config);
        }
        
        if (config->flow_control == STM32F103_UART_FLOWCTRL_CTS || 
            config->flow_control == STM32F103_UART_FLOWCTRL_RTS_CTS) {
            gpio_config.port = config->pins.cts_port;
            gpio_config.pin = config->pins.cts_pin;
            gpio_config.mode = STM32F103_GPIO_MODE_INPUT;
            gpio_config.cnf = STM32F103_GPIO_CNF_INPUT_FLOATING;
            stm32f103_gpio_configure_pin(&gpio_config);
        }
    }
    
    return 0;
}

/*==============================================================================
 * Public Functions - UART Initialization
 *============================================================================*/

int stm32f103_uart_init(stm32f103_uart_handle_t *handle, 
                        const stm32f103_uart_config_t *config) {
    if (!handle || !config) {
        return -1;
    }
    
    // Validate configuration
    if (stm32f103_uart_validate_config(config) != 0) {
        return -1;
    }
    
    // Get UART base address
    uint32_t base_addr = get_uart_base(config->peripheral);
    if (base_addr == 0) {
        return -1;
    }
    
    // Initialize handle
    handle->peripheral = config->peripheral;
    handle->base_address = base_addr;
    handle->config = *config;
    handle->initialized = false;
    
    // Enable UART clock
    if (enable_uart_clock(config->peripheral) != 0) {
        return -2;
    }
    
    // Configure pins
    if (configure_uart_pins(config) != 0) {
        return -3;
    }
    
    // Get peripheral clock frequency
    stm32f103_clock_frequencies_t freq;
    if (stm32f103_get_clock_frequencies(&freq) != 0) {
        return -4;
    }
    
    uint32_t pclk_hz = (config->peripheral == STM32F103_UART_1) ? freq.pclk2_hz : freq.pclk1_hz;
    
    // Disable UART first
    volatile uint32_t *cr1_reg = (volatile uint32_t *)(base_addr + 0x0C); // CR1 offset
    *cr1_reg = 0;
    
    // Configure baud rate
    volatile uint32_t *brr_reg = (volatile uint32_t *)(base_addr + 0x08); // BRR offset
    *brr_reg = stm32f103_uart_calculate_brr(pclk_hz, config->baudrate);
    
    // Configure CR1 (control register 1)
    uint32_t cr1 = 0;
    
    if (config->enable_tx) {
        cr1 |= USART_CR1_TE;
    }
    
    if (config->enable_rx) {
        cr1 |= USART_CR1_RE;
    }
    
    // Word length
    if (config->word_length == STM32F103_UART_WORDLEN_9B) {
        cr1 |= USART_CR1_M;
    }
    
    // Parity
    if (config->parity != STM32F103_UART_PARITY_NONE) {
        cr1 |= USART_CR1_PCE;
        if (config->parity == STM32F103_UART_PARITY_ODD) {
            cr1 |= USART_CR1_PS;
        }
    }
    
    // Interrupts
    if (config->enable_interrupts) {
        cr1 |= USART_CR1_RXNEIE; // RX interrupt
    }
    
    // Enable UART
    cr1 |= USART_CR1_UE;
    *cr1_reg = cr1;
    
    // Configure CR2 (stop bits)
    volatile uint32_t *cr2_reg = (volatile uint32_t *)(base_addr + 0x10); // CR2 offset
    uint32_t cr2 = 0;
    cr2 |= (config->stop_bits << 12); // STOP bits [13:12]
    *cr2_reg = cr2;
    
    // Configure CR3 (flow control)
    volatile uint32_t *cr3_reg = (volatile uint32_t *)(base_addr + 0x14); // CR3 offset
    uint32_t cr3 = 0;
    if (config->flow_control == STM32F103_UART_FLOWCTRL_RTS || 
        config->flow_control == STM32F103_UART_FLOWCTRL_RTS_CTS) {
        cr3 |= USART_CR3_RTSE;
    }
    if (config->flow_control == STM32F103_UART_FLOWCTRL_CTS || 
        config->flow_control == STM32F103_UART_FLOWCTRL_RTS_CTS) {
        cr3 |= USART_CR3_CTSE;
    }
    *cr3_reg = cr3;
    
    handle->initialized = true;
    return 0;
}

int stm32f103_uart_deinit(stm32f103_uart_handle_t *handle) {
    if (!handle || !handle->initialized) {
        return -1;
    }
    
    // Disable UART
    volatile uint32_t *cr1_reg = (volatile uint32_t *)(handle->base_address + 0x0C);
    *cr1_reg = 0;
    
    // Reset pins to input floating (default state)
    stm32f103_gpio_config_t gpio_config;
    
    if (handle->config.enable_tx) {
        gpio_config.port = handle->config.pins.tx_port;
        gpio_config.pin = handle->config.pins.tx_pin;
        gpio_config.mode = STM32F103_GPIO_MODE_INPUT;
        gpio_config.cnf = STM32F103_GPIO_CNF_INPUT_FLOATING;
        stm32f103_gpio_configure_pin(&gpio_config);
    }
    
    if (handle->config.enable_rx) {
        gpio_config.port = handle->config.pins.rx_port;
        gpio_config.pin = handle->config.pins.rx_pin;
        gpio_config.mode = STM32F103_GPIO_MODE_INPUT;
        gpio_config.cnf = STM32F103_GPIO_CNF_INPUT_FLOATING;
        stm32f103_gpio_configure_pin(&gpio_config);
    }
    
    handle->initialized = false;
    return 0;
}

int stm32f103_uart_reconfigure(stm32f103_uart_handle_t *handle,
                               const stm32f103_uart_config_t *config) {
    if (!handle || !config) {
        return -1;
    }
    
    // Deinitialize current configuration
    stm32f103_uart_deinit(handle);
    
    // Initialize with new configuration
    return stm32f103_uart_init(handle, config);
}

/*==============================================================================
 * Public Functions - Predefined Configurations
 *============================================================================*/

void stm32f103_uart_get_default_debug_config(stm32f103_uart_config_t *config) {
    stm32f103_uart_get_uart1_pa9_pa10_config(config, 115200);
}

void stm32f103_uart_get_uart1_pa9_pa10_config(stm32f103_uart_config_t *config, uint32_t baudrate) {
    if (!config) return;
    
    config->peripheral = STM32F103_UART_1;
    config->pins.tx_port = STM32F103_GPIO_PORT_A;
    config->pins.tx_pin = STM32F103_GPIO_PIN_9;
    config->pins.rx_port = STM32F103_GPIO_PORT_A;
    config->pins.rx_pin = STM32F103_GPIO_PIN_10;
    config->pins.use_flow_control = false;
    config->baudrate = baudrate;
    config->word_length = STM32F103_UART_WORDLEN_8B;
    config->stop_bits = STM32F103_UART_STOPBITS_1;
    config->parity = STM32F103_UART_PARITY_NONE;
    config->flow_control = STM32F103_UART_FLOWCTRL_NONE;
    config->enable_tx = true;
    config->enable_rx = true;
    config->enable_interrupts = false;
}

void stm32f103_uart_get_uart1_pb6_pb7_config(stm32f103_uart_config_t *config, uint32_t baudrate) {
    if (!config) return;
    
    config->peripheral = STM32F103_UART_1;
    config->pins.tx_port = STM32F103_GPIO_PORT_B;
    config->pins.tx_pin = STM32F103_GPIO_PIN_6;
    config->pins.rx_port = STM32F103_GPIO_PORT_B;
    config->pins.rx_pin = STM32F103_GPIO_PIN_7;
    config->pins.use_flow_control = false;
    config->baudrate = baudrate;
    config->word_length = STM32F103_UART_WORDLEN_8B;
    config->stop_bits = STM32F103_UART_STOPBITS_1;
    config->parity = STM32F103_UART_PARITY_NONE;
    config->flow_control = STM32F103_UART_FLOWCTRL_NONE;
    config->enable_tx = true;
    config->enable_rx = true;
    config->enable_interrupts = false;
}

void stm32f103_uart_get_uart2_pa2_pa3_config(stm32f103_uart_config_t *config, uint32_t baudrate) {
    if (!config) return;
    
    config->peripheral = STM32F103_UART_2;
    config->pins.tx_port = STM32F103_GPIO_PORT_A;
    config->pins.tx_pin = STM32F103_GPIO_PIN_2;
    config->pins.rx_port = STM32F103_GPIO_PORT_A;
    config->pins.rx_pin = STM32F103_GPIO_PIN_3;
    config->pins.use_flow_control = false;
    config->baudrate = baudrate;
    config->word_length = STM32F103_UART_WORDLEN_8B;
    config->stop_bits = STM32F103_UART_STOPBITS_1;
    config->parity = STM32F103_UART_PARITY_NONE;
    config->flow_control = STM32F103_UART_FLOWCTRL_NONE;
    config->enable_tx = true;
    config->enable_rx = true;
    config->enable_interrupts = false;
}

void stm32f103_uart_get_uart3_pb10_pb11_config(stm32f103_uart_config_t *config, uint32_t baudrate) {
    if (!config) return;
    
    config->peripheral = STM32F103_UART_3;
    config->pins.tx_port = STM32F103_GPIO_PORT_B;
    config->pins.tx_pin = STM32F103_GPIO_PIN_10;
    config->pins.rx_port = STM32F103_GPIO_PORT_B;
    config->pins.rx_pin = STM32F103_GPIO_PIN_11;
    config->pins.use_flow_control = false;
    config->baudrate = baudrate;
    config->word_length = STM32F103_UART_WORDLEN_8B;
    config->stop_bits = STM32F103_UART_STOPBITS_1;
    config->parity = STM32F103_UART_PARITY_NONE;
    config->flow_control = STM32F103_UART_FLOWCTRL_NONE;
    config->enable_tx = true;
    config->enable_rx = true;
    config->enable_interrupts = false;
}

/*==============================================================================
 * Public Functions - UART Transmission
 *============================================================================*/

int stm32f103_uart_send_byte(stm32f103_uart_handle_t *handle, uint8_t byte) {
    if (!handle || !handle->initialized) {
        return -1;
    }
    
    volatile uint32_t *sr_reg = (volatile uint32_t *)(handle->base_address + 0x00); // SR offset
    volatile uint32_t *dr_reg = (volatile uint32_t *)(handle->base_address + 0x04); // DR offset
    
    // Wait for TXE (Transmit Data Register Empty)
    while ((*sr_reg & USART_SR_TXE) == 0);
    
    // Send byte
    *dr_reg = byte;
    
    return 0;
}

int stm32f103_uart_send_data(stm32f103_uart_handle_t *handle, 
                             const uint8_t *data, uint32_t length) {
    if (!handle || !data || length == 0) {
        return -1;
    }
    
    for (uint32_t i = 0; i < length; i++) {
        if (stm32f103_uart_send_byte(handle, data[i]) != 0) {
            return i; // Return number of bytes sent before error
        }
    }
    
    return length;
}

int stm32f103_uart_send_string(stm32f103_uart_handle_t *handle, const char *str) {
    if (!handle || !str) {
        return -1;
    }
    
    int count = 0;
    while (*str) {
        if (stm32f103_uart_send_byte(handle, *str++) != 0) {
            return count; // Return number of bytes sent before error
        }
        count++;
    }
    
    return count;
}

int stm32f103_uart_send_string_num(stm32f103_uart_handle_t *handle, 
                                   const char *str, uint32_t num) {
    if (!handle) {
        return -1;
    }
    
    int bytes_sent = 0;
    
    // Send string part
    if (str) {
        int str_bytes = stm32f103_uart_send_string(handle, str);
        if (str_bytes < 0) return str_bytes;
        bytes_sent += str_bytes;
    }
    
    // Convert number to string and send
    char num_str[12]; // Max uint32_t is 10 digits + sign + null
    int pos = 10;
    num_str[11] = '\0';
    
    if (num == 0) {
        num_str[pos--] = '0';
    } else {
        while (num > 0) {
            num_str[pos--] = '0' + (num % 10);
            num /= 10;
        }
    }
    
    int num_bytes = stm32f103_uart_send_string(handle, &num_str[pos + 1]);
    if (num_bytes < 0) return num_bytes;
    bytes_sent += num_bytes;
    
    return bytes_sent;
}

/*==============================================================================
 * Public Functions - UART Reception
 *============================================================================*/

int stm32f103_uart_receive_byte(stm32f103_uart_handle_t *handle, 
                                uint8_t *byte, uint32_t timeout_ms) {
    if (!handle || !handle->initialized || !byte) {
        return -1;
    }
    
    volatile uint32_t *sr_reg = (volatile uint32_t *)(handle->base_address + 0x00); // SR offset
    volatile uint32_t *dr_reg = (volatile uint32_t *)(handle->base_address + 0x04); // DR offset
    
    uint32_t start_time = 0;
    if (timeout_ms > 0) {
        start_time = stm32f103_get_tick();
    }
    
    // Wait for RXNE (Receive Data Register Not Empty)
    while ((*sr_reg & USART_SR_RXNE) == 0) {
        if (timeout_ms > 0 && stm32f103_timeout_occurred(start_time, timeout_ms)) {
            return -2; // Timeout
        }
    }
    
    // Read byte
    *byte = (uint8_t)(*dr_reg & 0xFF);
    
    return 0;
}

bool stm32f103_uart_data_available(stm32f103_uart_handle_t *handle) {
    if (!handle || !handle->initialized) {
        return false;
    }
    
    volatile uint32_t *sr_reg = (volatile uint32_t *)(handle->base_address + 0x00); // SR offset
    return (*sr_reg & USART_SR_RXNE) != 0;
}

/*==============================================================================
 * Public Functions - Utility Functions
 *============================================================================*/

uint32_t stm32f103_uart_calculate_brr(uint32_t peripheral_clock_hz, uint32_t baudrate) {
    return (peripheral_clock_hz + (baudrate / 2)) / baudrate; // Round to nearest
}

int stm32f103_uart_validate_config(const stm32f103_uart_config_t *config) {
    if (!config) {
        return -1;
    }
    
    // Validate peripheral
    if (config->peripheral < STM32F103_UART_1 || config->peripheral > STM32F103_UART_3) {
        return -1;
    }
    
    // Validate pins
    if (!stm32f103_gpio_is_valid_port(config->pins.tx_port) ||
        !stm32f103_gpio_is_valid_pin(config->pins.tx_pin) ||
        !stm32f103_gpio_is_valid_port(config->pins.rx_port) ||
        !stm32f103_gpio_is_valid_pin(config->pins.rx_pin)) {
        return -1;
    }
    
    // Validate baudrate (reasonable range)
    if (config->baudrate < 300 || config->baudrate > 4500000) {
        return -1;
    }
    
    // At least one direction must be enabled
    if (!config->enable_tx && !config->enable_rx) {
        return -1;
    }
    
    return 0;
}

uint32_t stm32f103_uart_get_base_address(stm32f103_uart_peripheral_t peripheral) {
    return get_uart_base(peripheral);
}

/*==============================================================================
 * Public Functions - Debug UART Convenience Functions
 *============================================================================*/

int stm32f103_uart_debug_init(void) {
    if (debug_uart_initialized) {
        return 0; // Already initialized
    }
    
    stm32f103_uart_config_t config;
    stm32f103_uart_get_default_debug_config(&config);
    
    int result = stm32f103_uart_init(&debug_uart_handle, &config);
    if (result == 0) {
        debug_uart_initialized = true;
    }
    
    return result;
}

void stm32f103_uart_debug_print(const char *str) {
    if (debug_uart_initialized) {
        stm32f103_uart_send_string(&debug_uart_handle, str);
    }
}

void stm32f103_uart_debug_print_num(const char *str, uint32_t num) {
    if (debug_uart_initialized) {
        stm32f103_uart_send_string_num(&debug_uart_handle, str, num);
    }
}

void stm32f103_uart_debug_deinit(void) {
    if (debug_uart_initialized) {
        stm32f103_uart_deinit(&debug_uart_handle);
        debug_uart_initialized = false;
    }
}