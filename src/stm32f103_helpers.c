#include "stm32f103_helpers.h"
#include "stm32f103_registers.h"
#include "internal/stm32f103_internal.h"

uint32_t stm32f103_get_gpio_base(stm32f103_gpio_port_t port) {
    switch (port) {
        case STM32F103_GPIO_PORT_A: return GPIOA_BASE;
        case STM32F103_GPIO_PORT_B: return GPIOB_BASE;
        case STM32F103_GPIO_PORT_C: return GPIOC_BASE;
        case STM32F103_GPIO_PORT_D: return GPIOD_BASE;
        case STM32F103_GPIO_PORT_E: return GPIOE_BASE;
        default: return 0;
    }
}

uint32_t stm32f103_get_gpio_clk_mask(stm32f103_gpio_port_t port) {
    switch (port) {
        case STM32F103_GPIO_PORT_A: return RCC_APB2ENR_IOPAEN;
        case STM32F103_GPIO_PORT_B: return RCC_APB2ENR_IOPBEN;
        case STM32F103_GPIO_PORT_C: return RCC_APB2ENR_IOPCEN;
        case STM32F103_GPIO_PORT_D: return RCC_APB2ENR_IOPDEN;
        case STM32F103_GPIO_PORT_E: return RCC_APB2ENR_IOPEEN;
        default: return 0;
    }
}

static void stm32f103_delay_systick(uint32_t ticks) {
    SYSTICK_RVR = ticks - 1;
    SYSTICK_CVR = 0;
    SYSTICK_CSR = SYSTICK_CSR_ENABLE | SYSTICK_CSR_CLKSOURCE;
    
    while (!(SYSTICK_CSR & SYSTICK_CSR_COUNTFLAG));
    
    SYSTICK_CSR = 0;
}

void stm32f103_delay_us(uint32_t microseconds) {
    // Get actual system clock frequency from hardware registers
    uint32_t sysclk_hz = stm32f103_get_sysclk_hz();
    uint32_t ticks_per_us = sysclk_hz / 1000000;  // Ticks per microsecond
    
    uint32_t ticks = microseconds * ticks_per_us;
    if (ticks < 16777216) {  // SysTick is 24-bit
        stm32f103_delay_systick(ticks);
    } else {
        // For long delays, break into smaller chunks
        uint32_t max_us = 16777215 / ticks_per_us;
        while (microseconds > max_us) {
            stm32f103_delay_systick(16777215);
            microseconds -= max_us;
        }
        if (microseconds > 0) {
            stm32f103_delay_systick(microseconds * ticks_per_us);
        }
    }
}

void stm32f103_delay_ms(uint32_t milliseconds) {
    for (uint32_t i = 0; i < milliseconds; i++) {
        stm32f103_delay_us(1000);
    }
}

int stm32f103_system_init(const stm32f103_system_config_t *config) {
    if (!config) {
        return -1; // Invalid configuration
    }
    
    // Initialize SysTick if requested
    if (config->enable_systick) {
        uint32_t sysclk = stm32f103_get_sysclk_hz();
        
        if (config->systick_freq_hz > 0) {
            // Configure SysTick for requested frequency
            uint32_t reload_value = (sysclk / config->systick_freq_hz) - 1;
            if (reload_value > 0xFFFFFF) {
                return -2; // SysTick reload value too large
            }
            
            SYSTICK_RVR = reload_value;
            SYSTICK_CVR = 0;
            SYSTICK_CSR = SYSTICK_CSR_ENABLE | SYSTICK_CSR_CLKSOURCE | SYSTICK_CSR_TICKINT;
        }
    }
    
    // TODO: Clock configuration based on target_sysclk_hz could be added here
    // For now, we work with whatever the system boots with
    
    return 0; // Success
}

int stm32f103_uart_debug_init(void) {
    // Enable clocks for UART1, GPIOA, and AFIO
    RCC_APB2ENR |= RCC_APB2ENR_USART1EN | RCC_APB2ENR_IOPAEN | RCC_APB2ENR_AFIOEN;
    
    // Configure PA9 (TX) as alternate function push-pull
    // Clear PA9 configuration bits (bits 7:4 in CRH)
    uint32_t gpioa_base = GPIOA_BASE;
    volatile uint32_t* gpioa_crh = (volatile uint32_t*)(gpioa_base + 0x04);
    
    *gpioa_crh &= ~(0xF << 4); // Clear PA9 config bits
    *gpioa_crh |= (0xB << 4);  // MODE=11 (50MHz), CNF=10 (Alt func push-pull)
    
    // Configure PA10 (RX) as input floating
    *gpioa_crh &= ~(0xF << 8); // Clear PA10 config bits  
    *gpioa_crh |= (0x4 << 8);  // MODE=00 (input), CNF=01 (floating)
    
    // Configure UART1
    uint32_t sysclk = stm32f103_get_sysclk_hz();
    uint32_t baud = 115200;
    UART1_BRR = sysclk / baud; // Simple baud rate calculation
    
    UART1_CR1 = UART_CR1_UE | UART_CR1_TE; // Enable UART and transmitter
    
    return 0;
}

void stm32f103_uart_debug_print(const char* str) {
    if (!str) return;
    
    while (*str) {
        // Wait for transmit buffer to be empty
        while (!(UART1_SR & UART_SR_TXE));
        
        // Send character
        UART1_DR = *str;
        str++;
    }
    
    // Wait for transmission to complete
    while (!(UART1_SR & UART_SR_TC));
}

void stm32f103_uart_debug_print_num(const char* str, uint32_t num) {
    stm32f103_uart_debug_print(str);
    
    // Simple number to string conversion
    if (num == 0) {
        stm32f103_uart_debug_print("0\r\n");
        return;
    }
    
    char buffer[12]; // Enough for 32-bit number
    int i = 0;
    
    while (num > 0) {
        buffer[i++] = '0' + (num % 10);
        num /= 10;
    }
    
    // Print digits in reverse order
    while (i > 0) {
        i--;
        while (!(UART1_SR & UART_SR_TXE));
        UART1_DR = buffer[i];
    }
    
    stm32f103_uart_debug_print("\r\n");
}