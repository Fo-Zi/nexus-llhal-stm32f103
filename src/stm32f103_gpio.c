#include "stm32f103_gpio.h"
#include "stm32f103_clock.h"
#include "stm32f103_registers.h"

/**
 * @file stm32f103_gpio.c
 * @brief STM32F103 GPIO Utilities Implementation
 */

/*==============================================================================
 * Public Functions - GPIO Port Utilities
 *============================================================================*/

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

/*==============================================================================
 * Pin Availability Functions
 *============================================================================*/

bool stm32f103_is_pin_available(stm32f103_package_t package, stm32f103_pin_t pin) {
    switch (package) {
        case STM32F103_PACKAGE_48PIN_LQFP:
            // 48-pin package limitations (Blue Pill - STM32F103C8T6)
            switch (pin.port) {
                case STM32F103_GPIO_PORT_A:
                    return true; // PA0-PA15 all available
                    
                case STM32F103_GPIO_PORT_B:
                    return true; // PB0-PB15 all available
                    
                case STM32F103_GPIO_PORT_C:
                    // Only PC13, PC14, PC15 available in 48-pin
                    return (pin.pin >= STM32F103_GPIO_PIN_13);
                    
                case STM32F103_GPIO_PORT_D:
                    // Only PD0, PD1 available (OSC pins)
                    return (pin.pin <= STM32F103_GPIO_PIN_1);
                    
                case STM32F103_GPIO_PORT_E:
                    return false; // Port E not available in 48-pin
                    
                default:
                    return false;
            }
            
        case STM32F103_PACKAGE_64PIN_LQFP:
            // 64-pin package has more pins available
            switch (pin.port) {
                case STM32F103_GPIO_PORT_A:
                case STM32F103_GPIO_PORT_B:
                    return true; // All pins available
                    
                case STM32F103_GPIO_PORT_C:
                    return true; // PC0-PC15 all available
                    
                case STM32F103_GPIO_PORT_D:
                    return (pin.pin <= STM32F103_GPIO_PIN_2); // PD0-PD2
                    
                case STM32F103_GPIO_PORT_E:
                    return false; // Still not available in 64-pin
                    
                default:
                    return false;
            }
            
        case STM32F103_PACKAGE_100PIN_LQFP:
            // 100-pin package has all pins available
            switch (pin.port) {
                case STM32F103_GPIO_PORT_A:
                case STM32F103_GPIO_PORT_B:
                case STM32F103_GPIO_PORT_C:
                case STM32F103_GPIO_PORT_D:
                    return true; // All pins available
                    
                case STM32F103_GPIO_PORT_E:
                    return true; // PE0-PE15 available
                    
                default:
                    return false;
            }
            
        default:
            return false;
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

int stm32f103_gpio_enable_port_clock(stm32f103_gpio_port_t port) {
    uint32_t clock_mask = stm32f103_get_gpio_clk_mask(port);
    if (clock_mask == 0) {
        return -1; // Invalid port
    }
    
    return stm32f103_enable_peripheral_clock(RCC_APB2ENR_OFFSET, clock_mask);
}

int stm32f103_gpio_disable_port_clock(stm32f103_gpio_port_t port) {
    uint32_t clock_mask = stm32f103_get_gpio_clk_mask(port);
    if (clock_mask == 0) {
        return -1; // Invalid port
    }
    
    return stm32f103_disable_peripheral_clock(RCC_APB2ENR_OFFSET, clock_mask);
}

/*==============================================================================
 * Public Functions - GPIO Pin Configuration
 *============================================================================*/

void stm32f103_gpio_configure_pin_raw(uint32_t gpio_base, uint8_t pin, uint8_t mode, uint8_t cnf) {
    if (pin > 15) return; // Invalid pin
    
    volatile uint32_t *cr_reg = (pin < 8) ?
        (volatile uint32_t *)(gpio_base + 0x00) : // CRL
        (volatile uint32_t *)(gpio_base + 0x04);  // CRH
    
    uint8_t shift = (pin % 8) * 4;
    uint32_t mask = 0xF << shift;
    uint32_t config = ((cnf << 2) | mode) << shift;
    
    *cr_reg = (*cr_reg & ~mask) | config;
}

int stm32f103_gpio_configure_pin(const stm32f103_gpio_config_t *config) {
    if (!config) {
        return -1;
    }
    
    if (!stm32f103_gpio_is_valid_port(config->port) || 
        !stm32f103_gpio_is_valid_pin(config->pin)) {
        return -1;
    }
    
    // Enable port clock if not already enabled
    stm32f103_gpio_enable_port_clock(config->port);
    
    // Get port base address
    uint32_t gpio_base = stm32f103_get_gpio_base(config->port);
    
    // Configure the pin
    stm32f103_gpio_configure_pin_raw(gpio_base, config->pin, config->mode, config->cnf);
    
    return 0;
}

int stm32f103_gpio_configure_pins(stm32f103_gpio_port_t port, uint16_t pin_mask,
                                  stm32f103_gpio_mode_t mode, stm32f103_gpio_cnf_t cnf) {
    if (!stm32f103_gpio_is_valid_port(port)) {
        return -1;
    }
    
    // Enable port clock if not already enabled
    stm32f103_gpio_enable_port_clock(port);
    
    // Get port base address
    uint32_t gpio_base = stm32f103_get_gpio_base(port);
    
    // Configure each pin specified in the mask
    for (int pin = 0; pin < 16; pin++) {
        if (pin_mask & (1 << pin)) {
            stm32f103_gpio_configure_pin_raw(gpio_base, pin, mode, cnf);
        }
    }
    
    return 0;
}

/*==============================================================================
 * Public Functions - GPIO Pin Control
 *============================================================================*/

void stm32f103_gpio_set_pin(stm32f103_gpio_port_t port, stm32f103_gpio_pin_t pin) {
    uint32_t gpio_base = stm32f103_get_gpio_base(port);
    if (gpio_base == 0) return;
    
    volatile uint32_t *bsrr = (volatile uint32_t *)(gpio_base + 0x10); // BSRR offset
    *bsrr = 1 << pin; // Set bit
}

void stm32f103_gpio_clear_pin(stm32f103_gpio_port_t port, stm32f103_gpio_pin_t pin) {
    uint32_t gpio_base = stm32f103_get_gpio_base(port);
    if (gpio_base == 0) return;
    
    volatile uint32_t *bsrr = (volatile uint32_t *)(gpio_base + 0x10); // BSRR offset
    *bsrr = 1 << (pin + 16); // Reset bit (upper 16 bits)
}

void stm32f103_gpio_toggle_pin(stm32f103_gpio_port_t port, stm32f103_gpio_pin_t pin) {
    if (stm32f103_gpio_read_pin(port, pin)) {
        stm32f103_gpio_clear_pin(port, pin);
    } else {
        stm32f103_gpio_set_pin(port, pin);
    }
}

void stm32f103_gpio_write_pin(stm32f103_gpio_port_t port, stm32f103_gpio_pin_t pin, bool value) {
    if (value) {
        stm32f103_gpio_set_pin(port, pin);
    } else {
        stm32f103_gpio_clear_pin(port, pin);
    }
}

bool stm32f103_gpio_read_pin(stm32f103_gpio_port_t port, stm32f103_gpio_pin_t pin) {
    uint32_t gpio_base = stm32f103_get_gpio_base(port);
    if (gpio_base == 0) return false;
    
    volatile uint32_t *idr = (volatile uint32_t *)(gpio_base + 0x08); // IDR offset
    return (*idr & (1 << pin)) != 0;
}

/*==============================================================================
 * Public Functions - GPIO Port Control
 *============================================================================*/

void stm32f103_gpio_write_port(stm32f103_gpio_port_t port, uint16_t value) {
    uint32_t gpio_base = stm32f103_get_gpio_base(port);
    if (gpio_base == 0) return;
    
    volatile uint32_t *odr = (volatile uint32_t *)(gpio_base + 0x0C); // ODR offset
    *odr = value;
}

uint16_t stm32f103_gpio_read_port(stm32f103_gpio_port_t port) {
    uint32_t gpio_base = stm32f103_get_gpio_base(port);
    if (gpio_base == 0) return 0;
    
    volatile uint32_t *idr = (volatile uint32_t *)(gpio_base + 0x08); // IDR offset
    return (uint16_t)(*idr & 0xFFFF);
}

/*==============================================================================
 * Public Functions - Utility Functions
 *============================================================================*/

bool stm32f103_gpio_is_valid_port(stm32f103_gpio_port_t port) {
    return (port <= STM32F103_GPIO_PORT_E);
}

bool stm32f103_gpio_is_valid_pin(stm32f103_gpio_pin_t pin) {
    return (pin <= STM32F103_GPIO_PIN_15);
}

const char* stm32f103_gpio_get_port_name(stm32f103_gpio_port_t port) {
    switch (port) {
        case STM32F103_GPIO_PORT_A: return "PORTA";
        case STM32F103_GPIO_PORT_B: return "PORTB";
        case STM32F103_GPIO_PORT_C: return "PORTC";
        case STM32F103_GPIO_PORT_D: return "PORTD";
        case STM32F103_GPIO_PORT_E: return "PORTE";
        default: return "INVALID";
    }
}