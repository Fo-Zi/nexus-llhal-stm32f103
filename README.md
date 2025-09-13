# STM32F103 Low-Level Hardware Abstraction Layer (LLHAL)

This is an STM32F103-specific implementation of the NHAL (NEXUS Hardware Abstraction Layer) interface. It provides direct hardware access and peripheral control for STM32F103 microcontrollers through a modular architecture.

## Dependencies

**Critical:** This LLHAL implementation requires the NHAL interface definitions from the `hal-interface` project:
- `nhal_pin.h` - Pin control interface definitions
- `nhal_common.h` - Common result types and timing functions
- `nhal_pin_types.h` - Pin data structure definitions

The LLHAL acts as a concrete implementation of these abstract interfaces, translating NHAL calls into STM32F103-specific register operations.

## Architecture

The implementation is organized into focused, single-responsibility modules:

```
nexus-llhal-stm32f103/
├── include/
│   ├── stm32f103_clock.h      # Clock system configuration
│   ├── stm32f103_timing.h     # SysTick and DWT timing
│   ├── stm32f103_gpio.h       # GPIO pin control utilities
│   ├── stm32f103_uart.h       # UART peripheral management
│   ├── stm32f103_mcu.h        # MCU package pin definitions
│   ├── stm32f103_registers.h  # Hardware register mappings
│   ├── stm32f103_helpers.h    # Legacy compatibility header
│   └── nhal_pin_stm32f103.h   # NHAL pin interface implementation
└── src/
    ├── stm32f103_clock.c      # Clock tree initialization
    ├── stm32f103_timing.c     # Delay and timing functions
    ├── stm32f103_gpio.c       # GPIO configuration and control
    ├── stm32f103_uart.c       # UART communication
    ├── nhal_pin_stm32f103.c   # NHAL interface implementation
    └── nhal_common.c          # NHAL timing function implementations
```

## Implemented Features

### Clock Management (`stm32f103_clock.h/.c`)
- HSI, HSE, and PLL clock source configuration with validation
- 72MHz system clock setup from external crystal
- Peripheral clock enable/disable by register offset
- System reset and recovery functions
- Clock frequency retrieval for other modules

**Limitations:** No support for CSS (Clock Security System) or MCO (Master Clock Output).

### Timing Services (`stm32f103_timing.h/.c`)
- SysTick-based millisecond counter with interrupt handling
- DWT cycle counter for microsecond precision delays
- Timeout checking utilities for peripheral operations
- NHAL timing interface implementation

### GPIO Control (`stm32f103_gpio.h/.c`)
- Pin configuration with mode and CNF register control
- Individual pin set/clear/read operations
- Port-level bulk operations
- Pin availability checking by MCU package (48/64/100-pin)
- Clock management for GPIO ports

**Limitations:** No analog pin support or alternate function mapping utilities.

### UART Communication (`stm32f103_uart.h/.c`)
- Support for UART1, UART2, UART3 peripherals
- Configurable baud rates, word length, parity, stop bits
- Debug UART functions for development
- Timeout-based blocking operations

**Limitations:** No interrupt-driven or DMA-based communication, no hardware flow control implementation.

### NHAL Interface (`nhal_pin_stm32f103.h/.c`)
- Complete implementation of NHAL pin interface
- GPIO pin configuration and state management
- External interrupt (EXTI) support with individual and shared handlers
- Resource conflict detection (EXTI line ownership)

**Limitations:** No support for analog pins or special function pins beyond basic GPIO.

### MCU Package Support (`stm32f103_mcu.h`)
- Pin availability definitions for 48-pin (STM32F103C8T6), 64-pin, and 100-pin packages
- Package-aware pin validation

## System Initialization Example

A possible platform_init method example:

```c
#include "platform_config.h"
#include "stm32f103_clock.h"
#include "stm32f103_uart.h"
#include "stm32f103_timing.h"

nhal_result_t platform_init(void) {
    // Configure and initialize 72MHz system clock from HSE
    stm32f103_clock_config_t clock_config;
    stm32f103_get_default_72mhz_config(&clock_config);

    if (stm32f103_clock_init(&clock_config, NULL) != 0) {
        return NHAL_ERR_HW_FAILURE;
    }

    // Initialize debug UART (typically UART1 on PA9/PA10)
    stm32f103_uart_debug_init();

    // Initialize SysTick timer for system timing
    nhal_systick_init();

    // Configure logical pins with hardware mappings
    // PIN_LED_BUILTIN -> PC13, PIN_DHT11_DATA -> PA2, etc.
    setup_pin_mappings();
    setup_pin_contexts();

    // Initialize and configure all defined pins through NHAL
    for (int i = 0; i < PIN_TOTAL_NUM; i++) {
        nhal_result_t result = nhal_pin_init(&pin_contexts[i]);
        if (result != NHAL_OK) return result;

        result = nhal_pin_set_config(&pin_contexts[i], &pin_configs[i]);
        if (result != NHAL_OK) return result;
    }

    return NHAL_OK;
}
```

## Usage Examples

### Direct GPIO Control
```c
#include "stm32f103_gpio.h"

// Configure PC13 as output (Blue Pill LED)
stm32f103_gpio_pin_config_t led_config = {
    .port = STM32F103_GPIO_PORT_C,
    .pin = STM32F103_GPIO_PIN_13,
    .mode = GPIO_MODE_OUTPUT_50M,
    .cnf = GPIO_CNF_OUTPUT_PUSHPULL
};
stm32f103_gpio_configure_pin(&led_config);

// Toggle LED
stm32f103_gpio_set_pin(STM32F103_GPIO_PORT_C, STM32F103_GPIO_PIN_13);
stm32f103_delay_ms(500);
stm32f103_gpio_clear_pin(STM32F103_GPIO_PORT_C, STM32F103_GPIO_PIN_13);
```

### NHAL Interface Usage
```c
#include "nhal_pin.h"

// Use pre-configured platform pins
struct nhal_pin_context *led_ctx = platform_get_pin_ctx(PIN_LED_BUILTIN);

// Control through NHAL interface
nhal_pin_set_state(led_ctx, NHAL_PIN_HIGH);
nhal_delay_milliseconds(500);
nhal_pin_set_state(led_ctx, NHAL_PIN_LOW);
```

### Debug Output
```c
#include "stm32f103_uart.h"

// Send debug messages (UART must be initialized in platform_init)
stm32f103_uart_debug_print("System initialized\r\n");
stm32f103_uart_debug_print_num("Clock frequency: ", stm32f103_get_sysclk_hz());
```

## Build Requirements

- ARM GCC toolchain (`arm-none-eabi-gcc`)
- CMake 3.16+
- Access to `hal-interface` headers
- STM32F103 target hardware

## Current Limitations

- **No SPI or I2C implementations** - Only GPIO, UART, and timing are implemented
- **No PWM or advanced timer functions** - Basic timing only
- **No DMA support** - All operations are blocking/polling
- **No low-power modes** - Standard run mode only
- **No ADC/DAC support** - Digital I/O only
- **Limited UART features** - Basic communication without advanced flow control
- **No USB support** - Peripheral not implemented

## Integration Notes

This LLHAL is designed to work with the broader NEXUS ecosystem:
- Applications use logical pin definitions (e.g., `PIN_LED_BUILTIN`)
- Platform layer maps logical pins to physical STM32F103 pins
- NHAL interface provides hardware abstraction for portable code
- Direct STM32F103 functions available for performance-critical code

The modular design allows mixing NHAL abstracted code with direct hardware access as needed.
