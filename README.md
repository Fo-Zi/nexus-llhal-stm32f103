# STM32F103 Low-Level Hardware Abstraction Layer

![GitHub License](https://img.shields.io/github/license/Fo-Zi/nexus-llhal-stm32f103?color=lightgrey)
![Status](https://img.shields.io/badge/status-development-yellow)
[![Dependency](https://img.shields.io/badge/depends%20on-nexus--hal--interface%20v0.6.0-orange)](https://github.com/Fo-Zi/nexus-hal-interface/tree/v0.6.0)

A register-level hardware abstraction layer for STM32F103 microcontrollers that implements the NHAL interface while providing direct hardware access. Part of the [Nexus Ecosystem](https://github.com/Fo-Zi/nexus-embedded-ecosystem).

## Architecture

This implementation uses a dual-layer approach that bridges register-level hardware control with portable abstractions:

**Direct Hardware Layer**: Register-level functions for immediate hardware control
**NHAL Interface Layer**: Context-based abstraction for portability across MCUs

```
Application Code
├── NHAL Interface (portable)          ├── Direct Hardware Access (performance)
│   └── nhal_pin_set_state()          │   └── stm32f103_gpio_set_pin()
└── Platform Layer (logical → physical pin mapping)
    └── STM32F103 Register Layer (stm32f103_gpio.c)
        └── Hardware Registers
```

### Key Design Decisions

**Context-Based Resource Management**
- Static allocation of all contexts at compile time
- No dynamic memory allocation or runtime conflicts
- Platform layer maps logical pins (PIN_LED_BUILTIN) to hardware (PC13)

**Modular Peripheral Organization**
- Each peripheral is an independent module with separate headers
- Headers split into interface (.h) and definitions (_defs.h)
- Can mix direct hardware access with NHAL interface calls

**Three-Phase Lifecycle**
- Uninitialized → Initialized → Configured → Operational
- Clear error codes for wrong-state operations
- Consistent state management across all peripherals

## Module Structure

```
include/
├── stm32f103.h                 # Convenience header including all modules
├── stm32f103_registers.h       # Register base addresses and bit definitions
├── stm32f103_clock.h          # System and peripheral clock configuration
├── stm32f103_timing.h         # SysTick and DWT-based delay functions
├── stm32f103_gpio.h           # GPIO configuration and control
├── stm32f103_uart.h           # UART communication
├── stm32f103_spi.h            # SPI peripheral interface and definitions
├── stm32f103_i2c.h            # I2C peripheral interface and definitions
└── nhal_pin_stm32f103.h       # NHAL pin interface implementation

src/
├── stm32f103_clock.c          # Clock tree initialization
├── stm32f103_timing.c         # Delay and timeout functions
├── stm32f103_gpio.c           # Pin control implementation
├── stm32f103_uart.c           # UART direct hardware access
├── nhal_pin_stm32f103.c       # NHAL pin interface implementation
├── nhal_uart_stm32f103.c      # NHAL UART interface implementation
├── nhal_spi_master_stm32f103.c # NHAL SPI master implementation
└── nhal_i2c_master_stm32f103.c # NHAL I2C master implementation
```

## Hardware Support

**MCU Variants**: STM32F103C8T6, C6T6, R8T6, RBT6, and other 103 series
**Packages**: 48-pin, 64-pin, 100-pin variants with package-aware pin validation
**Clock Sources**: HSI (8MHz), HSE (4-16MHz), PLL (up to 72MHz)
**Peripherals**: GPIO, UART1-3, SPI1-3, I2C1-2, SysTick, DWT, EXTI

## Implemented Peripherals

### Clock Management
- System clock configuration up to 72MHz from HSI, HSE, or PLL
- Peripheral clock enable/disable for power management
- Clock validation and automatic recovery from failures

### GPIO Control
- Pin configuration with mode, speed, and CNF register control
- Digital I/O operations with package-aware pin validation
- External interrupt (EXTI) support with callbacks

### UART Communication (1-3)
- Full NHAL interface with init/config/read/write operations
- Baud rates up to 115200, configurable data/parity/stop bits
- Timeout-based blocking operations
- Pin remapping support for alternate pin configurations

### SPI Master (1-3)
- Complete NHAL SPI master interface implementation
- Modes 0-3 (CPOL/CPHA configuration)
- MSB/LSB first bit order
- Full-duplex and half-duplex modes
- Hardware and software chip select control
- Pin remapping support

### I2C Master (1-2)
- Full NHAL I2C master interface implementation
- Standard (100kHz) and fast (400kHz) modes
- 7-bit addressing support
- Register read/write operations (write-then-read)
- Advanced transfer operations with START/STOP control
- Pin remapping support

### Timing Services
- SysTick integration with 1ms tick counter
- DWT cycle counter for microsecond precision delays
- Timeout utilities for peripheral operations

## Usage Examples

### Platform Initialization

```c
#include "stm32f103_clock.h"
#include "stm32f103_timing.h"

nhal_result_t platform_init(void) {
    // Get default 72MHz configuration
    stm32f103_clock_config_t clock_config;
    stm32f103_get_default_72mhz_config(&clock_config);

    // Initialize clock system with automatic recovery
    if (stm32f103_clock_init(&clock_config, NULL) != 0) {
        return NHAL_ERR_HW_FAILURE;
    }

    // Initialize timing system
    stm32f103_timing_init();
    return NHAL_OK;
}
```

### Direct Hardware Access

```c
#include "stm32f103_gpio.h"

// Configure Blue Pill LED (PC13) as output
stm32f103_gpio_config_t led_config = {
    .port = STM32F103_GPIO_PORT_C,
    .pin = STM32F103_GPIO_PIN_13,
    .mode = STM32F103_GPIO_MODE_OUTPUT_50MHz,
    .cnf = STM32F103_GPIO_CNF_OUTPUT_PUSH_PULL
};
stm32f103_gpio_configure_pin(&led_config);

// Control LED directly
stm32f103_gpio_set_pin(STM32F103_GPIO_PORT_C, STM32F103_GPIO_PIN_13);
stm32f103_delay_ms(500);
stm32f103_gpio_clear_pin(STM32F103_GPIO_PORT_C, STM32F103_GPIO_PIN_13);
```

### NHAL Interface Usage

```c
#include "nhal_pin.h"

// Application uses logical pin names
struct nhal_pin_context *led_ctx = platform_get_pin_ctx(PIN_LED_BUILTIN);

// Portable API calls
nhal_pin_set_state(led_ctx, NHAL_PIN_HIGH);
nhal_delay_milliseconds(500);
nhal_pin_set_state(led_ctx, NHAL_PIN_LOW);
```

### SPI Communication

```c
#include "nhal_spi_master.h"

// Direct HAL usage - allocate and configure SPI1
struct nhal_spi_context spi_ctx;
struct nhal_spi_impl_config spi_impl_config;
struct nhal_spi_config spi_config = {
    .duplex = NHAL_SPI_FULL_DUPLEX,
    .mode = NHAL_SPI_MODE_0,
    .bit_order = NHAL_SPI_BIT_ORDER_MSB_FIRST,
    .impl_config = &spi_impl_config
};

// Configure STM32F103-specific settings
spi_impl_config.clock_freq_hz = 8000000;  // 8MHz
spi_impl_config.use_hardware_cs = false;  // Software CS control
spi_impl_config.use_remap = false;        // Use default pins PA5/PA7

// Initialize SPI1
spi_ctx.spi_id = 1;
nhal_spi_master_init(&spi_ctx);
nhal_spi_master_set_config(&spi_ctx, &spi_config);

// Send data to LCD
uint8_t cmd = 0x2A;  // Column address set
nhal_spi_master_write(&spi_ctx, &cmd, 1);
```

### I2C Communication

```c
#include "nhal_i2c_master.h"

// Direct HAL usage - allocate and configure I2C1
struct nhal_i2c_context i2c_ctx;
struct nhal_i2c_impl_config i2c_impl_config;
struct nhal_i2c_config i2c_config = {
    .impl_config = &i2c_impl_config
};

// Configure STM32F103-specific settings
i2c_impl_config.clock_freq_hz = 100000;  // 100kHz standard mode
i2c_impl_config.use_remap = false;       // Use PB6/PB7

// Initialize I2C1
i2c_ctx.i2c_id = STM32F103_I2C_1;
nhal_i2c_master_init(&i2c_ctx);
nhal_i2c_master_set_config(&i2c_ctx, &i2c_config);

// Read sensor register
nhal_i2c_address_t sensor_addr = {
    .type = NHAL_I2C_7BIT_ADDR,
    .addr.address_7bit = 0x48
};

uint8_t reg_addr = 0x00;
uint8_t sensor_data[2];
nhal_i2c_master_write_read_reg(&i2c_ctx, sensor_addr, &reg_addr, 1, sensor_data, 2);
```

### Platform Integration Example

From nexus-showcase-project, showing how applications use logical pins:

```c
// platform_config.c - maps logical to physical pins
pin_ids[PIN_LED_BUILTIN] = (struct nhal_pin_id){
    .port = STM32F103_GPIO_PORT_C,
    .pin = STM32F103_GPIO_PIN_13
};

pin_ids[PIN_LCD_DC] = (struct nhal_pin_id){
    .port = STM32F103_GPIO_PORT_A,
    .pin = STM32F103_GPIO_PIN_0
};
```

## Implementation Status

### Working Features
- Clock system (HSI, HSE, PLL up to 72MHz)
- GPIO configuration and control with EXTI interrupts
- UART communication with NHAL interface
- SPI master (1-3) with full NHAL interface
- I2C master (1-2) with full NHAL interface
- SysTick and DWT-based timing functions
- Context-based resource management

### Current Limitations
- No DMA support - all operations are blocking/polling
- No PWM or advanced timer features
- No ADC, DAC, or USB peripherals
- No low-power mode support
- UART hardware flow control not implemented

### Known Tradeoffs

**Static vs Dynamic Allocation**
- All contexts are statically allocated at compile time
- Eliminates runtime allocation failures but requires upfront planning
- Memory usage is predictable but may waste space for unused features

**Direct Register Access vs Abstraction**
- Minimal abstraction over hardware registers for predictable performance
- Requires more STM32F103-specific knowledge but offers full control
- NHAL layer provides portability when needed

**Polling vs Interrupt-Driven**
- Current implementation uses polling for simplicity and determinism
- Higher CPU usage but more predictable timing behavior
- No complex interrupt state management or race conditions

## Integration Notes

### Build Requirements
- ARM GCC toolchain (arm-none-eabi-gcc)
- CMake 3.16 or newer
- Access to nexus-hal-interface headers
- STM32F103 target hardware for testing

### Platform Layer Responsibilities
Applications should provide a platform layer that:
- Defines logical pin constants (PIN_LED_BUILTIN, PIN_BUTTON_USER, etc.)
- Maps logical pins to physical STM32F103 pins
- Manages context allocation and initialization
- Provides debug utilities if needed

### Error Handling
All NHAL functions return standardized error codes:
- `NHAL_OK`: Success
- `NHAL_ERR_INVALID_ARG`: Invalid parameters
- `NHAL_ERR_NOT_INITIALIZED`: Context not initialized
- `NHAL_ERR_NOT_CONFIGURED`: Context not configured
- `NHAL_ERR_HW_FAILURE`: Hardware operation failed
- `NHAL_ERR_TIMEOUT`: Operation timed out

This implementation balances direct hardware control with portable abstractions. Use direct functions for performance-critical code and NHAL interface for portable applications. The complete SPI and I2C implementations make it suitable for interfacing with displays, sensors, and other peripheral devices.
