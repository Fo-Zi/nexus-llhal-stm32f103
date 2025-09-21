#include "nhal_spi_master.h"
#include "stm32f103_spi.h"
#include "stm32f103_spi_defs.h"
#include "stm32f103_gpio.h"
#include "stm32f103_timing.h"
#include "stm32f103_registers.h"
#include "nhal_pin_stm32f103.h"

/**
 * @file nhal_spi_master_stm32f103.c
 * @brief STM32F103 implementation of NHAL SPI Master interface
 */

static uint32_t get_spi_base(uint8_t spi_id) {
    switch (spi_id) {
        case 1: return SPI1_BASE;
        case 2: return SPI2_BASE;
        case 3: return SPI3_BASE;
        default: return 0;
    }
}

/**
 * @brief Enable SPI peripheral clock
 */
static nhal_result_t enable_spi_clock(stm32f103_spi_id_t spi_id) {
    switch (spi_id) {
        case STM32F103_SPI_1: {
            volatile uint32_t *rcc_apb2enr = (volatile uint32_t *)(RCC_BASE + RCC_APB2ENR_OFFSET);
            *rcc_apb2enr |= RCC_APB2ENR_SPI1EN;
            break;
        }
        case STM32F103_SPI_2: {
            volatile uint32_t *rcc_apb1enr = (volatile uint32_t *)(RCC_BASE + RCC_APB1ENR_OFFSET);
            *rcc_apb1enr |= RCC_APB1ENR_SPI2EN;
            break;
        }
        case STM32F103_SPI_3: {
            volatile uint32_t *rcc_apb1enr = (volatile uint32_t *)(RCC_BASE + RCC_APB1ENR_OFFSET);
            *rcc_apb1enr |= RCC_APB1ENR_SPI3EN;
            break;
        }
        default:
            return NHAL_ERR_INVALID_ARG;
    }
    return NHAL_OK;
}

/**
 * @brief Wait for SPI flag with timeout
 */
static nhal_result_t wait_for_spi_flag(uint32_t base, uint32_t flag, bool set, uint32_t timeout_ms) {
    uint32_t start_tick = stm32f103_get_tick();
    volatile uint32_t *sr = (volatile uint32_t *)(base + SPI_SR_OFFSET);

    while (timeout_ms == 0 || !stm32f103_timeout_occurred(start_tick, timeout_ms)) {
        uint32_t sr_val = *sr;
        if (set && (sr_val & flag)) return NHAL_OK;
        if (!set && !(sr_val & flag)) return NHAL_OK;
    }
    return NHAL_ERR_TIMEOUT;
}

/**
 * @brief Calculate SPI baud rate prescaler
 */
static uint32_t calculate_spi_prescaler(uint32_t pclk_hz, uint32_t target_hz) {
    // SPI1 is on APB2 (up to 72MHz), SPI2/3 are on APB1 (up to 36MHz)
    uint32_t prescaler_value = pclk_hz / target_hz;

    // Find the closest prescaler (2, 4, 8, 16, 32, 64, 128, 256)
    if (prescaler_value <= 2) return 0; // /2
    if (prescaler_value <= 4) return 1; // /4
    if (prescaler_value <= 8) return 2; // /8
    if (prescaler_value <= 16) return 3; // /16
    if (prescaler_value <= 32) return 4; // /32
    if (prescaler_value <= 64) return 5; // /64
    if (prescaler_value <= 128) return 6; // /128
    return 7; // /256
}

static nhal_result_t configure_spi_pins(stm32f103_spi_id_t spi_id, bool use_remap, bool use_hardware_cs) {
    stm32f103_gpio_config_t pin_config;
    struct nhal_pin_id sck_pin, miso_pin, mosi_pin, nss_pin;

    // Select pins based on SPI ID and remap setting
    switch (spi_id) {
        case STM32F103_SPI_1:
            if (use_remap) {
                // SPI1 remap uses the same pins as SPI3
                sck_pin.port = STM32F103_GPIO_PORT_B;
                sck_pin.pin = STM32F103_GPIO_PIN_3;
                miso_pin.port = STM32F103_GPIO_PORT_B;
                miso_pin.pin = STM32F103_GPIO_PIN_4;
                mosi_pin.port = STM32F103_GPIO_PORT_B;
                mosi_pin.pin = STM32F103_GPIO_PIN_5;
                nss_pin.port = STM32F103_GPIO_PORT_A;
                nss_pin.pin = STM32F103_GPIO_PIN_15;
            } else {
                sck_pin.port = STM32F103_GPIO_PORT_A;
                sck_pin.pin = STM32F103_GPIO_PIN_5;
                miso_pin.port = STM32F103_GPIO_PORT_A;
                miso_pin.pin = STM32F103_GPIO_PIN_6;
                mosi_pin.port = STM32F103_GPIO_PORT_A;
                mosi_pin.pin = STM32F103_GPIO_PIN_7;
                nss_pin.port = STM32F103_GPIO_PORT_A;
                nss_pin.pin = STM32F103_GPIO_PIN_4;
            }
            break;
        case STM32F103_SPI_2:
            sck_pin.port = STM32F103_GPIO_PORT_B;
            sck_pin.pin = STM32F103_GPIO_PIN_13;
            miso_pin.port = STM32F103_GPIO_PORT_B;
            miso_pin.pin = STM32F103_GPIO_PIN_14;
            mosi_pin.port = STM32F103_GPIO_PORT_B;
            mosi_pin.pin = STM32F103_GPIO_PIN_15;
            nss_pin.port = STM32F103_GPIO_PORT_B;
            nss_pin.pin = STM32F103_GPIO_PIN_12;
            break;
        case STM32F103_SPI_3:
            sck_pin.port = STM32F103_GPIO_PORT_B;
            sck_pin.pin = STM32F103_GPIO_PIN_3;
            miso_pin.port = STM32F103_GPIO_PORT_B;
            miso_pin.pin = STM32F103_GPIO_PIN_4;
            mosi_pin.port = STM32F103_GPIO_PORT_B;
            mosi_pin.pin = STM32F103_GPIO_PIN_5;
            nss_pin.port = STM32F103_GPIO_PORT_A;
            nss_pin.pin = STM32F103_GPIO_PIN_15;
            break;
        default:
            return NHAL_ERR_INVALID_ARG;
    }

    // Configure SCK - alternate function push-pull
    pin_config.port = sck_pin.port;
    pin_config.pin = sck_pin.pin;
    pin_config.mode = STM32F103_GPIO_MODE_OUTPUT_50MHz;
    pin_config.cnf = STM32F103_GPIO_CNF_ALTFN_PUSH_PULL;
    if (stm32f103_gpio_configure_pin(&pin_config) != 0) {
        return NHAL_ERR_HW_FAILURE;
    }

    // Configure MOSI - alternate function push-pull
    pin_config.port = mosi_pin.port;
    pin_config.pin = mosi_pin.pin;
    pin_config.mode = STM32F103_GPIO_MODE_OUTPUT_50MHz;
    pin_config.cnf = STM32F103_GPIO_CNF_ALTFN_PUSH_PULL;
    if (stm32f103_gpio_configure_pin(&pin_config) != 0) {
        return NHAL_ERR_HW_FAILURE;
    }

    // Configure MISO - input floating
    pin_config.port = miso_pin.port;
    pin_config.pin = miso_pin.pin;
    pin_config.mode = STM32F103_GPIO_MODE_INPUT;
    pin_config.cnf = STM32F103_GPIO_CNF_INPUT_FLOATING;
    if (stm32f103_gpio_configure_pin(&pin_config) != 0) {
        return NHAL_ERR_HW_FAILURE;
    }

    // Configure NSS/CS pin
    pin_config.port = nss_pin.port;
    pin_config.pin = nss_pin.pin;
    pin_config.mode = STM32F103_GPIO_MODE_OUTPUT_50MHz;
    if (use_hardware_cs) {
        pin_config.cnf = STM32F103_GPIO_CNF_ALTFN_PUSH_PULL;
    } else {
        pin_config.cnf = STM32F103_GPIO_CNF_OUTPUT_PUSH_PULL;
        // Set CS high (inactive) for software control
        stm32f103_gpio_set_pin(nss_pin.port, nss_pin.pin);
    }
    if (stm32f103_gpio_configure_pin(&pin_config) != 0) {
        return NHAL_ERR_HW_FAILURE;
    }

    return NHAL_OK;
}

nhal_result_t nhal_spi_master_init(struct nhal_spi_context *ctx) {
    if (!ctx) {
        return NHAL_ERR_INVALID_ARG;
    }

    // Validate SPI ID (base address derivation will be done as needed)
    if (get_spi_base(ctx->spi_id) == 0) {
        return NHAL_ERR_INVALID_ARG;
    }

    // Pin configuration will be done in set_config when config is provided
    ctx->is_initialized = true;
    ctx->is_configured = false; // Will be set true when configured
    return NHAL_OK;
}

nhal_result_t nhal_spi_master_deinit(struct nhal_spi_context *ctx) {
    if (!ctx) {
        return NHAL_ERR_INVALID_ARG;
    }

    ctx->is_initialized = false;
    ctx->is_configured = false;
    return NHAL_OK;
}

nhal_result_t nhal_spi_master_set_config(struct nhal_spi_context *ctx, struct nhal_spi_config *config) {
    if (!ctx || !config || !ctx->is_initialized || !config->impl_config) {
        return NHAL_ERR_INVALID_ARG;
    }

    uint32_t base = get_spi_base(ctx->spi_id);
    if (base == 0) {
        return NHAL_ERR_INVALID_ARG;
    }

    // Enable SPI clock
    nhal_result_t result = enable_spi_clock(ctx->spi_id);
    if (result != NHAL_OK) {
        return result;
    }

    // Configure pins based on SPI ID and remap setting from config
    result = configure_spi_pins(ctx->spi_id, config->impl_config->use_remap, config->impl_config->use_hardware_cs);
    if (result != NHAL_OK) {
        return result;
    }

    // Disable SPI before configuration
    volatile uint32_t *cr1 = (volatile uint32_t *)(base + SPI_CR1_OFFSET);
    *cr1 &= ~SPI_CR1_SPE;

    // Configure CR1 register
    uint32_t cr1_val = 0;

    // Set master mode
    cr1_val |= SPI_CR1_MSTR;

    // Set clock polarity and phase based on SPI mode
    switch (config->mode) {
        case NHAL_SPI_MODE_0: // CPOL=0, CPHA=0
            break;
        case NHAL_SPI_MODE_1: // CPOL=0, CPHA=1
            cr1_val |= SPI_CR1_CPHA;
            break;
        case NHAL_SPI_MODE_2: // CPOL=1, CPHA=0
            cr1_val |= SPI_CR1_CPOL;
            break;
        case NHAL_SPI_MODE_3: // CPOL=1, CPHA=1
            cr1_val |= SPI_CR1_CPOL | SPI_CR1_CPHA;
            break;
        default:
            return NHAL_ERR_INVALID_ARG;
    }

    // Set bit order
    if (config->bit_order == NHAL_SPI_BIT_ORDER_LSB_FIRST) {
        cr1_val |= SPI_CR1_LSBFIRST;
    }

    // Calculate and set baud rate prescaler
    uint32_t pclk_hz;
    if (ctx->spi_id == STM32F103_SPI_1) {
        pclk_hz = 72000000; // APB2 clock (up to 72MHz)
    } else {
        pclk_hz = 36000000; // APB1 clock (up to 36MHz)
    }

    uint32_t prescaler = calculate_spi_prescaler(pclk_hz, config->impl_config->clock_freq_hz);
    cr1_val |= (prescaler << SPI_CR1_BR_Pos);

    // Set software slave management if not using hardware CS
    if (!config->impl_config->use_hardware_cs) {
        cr1_val |= SPI_CR1_SSM | SPI_CR1_SSI;
    }

    // Configure duplex mode
    if (config->duplex == NHAL_SPI_HALF_DUPLEX) {
        cr1_val |= SPI_CR1_BIDIMODE;
    }

    // Write CR1 configuration
    *cr1 = cr1_val;

    // Configure CR2 register
    volatile uint32_t *cr2 = (volatile uint32_t *)(base + SPI_CR2_OFFSET);
    uint32_t cr2_val = 0;

    // Enable SS output if using hardware CS
    if (config->impl_config->use_hardware_cs) {
        cr2_val |= SPI_CR2_SSOE;
    }

    *cr2 = cr2_val;

    // Enable SPI
    *cr1 |= SPI_CR1_SPE;

    ctx->is_configured = true;
    return NHAL_OK;
}

nhal_result_t nhal_spi_master_get_config(struct nhal_spi_context *ctx, struct nhal_spi_config *config) {
    if (!ctx || !config || !ctx->is_initialized) {
        return NHAL_ERR_INVALID_ARG;
    }

    return NHAL_ERR_UNSUPPORTED;
}

nhal_result_t nhal_spi_master_write(struct nhal_spi_context *ctx, const uint8_t *data, size_t len) {
    if (!ctx || !data || !ctx->is_initialized || !ctx->is_configured) {
        return NHAL_ERR_INVALID_ARG;
    }

    if (len == 0) {
        return NHAL_OK;
    }

    uint32_t base = get_spi_base(ctx->spi_id);
    if (base == 0) {
        return NHAL_ERR_INVALID_ARG;
    }

    volatile uint32_t *cr1 = (volatile uint32_t *)(base + SPI_CR1_OFFSET);
    volatile uint32_t *dr = (volatile uint32_t *)(base + SPI_DR_OFFSET);
    uint32_t timeout_ms = 1000; // Default timeout

    // Check if we're in half-duplex mode
    bool is_half_duplex = (*cr1 & SPI_CR1_BIDIMODE) != 0;

    for (size_t i = 0; i < len; i++) {
        // Enable output for each byte in half-duplex mode
        if (is_half_duplex) {
            *cr1 |= SPI_CR1_BIDIOE;
        }

        // Send data
        *dr = data[i];

        // Wait for TX buffer to be empty
        nhal_result_t result = wait_for_spi_flag(base, SPI_SR_TXE, true, timeout_ms);
        if (result != NHAL_OK) {
            if (is_half_duplex) {
                *cr1 &= ~SPI_CR1_BIDIOE; // Clear BIDIOE on error
            }
            return result;
        }

        // Wait for transmission to complete for this byte
        if (is_half_duplex) {
            // In half-duplex mode, wait for BSY to clear (transmission done)
            result = wait_for_spi_flag(base, SPI_SR_BSY, false, timeout_ms);
            if (result != NHAL_OK) {
                *cr1 &= ~SPI_CR1_BIDIOE; // Clear BIDIOE on error
                return result;
            }
            // Clear output enable after each byte
            *cr1 &= ~SPI_CR1_BIDIOE;
        } else {
            // Wait for RX buffer to have data (dummy read in full-duplex mode)
            result = wait_for_spi_flag(base, SPI_SR_RXNE, true, timeout_ms);
            if (result != NHAL_OK) {
                return result;
            }

            // Read and discard received data
            (void)*dr;
        }
    }

    // Final wait for transmission to complete (full-duplex only)
    if (!is_half_duplex) {
        nhal_result_t result = wait_for_spi_flag(base, SPI_SR_BSY, false, timeout_ms);
        if (result != NHAL_OK) {
            return result;
        }
    }

    return NHAL_OK;
}

nhal_result_t nhal_spi_master_read(struct nhal_spi_context *ctx, uint8_t *data, size_t len) {
    if (!ctx || !data || !ctx->is_initialized || !ctx->is_configured) {
        return NHAL_ERR_INVALID_ARG;
    }

    if (len == 0) {
        return NHAL_OK;
    }

    uint32_t base = get_spi_base(ctx->spi_id);
    if (base == 0) {
        return NHAL_ERR_INVALID_ARG;
    }

    volatile uint32_t *dr = (volatile uint32_t *)(base + SPI_DR_OFFSET);
    uint32_t timeout_ms = 1000; // Default timeout

    for (size_t i = 0; i < len; i++) {
        // Wait for TX buffer to be empty
        nhal_result_t result = wait_for_spi_flag(base, SPI_SR_TXE, true, timeout_ms);
        if (result != NHAL_OK) {
            return result;
        }

        // Send dummy byte to generate clock
        *dr = 0xFF;

        // Wait for RX buffer to have data
        result = wait_for_spi_flag(base, SPI_SR_RXNE, true, timeout_ms);
        if (result != NHAL_OK) {
            return result;
        }

        // Read received data
        data[i] = (uint8_t)*dr;
    }

    // Wait for transmission to complete
    nhal_result_t result = wait_for_spi_flag(base, SPI_SR_BSY, false, timeout_ms);
    if (result != NHAL_OK) {
        return result;
    }

    return NHAL_OK;
}

nhal_result_t nhal_spi_master_write_read(struct nhal_spi_context *ctx,
                                        const uint8_t *tx_data, size_t tx_len,
                                        uint8_t *rx_data, size_t rx_len) {
    if (!ctx || !ctx->is_initialized || !ctx->is_configured) {
        return NHAL_ERR_INVALID_ARG;
    }

    // Validate parameters based on what data is provided
    if ((tx_len > 0 && !tx_data) || (rx_len > 0 && !rx_data)) {
        return NHAL_ERR_INVALID_ARG;
    }

    if (tx_len == 0 && rx_len == 0) {
        return NHAL_OK;
    }

    uint32_t base = get_spi_base(ctx->spi_id);
    if (base == 0) {
        return NHAL_ERR_INVALID_ARG;
    }

    volatile uint32_t *dr = (volatile uint32_t *)(base + SPI_DR_OFFSET);
    uint32_t timeout_ms = 1000; // Default timeout

    size_t max_len = (tx_len > rx_len) ? tx_len : rx_len;

    for (size_t i = 0; i < max_len; i++) {
        // Wait for TX buffer to be empty
        nhal_result_t result = wait_for_spi_flag(base, SPI_SR_TXE, true, timeout_ms);
        if (result != NHAL_OK) {
            return result;
        }

        // Send data (or dummy byte if no more tx data)
        uint8_t tx_byte = (i < tx_len) ? tx_data[i] : 0xFF;
        *dr = tx_byte;

        // Wait for RX buffer to have data
        result = wait_for_spi_flag(base, SPI_SR_RXNE, true, timeout_ms);
        if (result != NHAL_OK) {
            return result;
        }

        // Read received data (or discard if no more rx buffer space)
        uint8_t rx_byte = (uint8_t)*dr;
        if (i < rx_len) {
            rx_data[i] = rx_byte;
        }
    }

    // Wait for transmission to complete
    nhal_result_t result = wait_for_spi_flag(base, SPI_SR_BSY, false, timeout_ms);
    if (result != NHAL_OK) {
        return result;
    }

    return NHAL_OK;
}
