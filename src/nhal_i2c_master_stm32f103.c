#include "nhal_i2c_master.h"
#include "nhal_i2c_transfer.h"
#include "stm32f103_i2c_defs.h"
#include "stm32f103_gpio.h"
#include "stm32f103_registers.h"
#include "stm32f103_timing.h"

/**
 * @file nhal_i2c_master_stm32f103.c
 * @brief STM32F103 implementation of NHAL I2C Master interface
 */

/*==============================================================================
 * Private Helper Functions  
 *============================================================================*/

/**
 * @brief Get I2C base address from peripheral ID
 */
static uint32_t get_i2c_base(stm32f103_i2c_id_t i2c_id) {
    switch (i2c_id) {
        case STM32F103_I2C_1: return I2C1_BASE;
        case STM32F103_I2C_2: return I2C2_BASE;
        default: return 0;
    }
}

/**
 * @brief Enable I2C peripheral clock
 */
static nhal_result_t enable_i2c_clock(stm32f103_i2c_id_t i2c_id) {
    volatile uint32_t *rcc_apb1enr = (volatile uint32_t *)(RCC_BASE + RCC_APB1ENR_OFFSET);
    
    switch (i2c_id) {
        case STM32F103_I2C_1:
            *rcc_apb1enr |= RCC_APB1ENR_I2C1EN;
            break;
        case STM32F103_I2C_2:
            *rcc_apb1enr |= RCC_APB1ENR_I2C2EN;
            break;
        default:
            return NHAL_ERR_INVALID_ARG;
    }
    return NHAL_OK;
}

/**
 * @brief Wait for I2C flag with timeout
 */
static nhal_result_t wait_for_flag(uint32_t base, uint16_t reg_offset, uint32_t flag, bool set, uint32_t timeout_ms) {
    uint32_t start_tick = stm32f103_get_tick();
    volatile uint32_t *reg = (volatile uint32_t *)(base + reg_offset);
    
    while (timeout_ms == 0 || !stm32f103_timeout_occurred(start_tick, timeout_ms)) {
        uint32_t reg_val = *reg;
        if (set && (reg_val & flag)) return NHAL_OK;
        if (!set && !(reg_val & flag)) return NHAL_OK;
    }
    return NHAL_ERR_TIMEOUT;
}

/**
 * @brief Clear I2C error flags
 */
static void clear_error_flags(uint32_t base) {
    volatile uint32_t *sr1 = (volatile uint32_t *)(base + I2C_SR1_OFFSET);
    *sr1 &= ~(I2C_SR1_BERR | I2C_SR1_ARLO | I2C_SR1_AF | I2C_SR1_OVR | I2C_SR1_TIMEOUT);
}

nhal_result_t nhal_i2c_master_init(struct nhal_i2c_context *ctx) {
    if (!ctx) {
        return NHAL_ERR_INVALID_ARG;
    }

    // Validate I2C ID (base address derivation will be done as needed)
    uint32_t base = get_i2c_base(ctx->i2c_id);
    if (base == 0) {
        return NHAL_ERR_INVALID_ARG;
    }

    // Enable I2C peripheral clock
    nhal_result_t result = enable_i2c_clock(ctx->i2c_id);
    if (result != NHAL_OK) {
        return result;
    }

    // Reset I2C peripheral
    volatile uint32_t *cr1 = (volatile uint32_t *)(base + I2C_CR1_OFFSET);
    *cr1 |= I2C_CR1_SWRST;
    *cr1 &= ~I2C_CR1_SWRST;

    // GPIO pin configuration will be done in set_config when config is provided
    ctx->is_initialized = true;
    ctx->is_configured = false; // Will be set true when configured
    return NHAL_OK;
}

nhal_result_t nhal_i2c_master_deinit(struct nhal_i2c_context *ctx) {
    if (!ctx) {
        return NHAL_ERR_INVALID_ARG;
    }
    
    ctx->is_initialized = false;
    ctx->is_configured = false;
    return NHAL_OK;
}

nhal_result_t nhal_i2c_master_set_config(struct nhal_i2c_context *ctx, struct nhal_i2c_config *config) {
    if (!ctx || !config || !ctx->is_initialized || !config->impl_config) {
        return NHAL_ERR_INVALID_ARG;
    }
    
    struct nhal_i2c_impl_config *impl = config->impl_config;
    stm32f103_gpio_config_t pin_config;

    // Configure I2C pins based on I2C ID and remap setting
    // I2C1: Normal (PB6/PB7), Remap (PB8/PB9)
    // I2C2: PB10/PB11 (no remap)
    
    if (ctx->i2c_id == STM32F103_I2C_1) {
        if (impl->use_remap) {
            // I2C1 remap: SCL=PB8, SDA=PB9
            pin_config.port = STM32F103_GPIO_PORT_B;
            pin_config.pin = STM32F103_GPIO_PIN_8;
        } else {
            // I2C1 normal: SCL=PB6, SDA=PB7
            pin_config.port = STM32F103_GPIO_PORT_B;
            pin_config.pin = STM32F103_GPIO_PIN_6;
        }
    } else if (ctx->i2c_id == STM32F103_I2C_2) {
        // I2C2: SCL=PB10, SDA=PB11 (no remap option)
        pin_config.port = STM32F103_GPIO_PORT_B;
        pin_config.pin = STM32F103_GPIO_PIN_10;
    } else {
        return NHAL_ERR_INVALID_ARG;
    }
    
    // Configure SCL pin
    pin_config.mode = STM32F103_GPIO_MODE_OUTPUT_50MHz;
    pin_config.cnf = STM32F103_GPIO_CNF_ALTFN_OPEN_DRAIN;
    if (stm32f103_gpio_configure_pin(&pin_config) != 0) {
        return NHAL_ERR_HW_FAILURE;
    }

    // Configure SDA pin
    if (ctx->i2c_id == STM32F103_I2C_1) {
        if (impl->use_remap) {
            pin_config.pin = STM32F103_GPIO_PIN_9;  // PB9
        } else {
            pin_config.pin = STM32F103_GPIO_PIN_7;  // PB7
        }
    } else if (ctx->i2c_id == STM32F103_I2C_2) {
        pin_config.pin = STM32F103_GPIO_PIN_11;  // PB11
    }
    
    if (stm32f103_gpio_configure_pin(&pin_config) != 0) {
        return NHAL_ERR_HW_FAILURE;
    }

    // Configure I2C peripheral
    uint32_t base = get_i2c_base(ctx->i2c_id);
    volatile uint32_t *cr2 = (volatile uint32_t *)(base + I2C_CR2_OFFSET);
    volatile uint32_t *ccr = (volatile uint32_t *)(base + I2C_CCR_OFFSET);
    volatile uint32_t *trise = (volatile uint32_t *)(base + I2C_TRISE_OFFSET);
    volatile uint32_t *cr1 = (volatile uint32_t *)(base + I2C_CR1_OFFSET);

    // Set peripheral clock frequency (assuming 36MHz APB1 clock)
    *cr2 = (*cr2 & ~I2C_CR2_FREQ_Msk) | (36 & I2C_CR2_FREQ_Msk);

    // Configure clock control register for desired frequency
    uint32_t clock_speed = impl->clock_freq_hz;
    if (clock_speed <= 100000) {
        // Standard mode (up to 100kHz)
        uint32_t ccr_value = 36000000 / (2 * clock_speed);
        if (ccr_value < 4) ccr_value = 4;
        *ccr = ccr_value & I2C_CCR_CCR_Msk;
        *trise = 37; // (1000ns / 27.8ns) + 1
    } else {
        // Fast mode (up to 400kHz)
        uint32_t ccr_value = 36000000 / (3 * clock_speed);
        if (ccr_value < 1) ccr_value = 1;
        *ccr = I2C_CCR_FS | (ccr_value & I2C_CCR_CCR_Msk);
        *trise = 11; // (300ns / 27.8ns) + 1
    }

    // Enable I2C peripheral
    *cr1 |= I2C_CR1_PE;

    ctx->is_configured = true;
    return NHAL_OK;
}

nhal_result_t nhal_i2c_master_get_config(struct nhal_i2c_context *ctx, struct nhal_i2c_config *config) {
    if (!ctx || !config || !ctx->is_initialized) {
        return NHAL_ERR_INVALID_ARG;
    }
    
    return NHAL_ERR_UNSUPPORTED;
}

nhal_result_t nhal_i2c_master_write(struct nhal_i2c_context *ctx, 
                                   nhal_i2c_address_t dev_address,
                                   const uint8_t *data, size_t len) {
    if (!ctx || !data || len == 0 || !ctx->is_initialized || !ctx->is_configured) {
        return NHAL_ERR_INVALID_ARG;
    }
    
    // Create a single write operation
    nhal_i2c_transfer_op_t write_op = {
        .type = NHAL_I2C_WRITE_OP,
        .address = {0}, // Use default address (will use dev_address parameter)
        .flags = 0,     // Standard operation with START and STOP
        .write = {
            .bytes = data,
            .length = len
        }
    };
    
    // Perform the transfer using the core function
    return nhal_i2c_master_perform_transfer(ctx, dev_address, &write_op, 1);
}

nhal_result_t nhal_i2c_master_read(struct nhal_i2c_context *ctx,
                                  nhal_i2c_address_t dev_address, 
                                  uint8_t *data, size_t len) {
    if (!ctx || !data || len == 0 || !ctx->is_initialized || !ctx->is_configured) {
        return NHAL_ERR_INVALID_ARG;
    }
    
    // Create a single read operation
    nhal_i2c_transfer_op_t read_op = {
        .type = NHAL_I2C_READ_OP,
        .address = {0}, // Use default address (will use dev_address parameter)
        .flags = 0,     // Standard operation with START and STOP
        .read = {
            .buffer = data,
            .length = len
        }
    };
    
    // Perform the transfer using the core function
    return nhal_i2c_master_perform_transfer(ctx, dev_address, &read_op, 1);
}

nhal_result_t nhal_i2c_master_write_read_reg(struct nhal_i2c_context *ctx,
                                            nhal_i2c_address_t dev_address,
                                            const uint8_t *reg_address, size_t reg_len,
                                            uint8_t *data, size_t data_len) {
    if (!ctx || !reg_address || !data || reg_len == 0 || data_len == 0 || 
        !ctx->is_initialized || !ctx->is_configured) {
        return NHAL_ERR_INVALID_ARG;
    }
    
    // Create write and read operations for register access
    nhal_i2c_transfer_op_t ops[2] = {
        {
            .type = NHAL_I2C_WRITE_OP,
            .address = {0}, // Use default address
            .flags = NHAL_I2C_TRANSFER_MSG_NO_STOP, // Don't send STOP after write
            .write = {
                .bytes = reg_address,
                .length = reg_len
            }
        },
        {
            .type = NHAL_I2C_READ_OP,
            .address = {0}, // Use default address  
            .flags = 0,     // Standard read with START and STOP
            .read = {
                .buffer = data,
                .length = data_len
            }
        }
    };
    
    // Perform the combined transfer (write then repeated start + read)
    return nhal_i2c_master_perform_transfer(ctx, dev_address, ops, 2);
}

/*==============================================================================
 * Core I2C Transfer Implementation
 *============================================================================*/

nhal_result_t nhal_i2c_master_perform_transfer(struct nhal_i2c_context *ctx,
                                             nhal_i2c_address_t dev_address,
                                             nhal_i2c_transfer_op_t *ops,
                                             size_t num_ops) {
    if (!ctx || !ops || num_ops == 0 || !ctx->is_initialized || !ctx->is_configured) {
        return NHAL_ERR_INVALID_ARG;
    }
    
    uint32_t base = get_i2c_base(ctx->i2c_id);
    volatile uint32_t *cr1 = (volatile uint32_t *)(base + I2C_CR1_OFFSET);
    volatile uint32_t *dr = (volatile uint32_t *)(base + I2C_DR_OFFSET);
    volatile uint32_t *sr1 = (volatile uint32_t *)(base + I2C_SR1_OFFSET);
    volatile uint32_t *sr2 = (volatile uint32_t *)(base + I2C_SR2_OFFSET);
    
    // Wait for bus to be free
    if (wait_for_flag(base, I2C_SR2_OFFSET, I2C_SR2_BUSY, false, 1000) != NHAL_OK) {
        return NHAL_ERR_TIMEOUT;
    }
    
    bool transaction_started = false;
    
    for (size_t op_idx = 0; op_idx < num_ops; op_idx++) {
        nhal_i2c_transfer_op_t *op = &ops[op_idx];
        bool need_start = !transaction_started || !(op->flags & NHAL_I2C_TRANSFER_MSG_NO_START);
        bool need_stop = !(op->flags & NHAL_I2C_TRANSFER_MSG_NO_STOP) && 
                        (op_idx == num_ops - 1 || !(ops[op_idx + 1].flags & NHAL_I2C_TRANSFER_MSG_NO_START));
        bool need_addr = !(op->flags & NHAL_I2C_TRANSFER_MSG_NO_ADDR);
        
        // Generate start condition if needed
        if (need_start) {
            *cr1 |= I2C_CR1_START;
            
            // Wait for start bit
            if (wait_for_flag(base, I2C_SR1_OFFSET, I2C_SR1_SB, true, 1000) != NHAL_OK) {
                clear_error_flags(base);
                return NHAL_ERR_TIMEOUT;
            }
            transaction_started = true;
        }
        
        // Send address if needed
        if (need_addr) {
            // Use the device address from the operation or fallback to the main address
            nhal_i2c_address_t addr = (op->address.type != 0) ? op->address : dev_address;
            uint8_t addr_byte = (addr.type == NHAL_I2C_7BIT_ADDR) ? 
                                addr.addr.address_7bit : (addr.addr.address_10bit & 0xFF);
            
            if (op->type == NHAL_I2C_READ_OP) {
                *dr = (addr_byte << 1) | 0x01; // Read mode
                if (op->read.length == 1) {
                    *cr1 &= ~I2C_CR1_ACK; // Disable ACK for single byte read
                } else {
                    *cr1 |= I2C_CR1_ACK; // Enable ACK for multi-byte read
                }
            } else {
                *dr = (addr_byte << 1) & 0xFE; // Write mode
            }
            
            // Wait for address acknowledgment
            if (wait_for_flag(base, I2C_SR1_OFFSET, I2C_SR1_ADDR, true, 1000) != NHAL_OK) {
                *cr1 |= I2C_CR1_STOP;
                clear_error_flags(base);
                return NHAL_ERR_TIMEOUT;
            }
            
            // Clear ADDR flag by reading SR1 then SR2
            (void)*sr1;
            (void)*sr2;
        }
        
        // Perform the actual operation
        if (op->type == NHAL_I2C_WRITE_OP) {
            // Write operation
            const uint8_t *write_data = op->write.bytes;
            size_t write_len = op->write.length;
            
            if (!write_data || write_len == 0) {
                *cr1 |= I2C_CR1_STOP;
                return NHAL_ERR_INVALID_ARG;
            }
            
            for (size_t i = 0; i < write_len; i++) {
                // Wait for TXE flag
                if (wait_for_flag(base, I2C_SR1_OFFSET, I2C_SR1_TXE, true, 1000) != NHAL_OK) {
                    *cr1 |= I2C_CR1_STOP;
                    clear_error_flags(base);
                    return NHAL_ERR_TIMEOUT;
                }
                
                // Send data byte
                *dr = write_data[i];
            }
            
            // Wait for BTF (Byte Transfer Finished) after last byte
            if (wait_for_flag(base, I2C_SR1_OFFSET, I2C_SR1_BTF, true, 1000) != NHAL_OK) {
                *cr1 |= I2C_CR1_STOP;
                clear_error_flags(base);
                return NHAL_ERR_TIMEOUT;
            }
            
        } else if (op->type == NHAL_I2C_READ_OP) {
            // Read operation
            uint8_t *read_data = op->read.buffer;
            size_t read_len = op->read.length;
            
            if (!read_data || read_len == 0) {
                *cr1 |= I2C_CR1_STOP;
                return NHAL_ERR_INVALID_ARG;
            }
            
            if (read_len == 1 && need_stop) {
                // Single byte read with stop - generate stop after clearing ADDR
                *cr1 |= I2C_CR1_STOP;
            }
            
            // Read data bytes
            for (size_t i = 0; i < read_len; i++) {
                if (i == read_len - 2 && read_len > 1 && need_stop) {
                    // For second to last byte, disable ACK
                    *cr1 &= ~I2C_CR1_ACK;
                }
                if (i == read_len - 1 && read_len > 1 && need_stop) {
                    // For last byte, generate stop condition
                    *cr1 |= I2C_CR1_STOP;
                }
                
                // Wait for RXNE flag
                if (wait_for_flag(base, I2C_SR1_OFFSET, I2C_SR1_RXNE, true, 1000) != NHAL_OK) {
                    *cr1 |= I2C_CR1_STOP;
                    clear_error_flags(base);
                    return NHAL_ERR_TIMEOUT;
                }
                
                // Read data byte
                read_data[i] = (uint8_t)(*dr & 0xFF);
            }
        }
        
        // Generate stop condition if needed
        if (need_stop && op->type == NHAL_I2C_WRITE_OP) {
            *cr1 |= I2C_CR1_STOP;
            transaction_started = false;
        }
    }
    
    return NHAL_OK;
}