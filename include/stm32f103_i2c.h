#ifndef STM32F103_I2C_H
#define STM32F103_I2C_H

#include <stdint.h>
#include <stdbool.h>
#include "nhal_i2c_types.h"
#include "nhal_pin_types.h"
#include "stm32f103_i2c_defs.h"

/**
 * @file stm32f103_i2c.h
 * @brief STM32F103 I2C hardware definitions
 */

#ifdef __cplusplus
extern "C" {
#endif

/*==============================================================================
 * STM32F103 I2C Hardware Definitions
 *============================================================================*/

// I2C peripheral identifiers and enums are defined in stm32f103_i2c_defs.h

/*==============================================================================
 * STM32F103 I2C Pin Definitions
 *============================================================================*/

// I2C1 pins (APB1)
#define STM32F103_I2C1_SCL_PB6      {STM32F103_GPIO_PORT_B, STM32F103_GPIO_PIN_6}
#define STM32F103_I2C1_SDA_PB7      {STM32F103_GPIO_PORT_B, STM32F103_GPIO_PIN_7}
#define STM32F103_I2C1_SCL_PB8      {STM32F103_GPIO_PORT_B, STM32F103_GPIO_PIN_8}   // Remapped
#define STM32F103_I2C1_SDA_PB9      {STM32F103_GPIO_PORT_B, STM32F103_GPIO_PIN_9}   // Remapped

// I2C2 pins (APB1)
#define STM32F103_I2C2_SCL_PB10     {STM32F103_GPIO_PORT_B, STM32F103_GPIO_PIN_10}
#define STM32F103_I2C2_SDA_PB11     {STM32F103_GPIO_PORT_B, STM32F103_GPIO_PIN_11}

/*==============================================================================
 * STM32F103 I2C Context and Implementation Config
 *============================================================================*/

// Context and config structures are defined in stm32f103_i2c_defs.h

/*==============================================================================
 * STM32F103 I2C Functions (internal use)
 *============================================================================*/

int stm32f103_i2c_init(stm32f103_i2c_id_t i2c_id, uint32_t clock_freq_hz, stm32f103_i2c_duty_t duty_cycle);
int stm32f103_i2c_master_write(stm32f103_i2c_id_t i2c_id, uint16_t dev_addr, const uint8_t *data, size_t length, uint32_t timeout_ms);
int stm32f103_i2c_master_read(stm32f103_i2c_id_t i2c_id, uint16_t dev_addr, uint8_t *data, size_t length, uint32_t timeout_ms);
bool stm32f103_i2c_is_bus_busy(stm32f103_i2c_id_t i2c_id);
int stm32f103_i2c_reset(stm32f103_i2c_id_t i2c_id);

#ifdef __cplusplus
}
#endif

#endif /* STM32F103_I2C_H */