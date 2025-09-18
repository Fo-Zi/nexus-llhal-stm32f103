#ifndef STM32F103_SPI_H
#define STM32F103_SPI_H

#include <stdint.h>
#include <stdbool.h>
#include "nhal_spi_types.h"
#include "nhal_pin_types.h"
#include "stm32f103_spi_defs.h"

/**
 * @file stm32f103_spi.h
 * @brief STM32F103 SPI hardware definitions
 */

#ifdef __cplusplus
extern "C" {
#endif

/*==============================================================================
 * STM32F103 SPI Hardware Definitions
 *============================================================================*/

// All SPI type definitions (enums, structs) are now in stm32f103_spi_defs.h

/*==============================================================================
 * STM32F103 SPI Pin Definitions
 *============================================================================*/

// SPI1 pins (APB2)
#define STM32F103_SPI1_SCK_PA5      {STM32F103_GPIO_PORT_A, STM32F103_GPIO_PIN_5}
#define STM32F103_SPI1_MISO_PA6     {STM32F103_GPIO_PORT_A, STM32F103_GPIO_PIN_6}
#define STM32F103_SPI1_MOSI_PA7     {STM32F103_GPIO_PORT_A, STM32F103_GPIO_PIN_7}
#define STM32F103_SPI1_NSS_PA4      {STM32F103_GPIO_PORT_A, STM32F103_GPIO_PIN_4}

// SPI2 pins (APB1)
#define STM32F103_SPI2_SCK_PB13     {STM32F103_GPIO_PORT_B, STM32F103_GPIO_PIN_13}
#define STM32F103_SPI2_MISO_PB14    {STM32F103_GPIO_PORT_B, STM32F103_GPIO_PIN_14}
#define STM32F103_SPI2_MOSI_PB15    {STM32F103_GPIO_PORT_B, STM32F103_GPIO_PIN_15}
#define STM32F103_SPI2_NSS_PB12     {STM32F103_GPIO_PORT_B, STM32F103_GPIO_PIN_12}

// SPI3 pins (APB1) - 64/100 pin packages only
#define STM32F103_SPI3_SCK_PB3      {STM32F103_GPIO_PORT_B, STM32F103_GPIO_PIN_3}
#define STM32F103_SPI3_MISO_PB4     {STM32F103_GPIO_PORT_B, STM32F103_GPIO_PIN_4}
#define STM32F103_SPI3_MOSI_PB5     {STM32F103_GPIO_PORT_B, STM32F103_GPIO_PIN_5}
#define STM32F103_SPI3_NSS_PA15     {STM32F103_GPIO_PORT_A, STM32F103_GPIO_PIN_15}

/*==============================================================================
 * STM32F103 SPI Context and Implementation Config
 *============================================================================*/

// Context and config structures are defined in stm32f103_spi_defs.h

/*==============================================================================
 * STM32F103 SPI Functions (internal use)
 *============================================================================*/

int stm32f103_spi_init(stm32f103_spi_id_t spi_id, const struct nhal_spi_config *nhal_config, uint32_t clock_freq_hz);
int stm32f103_spi_transfer(stm32f103_spi_id_t spi_id, const uint8_t *tx_data, uint8_t *rx_data, size_t length, uint32_t timeout_ms);
stm32f103_spi_prescaler_t stm32f103_spi_calculate_prescaler(stm32f103_spi_id_t spi_id, uint32_t desired_freq_hz, uint32_t *actual_freq_hz);
bool stm32f103_spi_is_busy(stm32f103_spi_id_t spi_id);

#ifdef __cplusplus
}
#endif

#endif /* STM32F103_SPI_H */