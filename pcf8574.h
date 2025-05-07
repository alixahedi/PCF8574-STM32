/*
 *  pcf8574.h
 *
 *  Commented PCF8574 I2C GPIO expander driver header
 *
 *  Created on: Aug 26, 2024
 *  Author: Alixahedi
 */

#ifndef PCF8574_H
#define PCF8574_H

#include "stm32h7xx_hal.h"

/**
 * @brief  Base 7-bit I2C address for PCF8574.
 */
#define PCF8574_ADDRESS_BASE      0x40

/**
 * @brief  Handle structure for PCF8574 device.
 *         Contains I2C handle, device address, and current port state.
 */
typedef struct {
    I2C_HandleTypeDef *hi2c; /**< I2C handler for communication */
    uint8_t address;         /**< Full I2C address including A0-A2 bits */
    uint8_t portState;       /**< Cached state of the 8 GPIO pins */
} PCF8574_HandleTypeDef;

/**
 * @brief  Initializes the PCF8574 device and sets all pins low.
 * @param  pcf8574: Pointer to PCF8574 handle to initialize.
 * @param  hi2c   : I2C handle for communication.
 * @param  a0     : Logic level for address bit A0 (0 or 1).
 * @param  a1     : Logic level for address bit A1 (0 or 1).
 * @param  a2     : Logic level for address bit A2 (0 or 1).
 * @retval None
 */
void PCF8574_Init(PCF8574_HandleTypeDef *pcf8574, I2C_HandleTypeDef *hi2c, uint8_t a0, uint8_t a1, uint8_t a2);

/**
 * @brief  Sets or clears a single pin on the PCF8574 and updates the device.
 * @param  pcf8574: Pointer to PCF8574 handle.
 * @param  pin    : Pin number (0-7) to modify.
 * @param  state  : Desired GPIO pin state (GPIO_PIN_SET or GPIO_PIN_RESET).
 * @retval None
 */
void PCF8574_WritePin(PCF8574_HandleTypeDef *pcf8574, uint8_t pin, GPIO_PinState state);

/**
 * @brief  Reads the state of a single pin from the PCF8574.
 * @param  pcf8574: Pointer to PCF8574 handle.
 * @param  pin    : Pin number (0-7) to read.
 * @retval GPIO_PIN_SET if high, GPIO_PIN_RESET if low
 */
GPIO_PinState PCF8574_ReadPin(PCF8574_HandleTypeDef *pcf8574, uint8_t pin);

/**
 * @brief  Toggles the state of a single pin on the PCF8574.
 * @param  pcf8574: Pointer to PCF8574 handle.
 * @param  pin    : Pin number (0-7) to toggle.
 * @retval None
 */
void PCF8574_TogglePin(PCF8574_HandleTypeDef *pcf8574, uint8_t pin);

/**
 * @brief  Reads all 8 GPIO pins and updates the cached portState.
 * @param  pcf8574: Pointer to PCF8574 handle.
 * @retval Current 8-bit port state.
 */
uint8_t PCF8574_ReadPort(PCF8574_HandleTypeDef *pcf8574);

/**
 * @brief  Writes all 8 GPIO pins at once.
 * @param  pcf8574: Pointer to PCF8574 handle.
 * @param  value  : 8-bit value representing states of all pins.
 * @retval None
 */
void PCF8574_WritePort(PCF8574_HandleTypeDef *pcf8574, uint8_t value);

#endif // PCF8574_H
