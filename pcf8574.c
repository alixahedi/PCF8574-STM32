/*
 * pcf8574.c
 *
 *  Commented PCF8574 I2C GPIO expander driver implementation
 *  
 *  Created on: Aug 26, 2024
 *  Author: Alixahedi
 */

#include "pcf8574.h"

/**
 * @brief  Initializes the PCF8574 device and sets all pins low.
 * @param  pcf8574: Pointer to PCF8574 handle to initialize.
 * @param  hi2c   : I2C handle for communication.
 * @param  a0     : Logic level for address bit A0 (0 or 1).
 * @param  a1     : Logic level for address bit A1 (0 or 1).
 * @param  a2     : Logic level for address bit A2 (0 or 1).
 * @retval None
 */
void PCF8574_Init(PCF8574_HandleTypeDef *pcf8574, I2C_HandleTypeDef *hi2c, uint8_t a0, uint8_t a1, uint8_t a2) {
    pcf8574->hi2c = hi2c;
    pcf8574->address = PCF8574_ADDRESS_BASE | (a0 << 1) | (a1 << 2) | (a2 << 3);
    pcf8574->portState = 0x00;
    PCF8574_WritePort(pcf8574, pcf8574->portState);
}

/**
 * @brief  Sets or clears a single pin on the PCF8574 and writes the new state.
 * @param  pcf8574: Pointer to PCF8574 handle.
 * @param  pin    : Pin number (0-7) to modify.
 * @param  state  : Desired GPIO pin state.
 * @retval None
 */
void PCF8574_WritePin(PCF8574_HandleTypeDef *pcf8574, uint8_t pin, GPIO_PinState state) {
    if (state == GPIO_PIN_RESET) {
        pcf8574->portState &= ~(1 << pin);
    } else {
        pcf8574->portState |= (1 << pin);
    }
    PCF8574_WritePort(pcf8574, pcf8574->portState);
}

/**
 * @brief  Reads the state of a single pin from the PCF8574.
 * @param  pcf8574: Pointer to PCF8574 handle.
 * @param  pin    : Pin number (0-7) to read.
 * @retval GPIO_PIN_SET if high, GPIO_PIN_RESET if low
 */
GPIO_PinState PCF8574_ReadPin(PCF8574_HandleTypeDef *pcf8574, uint8_t pin) {
    PCF8574_ReadPort(pcf8574);
    return (pcf8574->portState & (1 << pin)) ? GPIO_PIN_SET : GPIO_PIN_RESET;
}

/**
 * @brief  Toggles a single pin on the PCF8574 and writes the new state.
 * @param  pcf8574: Pointer to PCF8574 handle.
 * @param  pin    : Pin number (0-7) to toggle.
 * @retval None
 */
void PCF8574_TogglePin(PCF8574_HandleTypeDef *pcf8574, uint8_t pin) {
    pcf8574->portState ^= (1 << pin);
    PCF8574_WritePort(pcf8574, pcf8574->portState);
}

/**
 * @brief  Writes the entire 8-bit port state to the PCF8574 via I2C.
 * @param  pcf8574: Pointer to PCF8574 handle.
 * @param  value  : 8-bit value representing pin states.
 * @retval None
 */
void PCF8574_WritePort(PCF8574_HandleTypeDef *pcf8574, uint8_t value) {
    pcf8574->portState = value;
    HAL_I2C_Master_Transmit(pcf8574->hi2c, pcf8574->address, &(pcf8574->portState), 1, HAL_MAX_DELAY);
}

/**
 * @brief  Reads the entire 8-bit port state from the PCF8574 via I2C.
 * @param  pcf8574: Pointer to PCF8574 handle.
 * @retval Current 8-bit value of all pins.
 */
uint8_t PCF8574_ReadPort(PCF8574_HandleTypeDef *pcf8574) {
    HAL_I2C_Master_Receive(pcf8574->hi2c, pcf8574->address, &(pcf8574->portState), 1, HAL_MAX_DELAY);
    return pcf8574->portState;
}
