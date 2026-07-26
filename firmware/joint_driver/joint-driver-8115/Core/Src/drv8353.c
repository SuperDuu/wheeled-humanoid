/**
  ******************************************************************************
  * @file           : drv8353.c
  * @brief          : Source file for DRV8353RS Gate Driver SPI Library
  ******************************************************************************
  */

#include "drv8353.h"

/**
  * @brief  Initialize DRV8353 instance
  */
HAL_StatusTypeDef DRV8353_Init(DRV8353_t *drv, SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port, uint16_t cs_pin, GPIO_TypeDef *en_port, uint16_t en_pin)
{
    if (drv == NULL || hspi == NULL) return HAL_ERROR;

    drv->hspi = hspi;
    drv->cs_port = cs_port;
    drv->cs_pin = cs_pin;
    drv->en_port = en_port;
    drv->en_pin = en_pin;
    drv->csa_gain = DRV8353_CSA_GAIN_20V; // Default 20V/V

    // Set CS high
    HAL_GPIO_WritePin(drv->cs_port, drv->cs_pin, GPIO_PIN_SET);

    // Enable DRV8353 (Active High)
    DRV8353_Enable(drv);
    HAL_Delay(2); // Wait 2ms for power-up & wake-up

    // Configure default CSA gain (20V/V)
    return DRV8353_SetCSAGain(drv, drv->csa_gain);
}

/**
  * @brief  Enable DRV8353 Driver (Wake Up)
  */
void DRV8353_Enable(DRV8353_t *drv)
{
    if (drv && drv->en_port) {
        HAL_GPIO_WritePin(drv->en_port, drv->en_pin, GPIO_PIN_SET);
    }
}

/**
  * @brief  Disable DRV8353 Driver (Sleep)
  */
void DRV8353_Disable(DRV8353_t *drv)
{
    if (drv && drv->en_port) {
        HAL_GPIO_WritePin(drv->en_port, drv->en_pin, GPIO_PIN_RESET);
    }
}

/**
  * @brief  Read DRV8353 SPI Register
  */
HAL_StatusTypeDef DRV8353_ReadRegister(DRV8353_t *drv, uint8_t regAddr, uint16_t *regVal)
{
    if (drv == NULL || regVal == NULL) return HAL_ERROR;

    // SPI Read Frame format: Bit 15 = 1 (Read), Bits [14:11] = Reg Addr, Bits [10:0] = 0
    uint16_t txData = (1 << 15) | ((regAddr & 0x0F) << 11);
    uint16_t rxData = 0;

    HAL_GPIO_WritePin(drv->cs_port, drv->cs_pin, GPIO_PIN_RESET);
    HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(drv->hspi, (uint8_t*)&txData, (uint8_t*)&rxData, 1, 10);
    HAL_GPIO_WritePin(drv->cs_port, drv->cs_pin, GPIO_PIN_SET);

    if (status == HAL_OK) {
        *regVal = rxData & 0x07FF; // 11-bit data
    }
    return status;
}

/**
  * @brief  Write DRV8353 SPI Register
  */
HAL_StatusTypeDef DRV8353_WriteRegister(DRV8353_t *drv, uint8_t regAddr, uint16_t regVal)
{
    if (drv == NULL) return HAL_ERROR;

    // SPI Write Frame format: Bit 15 = 0 (Write), Bits [14:11] = Reg Addr, Bits [10:0] = Data
    uint16_t txData = ((regAddr & 0x0F) << 11) | (regVal & 0x07FF);
    uint16_t rxData = 0;

    HAL_GPIO_WritePin(drv->cs_port, drv->cs_pin, GPIO_PIN_RESET);
    HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(drv->hspi, (uint8_t*)&txData, (uint8_t*)&rxData, 1, 10);
    HAL_GPIO_WritePin(drv->cs_port, drv->cs_pin, GPIO_PIN_SET);

    return status;
}

/**
  * @brief  Set Current Sense Amplifier (CSA) Gain
  */
HAL_StatusTypeDef DRV8353_SetCSAGain(DRV8353_t *drv, DRV8353_CSA_Gain_t gain)
{
    uint16_t regVal = 0;
    HAL_StatusTypeDef status = DRV8353_ReadRegister(drv, DRV8353_REG_CSA_CONTROL, &regVal);
    if (status != HAL_OK) return status;

    // Clear GAIN bits [7:6] and set new gain
    regVal &= ~(0x03 << 6);
    regVal |= ((gain & 0x03) << 6);
    drv->csa_gain = gain;

    return DRV8353_WriteRegister(drv, DRV8353_REG_CSA_CONTROL, regVal);
}

/**
  * @brief  Read Fault Status Registers 1 & 2
  */
HAL_StatusTypeDef DRV8353_ReadFaults(DRV8353_t *drv, uint16_t *fault1, uint16_t *fault2)
{
    HAL_StatusTypeDef status1 = DRV8353_ReadRegister(drv, DRV8353_REG_FAULT_STATUS_1, fault1);
    HAL_StatusTypeDef status2 = DRV8353_ReadRegister(drv, DRV8353_REG_VGS_STATUS_2, fault2);

    if (status1 == HAL_OK && status2 == HAL_OK) {
        return HAL_OK;
    }
    return HAL_ERROR;
}
