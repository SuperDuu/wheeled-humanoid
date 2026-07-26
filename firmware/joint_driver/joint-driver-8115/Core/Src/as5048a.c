/**
  ******************************************************************************
  * @file           : as5048a.c
  * @brief          : Source file for AS5048A Magnetic SPI Encoder Library
  ******************************************************************************
  */

#include "as5048a.h"

#define M_PI_F 3.14159265358979323846f

/**
  * @brief  Calculate Even Parity bit for 16-bit AS5048 frame
  */
uint16_t AS5048A_CalculateEvenParity(uint16_t value)
{
    uint8_t count = 0;
    uint16_t temp = value & 0x7FFF; // 15 bits data
    while (temp) {
        count += temp & 1;
        temp >>= 1;
    }
    // Set MSB (bit 15) so that total 1s is even
    if (count % 2 != 0) {
        value |= (1 << 15);
    } else {
        value &= ~(1 << 15);
    }
    return value;
}

/**
  * @brief  Initialize AS5048A instance
  */
HAL_StatusTypeDef AS5048A_Init(AS5048A_t *enc, SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port, uint16_t cs_pin)
{
    if (enc == NULL || hspi == NULL) return HAL_ERROR;

    enc->hspi = hspi;
    enc->cs_port = cs_port;
    enc->cs_pin = cs_pin;
    enc->raw_angle = 0;
    enc->angle_rad = 0.0f;
    enc->angle_deg = 0.0f;
    enc->error_flag = 0;

    // Set CS high initially
    HAL_GPIO_WritePin(enc->cs_port, enc->cs_pin, GPIO_PIN_SET);

    // Initial dummy read to clear status
    uint16_t dummy;
    return AS5048A_ReadRawAngle(enc, &dummy);
}

/**
  * @brief  Read 14-bit Raw Angle from AS5048A
  */
HAL_StatusTypeDef AS5048A_ReadRawAngle(AS5048A_t *enc, uint16_t *raw_angle)
{
    if (enc == NULL || raw_angle == NULL) return HAL_ERROR;

    // Command: Read Angle Register (0x3FFF) with Read Bit (1 << 14) and Parity
    uint16_t command = AS5048A_CalculateEvenParity((1 << 14) | AS5048A_CMD_ANGLE);
    uint16_t response = 0;

    // SPI Transfer (Transmits command for current frame, receives result of previous frame)
    HAL_GPIO_WritePin(enc->cs_port, enc->cs_pin, GPIO_PIN_RESET);
    HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(enc->hspi, (uint8_t*)&command, (uint8_t*)&response, 1, 10);
    HAL_GPIO_WritePin(enc->cs_port, enc->cs_pin, GPIO_PIN_SET);

    if (status != HAL_OK) return status;

    // Check Error Flag (Bit 14 of response)
    if (response & (1 << 14)) {
        enc->error_flag = 1;
    } else {
        enc->error_flag = 0;
    }

    // Extract 14-bit angle data (Bits 0..13)
    uint16_t angleData = response & 0x3FFF;
    enc->raw_angle = angleData;
    *raw_angle = angleData;

    // Update converted values
    enc->angle_rad = ((float)angleData / 16384.0f) * (2.0f * M_PI_F);
    enc->angle_deg = ((float)angleData / 16384.0f) * 360.0f;

    return HAL_OK;
}

/**
  * @brief  Read Angle in Radians
  */
HAL_StatusTypeDef AS5048A_ReadRadians(AS5048A_t *enc, float *angle_rad)
{
    uint16_t raw;
    HAL_StatusTypeDef status = AS5048A_ReadRawAngle(enc, &raw);
    if (status == HAL_OK && angle_rad != NULL) {
        *angle_rad = enc->angle_rad;
    }
    return status;
}

/**
  * @brief  Read Angle in Degrees
  */
HAL_StatusTypeDef AS5048A_ReadDegrees(AS5048A_t *enc, float *angle_deg)
{
    uint16_t raw;
    HAL_StatusTypeDef status = AS5048A_ReadRawAngle(enc, &raw);
    if (status == HAL_OK && angle_deg != NULL) {
        *angle_deg = enc->angle_deg;
    }
    return status;
}

/**
  * @brief  Read Diagnostics / AGC Status
  */
HAL_StatusTypeDef AS5048A_ReadDiagnostics(AS5048A_t *enc, uint8_t *agc, uint8_t *diag)
{
    if (enc == NULL) return HAL_ERROR;

    uint16_t command = AS5048A_CalculateEvenParity((1 << 14) | AS5048A_CMD_AGC_DIAG);
    uint16_t response = 0;

    HAL_GPIO_WritePin(enc->cs_port, enc->cs_pin, GPIO_PIN_RESET);
    HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(enc->hspi, (uint8_t*)&command, (uint8_t*)&response, 1, 10);
    HAL_GPIO_WritePin(enc->cs_port, enc->cs_pin, GPIO_PIN_SET);

    if (status == HAL_OK) {
        if (agc) *agc = response & 0xFF;         // AGC value (0..255)
        if (diag) *diag = (response >> 8) & 0x0F; // Diagnostic flags
    }
    return status;
}
