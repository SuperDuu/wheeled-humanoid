/**
  ******************************************************************************
  * @file           : as5048a.h
  * @brief          : Header for AS5048A Magnetic SPI Encoder Library
  ******************************************************************************
  */

#ifndef __AS5048A_H__
#define __AS5048A_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32g4xx_hal.h"
#include <math.h>

/* AS5048A Commands (Read Bit = 1, Parity calculated dynamically) */
#define AS5048A_CMD_NOP         0x0000
#define AS5048A_CMD_CLEAR_ERROR 0x0001
#define AS5048A_CMD_PROGRAMMING 0x0003
#define AS5048A_CMD_OTP_ZERO    0x0016
#define AS5048A_CMD_AGC_DIAG    0x3FFE
#define AS5048A_CMD_ANGLE       0x3FFF

#define AS5048A_CPR            16384
#define AS5048A_LUT_SIZE       128
#define AS5048A_N_POS_SAMPLES  128

/* AS5048A Structure with Ben Katz Integer-Count Differencing & Linearization */
typedef struct {
    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef      *cs_port;
    uint16_t           cs_pin;
    uint16_t           raw_angle;
    int32_t            count;
    int16_t            offset_lut[AS5048A_LUT_SIZE];
    uint8_t            use_lut;
    float              angle_rad;
    float              angle_deg;
    float              angle_singleturn;
    float              angle_multiturn;
    float              old_angle;
    int32_t            turns;
    uint8_t            first_sample;
    int32_t            count_buff[AS5048A_N_POS_SAMPLES];
    float              velocity_rad_s;
    float              velocity_rpm;
    uint8_t            error_flag;
    uint16_t           consecutive_errors;
    uint32_t           error_count;
} AS5048A_t;

/* Function Prototypes */
HAL_StatusTypeDef AS5048A_Init(AS5048A_t *enc, SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port, uint16_t cs_pin);
HAL_StatusTypeDef AS5048A_ClearError(AS5048A_t *enc);
HAL_StatusTypeDef AS5048A_ReadRawAngle(AS5048A_t *enc, uint16_t *raw_angle);
HAL_StatusTypeDef AS5048A_Sample(AS5048A_t *enc, float dt);
HAL_StatusTypeDef AS5048A_ReadRadians(AS5048A_t *enc, float *angle_rad);
HAL_StatusTypeDef AS5048A_ReadDegrees(AS5048A_t *enc, float *angle_deg);
HAL_StatusTypeDef AS5048A_ReadDiagnostics(AS5048A_t *enc, uint8_t *agc, uint8_t *diag);
uint16_t          AS5048A_CalculateEvenParity(uint16_t value);

#ifdef __cplusplus
}
#endif

#endif /* __AS5048A_H__ */
