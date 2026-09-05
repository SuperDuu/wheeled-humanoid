/**
  ******************************************************************************
  * @file           : as5600.h
  * @brief          : Header for AS5600 12-bit Magnetic I2C Output Link Encoder
  *                   Supports Dual-Encoder Absolute Vernier Fusion with AS5048A
  * @author         : Vu Duc Du
  ******************************************************************************
  */

#ifndef __AS5600_H
#define __AS5600_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdbool.h>

#define AS5600_I2C_ADDR_7BIT         0x36
#define AS5600_I2C_ADDR_8BIT         (AS5600_I2C_ADDR_7BIT << 1)

#define AS5600_REG_STATUS            0x0B
#define AS5600_REG_RAW_ANGLE_MSB     0x0C
#define AS5600_REG_RAW_ANGLE_LSB     0x0D
#define AS5600_REG_ANGLE_MSB         0x0E
#define AS5600_REG_ANGLE_LSB         0x0F

#define AS5600_STATUS_MH             (1 << 3) /* Magnet too strong */
#define AS5600_STATUS_ML             (1 << 4) /* Magnet too weak */
#define AS5600_STATUS_MD             (1 << 5) /* Magnet detected */

typedef struct {
    I2C_HandleTypeDef *hi2c;
    uint16_t raw_12bit;
    float    angle_deg;
    float    angle_rad;
    uint8_t  status;
    bool     connected;
    bool     magnet_detected;
    uint32_t error_count;
    uint32_t read_count;
} AS5600_t;

extern AS5600_t g_as5600;

/**
  * @brief  Initialize AS5600 I2C output encoder
  */
HAL_StatusTypeDef AS5600_Init(AS5600_t *enc, I2C_HandleTypeDef *hi2c);

/**
  * @brief  Read raw 12-bit angle from AS5600 (0..4095)
  */
HAL_StatusTypeDef AS5600_ReadAngle(AS5600_t *enc);

/**
  * @brief  Dual-Encoder Absolute Vernier Fusion:
  *         Resolves multi-turn ambiguity of the 1:N rotor encoder (AS5048A)
  *         using the single-turn output link encoder (AS5600).
  * @param  link_angle_rad: Single-turn output angle from AS5600 [0, 2*PI)
  * @param  rotor_angle_rad: Single-turn rotor angle from AS5048A [0, 2*PI)
  * @param  gear_ratio: Transmission reduction ratio (e.g. 17.0f)
  * @return High-resolution single-turn joint angle in radians
  */
float DualEncoder_Fuse(float link_angle_rad, float rotor_angle_rad, float gear_ratio);

#ifdef __cplusplus
}
#endif

#endif /* __AS5600_H */
