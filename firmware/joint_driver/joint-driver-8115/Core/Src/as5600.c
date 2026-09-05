/**
  ******************************************************************************
  * @file           : as5600.c
  * @brief          : Source file for AS5600 12-bit Magnetic I2C Output Link Encoder
  *                   with Dual-Encoder Absolute Vernier Fusion
  * @author         : Vu Duc Du
  ******************************************************************************
  */

#include "as5600.h"
#include <math.h>

#ifndef M_PI_F
#define M_PI_F 3.14159265358979323846f
#endif

#define TWO_PI_F (2.0f * M_PI_F)

AS5600_t g_as5600;

HAL_StatusTypeDef AS5600_Init(AS5600_t *enc, I2C_HandleTypeDef *hi2c)
{
    if (enc == NULL || hi2c == NULL) return HAL_ERROR;

    enc->hi2c = hi2c;
    enc->raw_12bit = 0;
    enc->angle_deg = 0.0f;
    enc->angle_rad = 0.0f;
    enc->status = 0;
    enc->connected = false;
    enc->magnet_detected = false;
    enc->error_count = 0;
    enc->read_count = 0;

    /* Check if AS5600 responds on I2C bus (address 0x36) */
    HAL_StatusTypeDef ret = HAL_I2C_IsDeviceReady(enc->hi2c, AS5600_I2C_ADDR_8BIT, 2, 5);
    if (ret == HAL_OK) {
        enc->connected = true;
        /* Read status register to verify magnet */
        uint8_t stat = 0;
        if (HAL_I2C_Mem_Read(enc->hi2c, AS5600_I2C_ADDR_8BIT, AS5600_REG_STATUS,
                             I2C_MEMADD_SIZE_8BIT, &stat, 1, 5) == HAL_OK) {
            enc->status = stat;
            enc->magnet_detected = (stat & AS5600_STATUS_MD) ? true : false;
        }
        /* Initial angle sample */
        AS5600_ReadAngle(enc);
    } else {
        enc->connected = false;
    }

    return ret;
}

HAL_StatusTypeDef AS5600_ReadAngle(AS5600_t *enc)
{
    if (enc == NULL || enc->hi2c == NULL) return HAL_ERROR;

    uint8_t buf[2] = {0, 0};
    /* Read RAW ANGLE registers (0x0C = MSB, 0x0D = LSB) */
    HAL_StatusTypeDef ret = HAL_I2C_Mem_Read(enc->hi2c, AS5600_I2C_ADDR_8BIT,
                                            AS5600_REG_RAW_ANGLE_MSB,
                                            I2C_MEMADD_SIZE_8BIT, buf, 2, 3);
    if (ret == HAL_OK) {
        uint16_t raw = (((uint16_t)(buf[0] & 0x0F)) << 8) | (uint16_t)buf[1];
        enc->raw_12bit = raw;
        enc->angle_deg = ((float)raw * 360.0f) / 4096.0f;
        enc->angle_rad = ((float)raw * TWO_PI_F) / 4096.0f;
        enc->read_count++;
        enc->connected = true;
    } else {
        enc->error_count++;
    }

    return ret;
}

float DualEncoder_Fuse(float link_angle_rad, float rotor_angle_rad, float gear_ratio)
{
    if (gear_ratio <= 1.05f) {
        /* Direct drive / bare motor: return rotor angle directly */
        return rotor_angle_rad;
    }

    /* Wrap input angles into [0, 2*PI) */
    while (link_angle_rad < 0.0f) link_angle_rad += TWO_PI_F;
    while (link_angle_rad >= TWO_PI_F) link_angle_rad -= TWO_PI_F;

    while (rotor_angle_rad < 0.0f) rotor_angle_rad += TWO_PI_F;
    while (rotor_angle_rad >= TWO_PI_F) rotor_angle_rad -= TWO_PI_F;

    /*
     * Vernier Turn Resolution:
     * theta_rotor_total = k * 2*PI + theta_rotor_single
     * theta_link_expected = theta_rotor_total / N
     * k * 2*PI = N * theta_link - theta_rotor_single
     */
    float k_float = (gear_ratio * link_angle_rad - rotor_angle_rad) / TWO_PI_F;
    int k = (int)roundf(k_float);

    int n_int = (int)roundf(gear_ratio);
    if (n_int > 0) {
        k = (k % n_int + n_int) % n_int;
    }

    float total_rotor_rad = ((float)k * TWO_PI_F) + rotor_angle_rad;
    float fused_joint_rad = total_rotor_rad / gear_ratio;

    return fused_joint_rad;
}
