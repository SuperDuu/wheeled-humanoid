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

HAL_StatusTypeDef AS5048A_ClearError(AS5048A_t *enc)
{
    if (enc == NULL || enc->hspi == NULL) return HAL_ERROR;
    uint16_t command = AS5048A_CalculateEvenParity((1 << 14) | AS5048A_CMD_CLEAR_ERROR);
    uint16_t response = 0;
    HAL_GPIO_WritePin(enc->cs_port, enc->cs_pin, GPIO_PIN_RESET);
    for (volatile int i = 0; i < 6; i++) { __NOP(); }
    HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(enc->hspi, (uint8_t*)&command, (uint8_t*)&response, 1, 2);
    for (volatile int i = 0; i < 3; i++) { __NOP(); }
    HAL_GPIO_WritePin(enc->cs_port, enc->cs_pin, GPIO_PIN_SET);
    return status;
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
    enc->count = 0;
    enc->use_lut = 0;
    enc->angle_rad = 0.0f;
    enc->angle_deg = 0.0f;
    enc->angle_singleturn = 0.0f;
    enc->angle_multiturn = 0.0f;
    enc->old_angle = 0.0f;
    enc->turns = 0;
    enc->first_sample = 0;
    enc->velocity_rad_s = 0.0f;
    enc->velocity_rpm = 0.0f;
    enc->error_flag = 0;
    enc->consecutive_errors = 0;
    enc->error_count = 0;
    for (int i = 0; i < AS5048A_N_POS_SAMPLES; i++) {
        enc->count_buff[i] = 0;
    }
    for (int i = 0; i < AS5048A_LUT_SIZE; i++) {
        enc->offset_lut[i] = 0;
    }

    // Set CS high initially
    HAL_GPIO_WritePin(enc->cs_port, enc->cs_pin, GPIO_PIN_SET);

    // Clear any power-on error flag
    AS5048A_ClearError(enc);
    AS5048A_ClearError(enc);

    /* Prime the AS5048A pipelined response. A response belongs to the previous
     * command, so two angle commands are required after CLEAR_ERROR. */
    uint16_t primed_angle = 0;
    AS5048A_ReadRawAngle(enc, &primed_angle);
    AS5048A_ReadRawAngle(enc, &primed_angle);

    // Initial tracked sample
    return AS5048A_Sample(enc, 0.000050f);
}

/**
  * @brief  Sample AS5048A Position & Velocity via Ben Katz Integer-Count Differencing
  */
HAL_StatusTypeDef AS5048A_Sample(AS5048A_t *enc, float dt)
{
    if (enc == NULL || enc->hspi == NULL) return HAL_ERROR;

    // 1. Shift around previous samples
    enc->old_angle = enc->angle_singleturn;
    for (int i = AS5048A_N_POS_SAMPLES - 1; i > 0; i--) {
        enc->count_buff[i] = enc->count_buff[i - 1];
    }

    // 2. SPI Read Raw Angle
    uint16_t previous_raw = enc->raw_angle;
    uint16_t raw = 0;
    HAL_StatusTypeDef status = AS5048A_ReadRawAngle(enc, &raw);

    /* At the 10 kHz closed-loop sample rate, even 2000 RPM advances less than
     * 55 encoder counts per sample. Reject isolated EMI frames before they can
     * corrupt electrical angle, velocity, and the speed PI. */
    if (status == HAL_OK && enc->first_sample && dt <= 0.0002f) {
        int32_t raw_step = (int32_t)enc->raw_angle - (int32_t)previous_raw;
        if (raw_step > (AS5048A_CPR / 2)) raw_step -= AS5048A_CPR;
        if (raw_step < -(AS5048A_CPR / 2)) raw_step += AS5048A_CPR;
        if (raw_step > 96 || raw_step < -96) {
            enc->raw_angle = previous_raw;
            enc->error_flag = 1;
            status = HAL_ERROR;
        }
    }

    if (status == HAL_OK) {
        enc->consecutive_errors = 0;
    } else {
        if (enc->consecutive_errors < UINT16_MAX) {
            enc->consecutive_errors++;
        }
        enc->error_count++;
        if (enc->consecutive_errors >= 2 && (enc->consecutive_errors % 5 == 0)) {
            AS5048A_ClearError(enc);
        }
    }

    // 3. Linearization (128-point LUT bit-shift interpolation, AS5048A 14-bit: 16384 >> 7 = 128)
    int32_t off_interp = 0;
    if (enc->use_lut) {
        int idx = (enc->raw_angle >> 7) & 0x7F; // 0..127
        int off_1 = enc->offset_lut[idx];
        int off_2 = enc->offset_lut[(idx + 1) & 0x7F];
        int frac = enc->raw_angle - (idx << 7); // 0..127
        off_interp = off_1 + (((off_2 - off_1) * frac) >> 7);
    }
    enc->count = (int32_t)enc->raw_angle + off_interp;

    // 4. Single-turn count & angle [0, 2*PI)
    int32_t count_wrapped = enc->count % AS5048A_CPR;
    if (count_wrapped < 0) { count_wrapped += AS5048A_CPR; }
    enc->angle_singleturn = (2.0f * (float)M_PI) * ((float)count_wrapped) / (float)AS5048A_CPR;
    enc->angle_rad = enc->angle_singleturn;
    enc->angle_deg = enc->angle_singleturn * (180.0f / (float)M_PI);

    // 5. Shortest-path single-turn angular displacement
    float d_angle = enc->angle_singleturn - enc->old_angle;
    if (d_angle > (float)M_PI) {
        d_angle -= 2.0f * (float)M_PI;
    } else if (d_angle < -(float)M_PI) {
        d_angle += 2.0f * (float)M_PI;
    }

    int first_sample = !enc->first_sample;
    if (first_sample) {
        enc->turns = 0;
        enc->first_sample = 1;
        enc->velocity_rad_s = 0.0f;
        enc->velocity_rpm = 0.0f;
    } else {
        // Multi-turn tracking
        if (enc->angle_singleturn < enc->old_angle - (float)M_PI) {
            enc->turns++;
        } else if (enc->angle_singleturn > enc->old_angle + (float)M_PI) {
            enc->turns--;
        }

        // 6. Robust Instantaneous Velocity with Glitch Rejection
        if (dt > 0.000001f) {
            float raw_vel = d_angle / dt;
            /* Reject glitch spikes (> 120 rad/s = > 1150 RPM) */
            if (raw_vel > 120.0f || raw_vel < -120.0f) {
                raw_vel = enc->velocity_rad_s;
            }
            /* Smooth 1st-order filter (~30 Hz cutoff at 10kHz sample rate) */
            enc->velocity_rad_s += 0.025f * (raw_vel - enc->velocity_rad_s);
            if (fabsf(enc->velocity_rad_s) < 0.08f && fabsf(d_angle) < 0.0001f) {
                enc->velocity_rad_s = 0.0f;
            }
            enc->velocity_rpm = enc->velocity_rad_s * (60.0f / (2.0f * (float)M_PI));
        }
    }

    // 7. Multi-turn position
    enc->count_buff[0] = count_wrapped + (AS5048A_CPR * enc->turns);
    enc->angle_multiturn = (2.0f * (float)M_PI) * ((float)enc->count_buff[0]) / (float)AS5048A_CPR;

    return status;
}

/**
  * @brief  Read 14-bit Raw Angle from AS5048A
  */
HAL_StatusTypeDef AS5048A_ReadRawAngle(AS5048A_t *enc, uint16_t *raw_angle)
{
    if (enc == NULL || raw_angle == NULL || enc->hspi == NULL) return HAL_ERROR;

    uint16_t command = AS5048A_CalculateEvenParity((1 << 14) | AS5048A_CMD_ANGLE);
    uint16_t response = 0;

    // Assert CS (Active LOW)
    HAL_GPIO_WritePin(enc->cs_port, enc->cs_pin, GPIO_PIN_RESET);
    for (volatile int i = 0; i < 6; i++) { __NOP(); } // CS setup time > 350ns

    // Perform 16-bit SPI transfer (takes ~3.0µs at 5.3Mbps SPI clock)
    HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(enc->hspi, (uint8_t*)&command, (uint8_t*)&response, 1, 2);

    for (volatile int i = 0; i < 3; i++) { __NOP(); } // CS hold time > 50ns
    // Deassert CS (HIGH)
    HAL_GPIO_WritePin(enc->cs_port, enc->cs_pin, GPIO_PIN_SET);

    if (status == HAL_OK) {
        // Verify Even Parity on 16-bit frame
        uint16_t expected_parity_frame = AS5048A_CalculateEvenParity(response & 0x7FFF);
        if (response == expected_parity_frame && (response & (1 << 14)) == 0) {
            // Valid frame: Extract 14-bit absolute angle (bits 13:0)
            uint16_t angleData = response & 0x3FFF;
            enc->raw_angle = angleData;
            enc->angle_rad = ((float)angleData / 16384.0f) * (2.0f * (float)M_PI);
            enc->angle_deg = ((float)angleData / 16384.0f) * 360.0f;
            enc->error_flag = 0;
        } else {
            // Parity/sensor error: retain the last valid angle.
            enc->error_flag = 1;
            status = HAL_ERROR;
        }
    } else {
        // Clear SPI state on error to prevent latchup
        enc->hspi->State = HAL_SPI_STATE_READY;
        __HAL_SPI_CLEAR_OVRFLAG(enc->hspi);
        enc->error_flag = 1;
    }

    *raw_angle = enc->raw_angle;
    return status;
}

/**
  * @brief  Read Angle in Radians
  */
HAL_StatusTypeDef AS5048A_ReadRadians(AS5048A_t *enc, float *angle_rad)
{
    uint16_t raw;
    HAL_StatusTypeDef status = AS5048A_ReadRawAngle(enc, &raw);
    if (enc != NULL && angle_rad != NULL) {
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
    if (enc != NULL && angle_deg != NULL) {
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
