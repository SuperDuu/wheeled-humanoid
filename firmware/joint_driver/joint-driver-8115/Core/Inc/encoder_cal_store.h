#ifndef ENCODER_CAL_STORE_H
#define ENCODER_CAL_STORE_H

#include "as5048a.h"
#include <stdbool.h>
#include <stdint.h>

bool EncoderCalStore_Load(AS5048A_t *encoder, float *zero_electric_angle, int8_t *encoder_dir, bool *is_aligned);
bool EncoderCalStore_Save(const int16_t lut[AS5048A_LUT_SIZE], float zero_electric_angle, int8_t encoder_dir);
bool EncoderCalStore_SaveAlignment(float zero_electric_angle, int8_t encoder_dir);

#endif /* ENCODER_CAL_STORE_H */
