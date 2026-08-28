#ifndef ENCODER_CAL_STORE_H
#define ENCODER_CAL_STORE_H

#include "as5048a.h"
#include <stdbool.h>
#include <stdint.h>

bool EncoderCalStore_Load(AS5048A_t *encoder);
bool EncoderCalStore_Save(const int16_t lut[AS5048A_LUT_SIZE]);

#endif /* ENCODER_CAL_STORE_H */
