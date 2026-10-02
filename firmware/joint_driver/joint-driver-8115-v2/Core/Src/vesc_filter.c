/*
	Author: Vu Duc Du
	Adapted for STM32G4 HAL Joint Driver - Cycloidal Actuator Project

	This file is part of the VESC firmware.
	GNU General Public License v3. See <http://www.gnu.org/licenses/>.
 */

#include "vesc_filter.h"
#include <math.h>

float biquad_process(Biquad *biquad, float in) {
    float out = in * biquad->a0 + biquad->z1;
    biquad->z1 = in * biquad->a1 + biquad->z2 - biquad->b1 * out;
    biquad->z2 = in * biquad->a2 - biquad->b2 * out;
    return out;
}

void biquad_config(Biquad *biquad, BiquadType type, float Fc) {
	float K = tanf((float)M_PI * Fc);
	float Q = 0.707f; // maximum sharpness (0.5 = maximum smoothness)
	float norm = 1.0f / (1.0f + K / Q + K * K);
	if (type == BQ_LOWPASS) {
		biquad->a0 = K * K * norm;
		biquad->a1 = 2.0f * biquad->a0;
		biquad->a2 = biquad->a0;
	}
	else if (type == BQ_HIGHPASS) {
		biquad->a0 = 1.0f * norm;
		biquad->a1 = -2.0f * biquad->a0;
		biquad->a2 = biquad->a0;
	}
	biquad->b1 = 2.0f * (K * K - 1.0f) * norm;
	biquad->b2 = (1.0f - K / Q + K * K) * norm;
}

void biquad_reset(Biquad *biquad) {
	biquad->z1 = 0.0f;
	biquad->z2 = 0.0f;
}
