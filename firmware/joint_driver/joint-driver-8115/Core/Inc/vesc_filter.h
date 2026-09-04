/*
	Author: Vu Duc Du
	Adapted for STM32G4 HAL Joint Driver - Cycloidal Actuator Project

	This file is part of the VESC firmware.
	GNU General Public License v3. See <http://www.gnu.org/licenses/>.
 */

#ifndef VESC_FILTER_H_
#define VESC_FILTER_H_

#include <stdint.h>

typedef struct {
	float a0, a1, a2, b1, b2;
	float z1, z2;
} Biquad;

typedef enum {
	BQ_LOWPASS,
	BQ_HIGHPASS
} BiquadType;

// Functions
float biquad_process(Biquad *biquad, float in);
void biquad_config(Biquad *biquad, BiquadType type, float Fc);
void biquad_reset(Biquad *biquad);

#endif /* VESC_FILTER_H_ */
