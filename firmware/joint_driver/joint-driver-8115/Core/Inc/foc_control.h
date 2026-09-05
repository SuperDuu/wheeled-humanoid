/*
	Author: Vu Duc Du
	Adapted for STM32G4 HAL Joint Driver - Cycloidal Actuator Project

	This file is part of the VESC firmware.
	GNU General Public License v3. See <http://www.gnu.org/licenses/>.
 */

#ifndef FOC_CONTROL_H_
#define FOC_CONTROL_H_

#include "foc_math.h"
#include "drv8353.h"
#include "as5048a.h"

#define ENCODER_LUT_SIZE 128

/* High-Speed FOC ISR Controller Handle */
typedef struct {
	motor_all_state_t motor;
	mc_configuration  conf;
	mc_fault_code     fault;

	// Hardware Pointers
	SPI_HandleTypeDef *hspi_drv;
	SPI_HandleTypeDef *hspi_enc;
	DRV8353_t          drv8353;
	AS5048A_t          encoder;

	// Measured Raw ADC Offsets
	float offset_ia;
	float offset_ib;
	bool  calibrated_offsets;

	// Encoder Zero Alignment
	float zero_electric_angle;
	bool  aligned;
	bool  observer_angle_active;
	float observer_phase_interp;
	float observer_angle_blend;

	// Phase Mapping: Auto-detected during alignment
	// If true, swap Phase B ↔ C at PWM output and current sensing
	// This compensates for PCB trace routing where SOB/SOC or PWM_B/PWM_C are swapped
	bool  phase_swap_bc;

	// Encoder Non-Linearity Calibration Lookup Table (Ben Katz / MIT Cheetah method)
	bool    use_encoder_lut;
	int16_t encoder_lut[ENCODER_LUT_SIZE]; // in units of 0.0001 rad (0.1 mrad)

	// Anti-Cogging Harmonic Current Compensation (6th and 12th Electrical Harmonics)
	bool  anticog_enabled;
	float anticog_amp_6th;    // Amplitude in Amperes (e.g. 0.05f to 0.25f)
	float anticog_phase_6th;  // Phase offset in Radians
	float anticog_amp_12th;   // Amplitude in Amperes
	float anticog_phase_12th; // Phase offset in Radians

	// Output Duty Cycles (0.0 to 1.0)
	float duty_a, duty_b, duty_c;
} FOC_Controller_t;

/* Global Controller Instance */
extern FOC_Controller_t g_foc_controller;

/* Function Prototypes */
void FOC_Control_Init(FOC_Controller_t *foc, SPI_HandleTypeDef *hspi1_drv, SPI_HandleTypeDef *hspi3_enc);
void FOC_Control_AdcCalibrate(FOC_Controller_t *foc, uint16_t raw_adc_a, uint16_t raw_adc_b);
void FOC_Control_AlignEncoder(FOC_Controller_t *foc);
float FOC_Control_CorrectEncoderAngle(const FOC_Controller_t *foc, float raw_rad);
void FOC_Control_Current_ISR(FOC_Controller_t *foc, float current_a, float current_b, float vbus, float temp_fet, float dt);
void FOC_Control_SlowLoop(FOC_Controller_t *foc, float dt);
bool FOC_Control_CheckSafety(FOC_Controller_t *foc, float current_a, float current_b, float vbus, float temp_fet);

#endif /* FOC_CONTROL_H_ */
