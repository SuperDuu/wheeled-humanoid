/*
	Author: Vu Duc Du
	Adapted for STM32G4 HAL Joint Driver - Cycloidal Actuator Project

	This file is part of the VESC firmware.
	GNU General Public License v3. See <http://www.gnu.org/licenses/>.
 */

#ifndef VESC_DATATYPES_H_
#define VESC_DATATYPES_H_

#include <stdbool.h>
#include <stdint.h>

/* Motor & Driver Operating State */
typedef enum {
	MC_STATE_OFF = 0,
	MC_STATE_DETECTING,
	MC_STATE_RUNNING,
	MC_STATE_FULL_BRAKE
} mc_state;

/* Motor Control Mode */
typedef enum {
	CONTROL_MODE_DUTY = 0,
	CONTROL_MODE_POWER,
	CONTROL_MODE_CURRENT,
	CONTROL_MODE_CURRENT_BRAKE,
	CONTROL_MODE_SPEED,
	CONTROL_MODE_POS,
	CONTROL_MODE_HANDBRAKE,
	CONTROL_MODE_OPENLOOP
} mc_control_mode;

/* FOC Sensor Mode */
typedef enum {
	FOC_SENSOR_MODE_SENSORLESS = 0,
	FOC_SENSOR_MODE_ENCODER,
	FOC_SENSOR_MODE_HALL,
	FOC_SENSOR_MODE_HFI
} mc_foc_sensor_mode;

/* FOC Observer Type */
typedef enum {
	FOC_OBSERVER_ORTEGA_ORIGINAL = 0,
	FOC_OBSERVER_MXLEMMING,
	FOC_OBSERVER_ORTEGA_LAMBDA_COMP,
	FOC_OBSERVER_MXV
} foc_observer_type;

/* Cross-coupling Decoupling Mode */
typedef enum {
	FOC_CC_DECOUPLING_DISABLED = 0,
	FOC_CC_DECOUPLING_CROSS,
	FOC_CC_DECOUPLING_BEMF,
	FOC_CC_DECOUPLING_CROSS_BEMF
} foc_cc_decoupling_mode;

/* PWM Switching Mode */
typedef enum {
	FOC_PWM_DISABLED = 0,
	FOC_PWM_ENABLED,
	FOC_PWM_FULL_BRAKE
} foc_pwm_mode;

/* Safety Fault Code Bitmask */
typedef enum {
	MC_FAULT_NONE           = 0x00,
	MC_FAULT_OVER_CURRENT   = 0x01,
	MC_FAULT_OVER_VOLTAGE   = 0x02,
	MC_FAULT_UNDER_VOLTAGE  = 0x04,
	MC_FAULT_OVER_TEMP_MOS  = 0x08,
	MC_FAULT_OVER_TEMP_MOT  = 0x10,
	MC_FAULT_UNBALANCED     = 0x20,
	MC_FAULT_ENCODER        = 0x40,
	MC_FAULT_POS_LIMIT      = 0x80
} mc_fault_code;

/* Observer State Structure */
typedef struct {
	float x1;
	float x2;
	float lambda_est;
	float i_alpha_last;
	float i_beta_last;
} observer_state;

/* VESC Motor State Structure (Core FOC Math State) */
typedef struct {
	float va;
	float vb;
	float vc;
	float v_mag_filter;
	float mod_alpha_filter;
	float mod_beta_filter;
	float mod_alpha_measured;
	float mod_beta_measured;
	float mod_alpha_raw;
	float mod_beta_raw;
	float id_target;
	float iq_target;
	float max_duty;
	float duty_now;
	float phase;
	float phase_cos;
	float phase_sin;
	float i_alpha;
	float i_beta;
	float i_abs;
	float i_abs_filter;
	float i_bus;
	float v_bus;
	float v_alpha;
	float v_beta;
	float mod_d;
	float mod_q;
	float mod_q_filter;
	float id;
	float iq;
	float id_filter;
	float iq_filter;
	float vd;
	float vq;
	float vd_int;
	float vq_int;
	uint32_t svm_sector;
} motor_state_t;

#endif /* VESC_DATATYPES_H_ */
