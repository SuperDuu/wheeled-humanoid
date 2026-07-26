/*
	Copyright 2016 - 2022 Benjamin Vedder	benjamin@vedder.se
	Adapted for STM32G4 HAL Joint Driver - Cycloidal Actuator Project

	This file is part of the VESC firmware.
	GNU General Public License v3. See <http://www.gnu.org/licenses/>.
 */

#ifndef VESC_CONF_H_
#define VESC_CONF_H_

#include "vesc_datatypes.h"

/* Complete VESC Motor Configuration Structure */
typedef struct {
	// PWM & Switching Configuration
	float foc_f_zv;                    // Switching frequency (20000.0 Hz)
	float l_max_duty;                  // Max duty cycle (0.95)
	float l_min_duty;                  // Min duty cycle (0.005)

	// Motor Electrical Parameters (GB8115-4)
	uint8_t  foc_motor_pole_pairs;     // GB8115-4 has 21 pole pairs
	float    foc_motor_r;              // Phase resistance [Ohm]
	float    foc_motor_l;              // Phase inductance [H]
	float    foc_motor_flux_linkage;   // Flux linkage [Wb]
	float    foc_motor_ld_lq_diff;     // Ld - Lq difference [H]

	// Cycloidal Gearbox & Joint Configuration
	float    gear_ratio;               // Cycloidal Gear Ratio (17.0f = 1:17)
	int8_t   encoder_direction;        // +1 or -1
	float    joint_pos_min;            // Joint soft min limit [-PI rad = -180 deg]
	float    joint_pos_max;            // Joint soft max limit [+PI rad = +180 deg]

	// Current Controller Parameters (PI D/Q)
	float foc_current_kp;              // Kp for current loop
	float foc_current_ki;              // Ki for current loop
	float foc_current_filter_const;    // Filter constant for currents
	foc_cc_decoupling_mode foc_cc_decoupling; // Cross-coupling decoupling mode

	// Speed Controller Parameters (PID)
	float s_pid_kp;                    // Speed Kp
	float s_pid_ki;                    // Speed Ki
	float s_pid_kd;                    // Speed Kd
	float s_pid_kd_filter;             // Speed Kd filter
	float s_pid_min_erpm;              // Minimum ERPM
	float s_pid_ramp_erpms_s;          // Speed ramp rate ERPM/s

	// Position Controller Parameters (PID + Process D)
	float p_pid_kp;                    // Position Kp
	float p_pid_ki;                    // Position Ki
	float p_pid_kd;                    // Position Kd
	float p_pid_kd_proc;               // Position Kd on process variable
	float p_pid_kd_filter;             // Position Kd filter
	float p_pid_ang_div;               // Angle divisor
	float p_pid_gain_dec_angle;        // Gain decrease angle

	// Observer & Sensorless Configuration
	foc_observer_type foc_observer_type; // Observer type
	float foc_observer_gain;           // Observer gain gamma
	float foc_pll_kp;                  // PLL Kp
	float foc_pll_ki;                  // PLL Ki
	float foc_sl_erpm;                 // Sensorless ERPM threshold

	// Field Weakening Parameters
	float foc_fw_current_max;          // Max Field Weakening current [A]
	float foc_fw_duty_start;           // Duty start for FW (0.90)
	float foc_fw_ramp_time;            // FW ramp time [s]
	float foc_fw_backoff;              // FW backoff gain

	// Overmodulation & Voltage Vector Limits
	float foc_overmod_factor;          // Overmodulation factor (1.0..1.15)
	float foc_mag_vd_max;              // Max Vd voltage fraction (0.2)

	// Safety Limits
	float l_current_max;               // Max motor current [A] (e.g., 25.0A)
	float l_current_min;               // Min motor braking current [A] (-25.0A)
	float l_in_current_max;            // Max input battery current [A]
	float l_in_current_min;            // Max regen battery current [A]
	float l_max_erpm;                  // Max allowed ERPM
	float l_min_erpm;                  // Min allowed ERPM
	float l_max_erpm_fbreak;           // ERPM for full brake
	float l_max_erpm_fbreak_cc;        // ERPM for full brake in CC
	float l_voltage_max;               // Max bus voltage [V] (e.g., 50.0V)
	float l_voltage_min;               // Min bus voltage [V] (e.g., 12.0V)
	float l_temp_fet_start;            // MOSFET temp limit start [C]
	float l_temp_fet_end;              // MOSFET temp limit cutoff [C]
	float l_temp_motor_start;          // Motor temp limit start [C]
	float l_temp_motor_end;            // Motor temp limit cutoff [C]

	// Sensor Mode
	mc_foc_sensor_mode foc_sensor_mode;
	bool               foc_encoder_inverted;
} mc_configuration;

/* Default Configuration Function */
void vesc_conf_set_defaults(mc_configuration *conf);

#endif /* VESC_CONF_H_ */
