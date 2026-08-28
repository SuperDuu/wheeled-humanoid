/*
	Copyright 2016 - 2022 Benjamin Vedder	benjamin@vedder.se
	Adapted for STM32G4 HAL Joint Driver - Cycloidal Actuator Project

	This file is part of the VESC firmware.
	GNU General Public License v3. See <http://www.gnu.org/licenses/>.
 */

#ifndef FOC_MATH_H_
#define FOC_MATH_H_

#include "vesc_datatypes.h"
#include "vesc_conf.h"

/* Structure to hold all motor controller state & pointers */
typedef struct {
	mc_configuration *m_conf;
	mc_state          m_state;
	mc_control_mode   m_control_mode;
	motor_state_t     m_motor_state;

	float m_currents_adc[3];
	float m_duty_cycle_set;
	float m_id_set;
	float m_iq_set;
	float m_i_fw_set;
	float m_pos_pid_set;
	float m_speed_pid_set_rpm;
	float m_speed_command_rpm;

	// Encoder & Observer Phase Angles (Radians)
	float m_phase_now_encoder;
	float m_phase_now_observer;
	observer_state m_observer_state;

	// Speed Estimation
	float m_pll_phase;
	float m_pll_speed;          // rad/s electrical
	float m_speed_est_fast;     // rad/s electrical

	// Multi-turn Cycloidal Joint Accumulator with Absolute Home Calibration
	float m_mech_home_offset;    // Calibrated Absolute Mechanical Home Angle [0, 2PI)
	bool  m_home_calibrated;
	float m_mech_angle_single;   // [0, 2PI)
	float m_prev_mech_angle;
	int32_t m_turn_count;
	float m_total_mech_angle;   // Total motor angle in Radians from Home
	float m_joint_angle;        // Output Joint Angle in Radians (after 1:17 gearbox)
	float m_joint_velocity_rad_s;

	// Position & Speed PID Integrators & Filters
	float m_pos_pid_now;
	float m_pos_i_term;
	float m_pos_prev_error;
	float m_pos_prev_proc;
	float m_pos_dt_int;
	float m_pos_dt_int_proc;
	float m_pos_d_filter;
	float m_pos_d_filter_proc;

	float m_speed_i_term;
	float m_speed_prev_error;
	float m_speed_d_filter;
	float m_speed_d_filter_proc;

	// Pre-calculated variables
	float p_lq;
	float p_ld;
	float p_duty_norm;
	float p_fs;
	float p_dt;

	// Smooth S-Curve Trajectory Profile Generator for Robot Arm Joints
	float m_traj_start_angle;
	float m_traj_target_angle;
	float m_traj_duration;              // in seconds
	float m_traj_time;                  // elapsed time in seconds
	float m_pos_holding_current_limit;  // Maximum holding current in Amperes (e.g. 3.0A, 5.0A)
	bool  m_traj_active;

	// Open-to-Closed Spinup Handover (Ben Katz & VESC standard for 1:17 Cycloid stiction)
	bool  m_openloop_spinup_active;
	float m_openloop_spinup_time;

	// Sensor Flag
	bool m_using_encoder;
} motor_all_state_t;

/* Core VESC FOC Functions */
void foc_observer_update(float v_alpha, float v_beta, float i_alpha, float i_beta,
		float dt, observer_state *state, float *phase, motor_all_state_t *motor);
void foc_pll_run(float phase, float dt, float *phase_var,
		float *speed_var, mc_configuration *conf);
void foc_svm(float alpha, float beta, float max_mod, uint32_t PWMFullDutyCycle,
		uint32_t* tAout, uint32_t* tBout, uint32_t* tCout, uint32_t *svm_sector);
void foc_run_pid_control_pos(bool index_found, float dt, motor_all_state_t *motor);
void foc_run_pid_control_speed(bool index_found, float dt, motor_all_state_t *motor);
void foc_start_trajectory(motor_all_state_t *motor, float target_angle_rad, float duration_s, float max_current_a);
void foc_set_home_position(motor_all_state_t *motor);
float foc_correct_encoder(float obs_angle, float enc_angle, float speed, float sl_erpm, motor_all_state_t *motor);
void foc_run_fw(motor_all_state_t *motor, float dt);
void foc_precalc_values(motor_all_state_t *motor);
void foc_update_cycloidal_joint_angle(motor_all_state_t *motor, float raw_mech_angle_rad);

#endif /* FOC_MATH_H_ */
