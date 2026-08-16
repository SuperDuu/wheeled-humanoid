/*
	Copyright 2016 - 2022 Benjamin Vedder	benjamin@vedder.se
	Adapted for STM32G4 HAL Joint Driver - Cycloidal Actuator Project

	This file is part of the VESC firmware.
	GNU General Public License v3. See <http://www.gnu.org/licenses/>.
 */

#include "foc_math.h"
#include "vesc_utils.h"
#include <math.h>

/**
  * @brief  VESC Flux Linkage Observer (Ortega / MxLemming)
  */
void foc_observer_update(float v_alpha, float v_beta, float i_alpha, float i_beta,
		float dt, observer_state *state, float *phase, motor_all_state_t *motor) {

	mc_configuration *conf_now = motor->m_conf;

	float R = conf_now->foc_motor_r;
	float L = conf_now->foc_motor_l;
	float lambda = conf_now->foc_motor_flux_linkage;

	float L_ia = L * i_alpha;
	float L_ib = L * i_beta;
	const float R_ia = R * i_alpha;
	const float R_ib = R * i_beta;
	const float gamma_half = conf_now->foc_observer_gain * 0.5f;

	switch (conf_now->foc_observer_type) {
	case FOC_OBSERVER_ORTEGA_ORIGINAL: {
		float err = SQ(lambda) - (SQ(state->x1 - L_ia) + SQ(state->x2 - L_ib));

		if (err > 0.0f) {
			err = 0.0f;
		}

		float x1_dot = v_alpha - R_ia + gamma_half * (state->x1 - L_ia) * err;
		float x2_dot = v_beta - R_ib + gamma_half * (state->x2 - L_ib) * err;

		state->x1 += x1_dot * dt;
		state->x2 += x2_dot * dt;
	} break;

	case FOC_OBSERVER_MXLEMMING: {
		state->x1 += (v_alpha - R_ia) * dt - L * (i_alpha - state->i_alpha_last);
		state->x2 += (v_beta - R_ib) * dt - L * (i_beta - state->i_beta_last);

		utils_truncate_number_abs(&(state->x1), lambda);
		utils_truncate_number_abs(&(state->x2), lambda);

		L_ia = 0.0f;
		L_ib = 0.0f;
	} break;

	default:
		break;
	}

	state->i_alpha_last = i_alpha;
	state->i_beta_last = i_beta;

	UTILS_NAN_ZERO(state->x1);
	UTILS_NAN_ZERO(state->x2);

	float mag = NORM2_f(state->x1, state->x2);
	if (mag < (lambda * 0.5f)) {
		state->x1 *= 1.1f;
		state->x2 *= 1.1f;
	}

	if (phase) {
		*phase = utils_fast_atan2(state->x2 - L_ib, state->x1 - L_ia);
	}
}

/**
  * @brief  Phase Locked Loop (PLL) for Speed Estimation
  */
void foc_pll_run(float phase, float dt, float *phase_var,
					float *speed_var, mc_configuration *conf) {
	UTILS_NAN_ZERO(*phase_var);
	float delta_theta = phase - *phase_var;
	utils_norm_angle_rad(&delta_theta);
	UTILS_NAN_ZERO(*speed_var);
	*phase_var += (*speed_var + conf->foc_pll_kp * delta_theta) * dt;
	utils_norm_angle_rad((float*)phase_var);
	*speed_var += conf->foc_pll_ki * delta_theta * dt;
}

/**
  * @brief  Space Vector Modulation (SVM) - 6-Sector Algorithm
  */
void foc_svm(float alpha, float beta, float max_mod, uint32_t PWMFullDutyCycle,
				uint32_t* tAout, uint32_t* tBout, uint32_t* tCout, uint32_t *svm_sector) {
	uint32_t sector;

	if (beta >= 0.0f) {
		if (alpha >= 0.0f) {
			if (ONE_BY_SQRT3 * beta > alpha) {
				sector = 2;
			} else {
				sector = 1;
			}
		} else {
			if (-ONE_BY_SQRT3 * beta > alpha) {
				sector = 3;
			} else {
				sector = 2;
			}
		}
	} else {
		if (alpha >= 0.0f) {
			if (-ONE_BY_SQRT3 * beta > alpha) {
				sector = 5;
			} else {
				sector = 6;
			}
		} else {
			if (ONE_BY_SQRT3 * beta > alpha) {
				sector = 4;
			} else {
				sector = 5;
			}
		}
	}

	int tA, tB, tC;

	switch (sector) {
	case 1: {
		int t1 = (alpha - ONE_BY_SQRT3 * beta) * PWMFullDutyCycle;
		int t2 = (TWO_BY_SQRT3 * beta) * PWMFullDutyCycle;
		tA = (PWMFullDutyCycle + t1 + t2) / 2;
		tB = tA - t1;
		tC = tB - t2;
		break;
	}
	case 2: {
		int t2 = (alpha + ONE_BY_SQRT3 * beta) * PWMFullDutyCycle;
		int t3 = (-alpha + ONE_BY_SQRT3 * beta) * PWMFullDutyCycle;
		tB = (PWMFullDutyCycle + t2 + t3) / 2;
		tA = tB - t3;
		tC = tA - t2;
		break;
	}
	case 3: {
		int t3 = (TWO_BY_SQRT3 * beta) * PWMFullDutyCycle;
		int t4 = (-alpha - ONE_BY_SQRT3 * beta) * PWMFullDutyCycle;
		tB = (PWMFullDutyCycle + t3 + t4) / 2;
		tC = tB - t3;
		tA = tC - t4;
		break;
	}
	case 4: {
		int t4 = (-alpha + ONE_BY_SQRT3 * beta) * PWMFullDutyCycle;
		int t5 = (-TWO_BY_SQRT3 * beta) * PWMFullDutyCycle;
		tC = (PWMFullDutyCycle + t4 + t5) / 2;
		tB = tC - t5;
		tA = tB - t4;
		break;
	}
	case 5: {
		int t5 = (-alpha - ONE_BY_SQRT3 * beta) * PWMFullDutyCycle;
		int t6 = (alpha - ONE_BY_SQRT3 * beta) * PWMFullDutyCycle;
		tC = (PWMFullDutyCycle + t5 + t6) / 2;
		tA = tC - t5;
		tB = tA - t6;
		break;
	}
	case 6: {
		int t6 = (-TWO_BY_SQRT3 * beta) * PWMFullDutyCycle;
		int t1 = (alpha + ONE_BY_SQRT3 * beta) * PWMFullDutyCycle;
		tA = (PWMFullDutyCycle + t6 + t1) / 2;
		tC = tA - t1;
		tB = tC - t6;
		break;
	}
	}

	int t_max = PWMFullDutyCycle * (1.0f - (1.0f - max_mod) * 0.5f);
	utils_truncate_number_int(&tA, 0, t_max);
	utils_truncate_number_int(&tB, 0, t_max);
	utils_truncate_number_int(&tC, 0, t_max);

	*tAout = tA;
	*tBout = tB;
	*tCout = tC;
	*svm_sector = sector;
}

/**
  * @brief  VESC Position Controller Loop (PID + Process Derivative + Joint Soft Limits)
  */
void foc_run_pid_control_pos(bool index_found, float dt, motor_all_state_t *motor) {
	mc_configuration *conf_now = motor->m_conf;

	float angle_now = motor->m_joint_angle; // Controlled on Joint Output Angle
	float angle_set = motor->m_pos_pid_set;

	// Clamp target within software joint limits (-PI to +PI rad = -180 to +180 deg)
	utils_truncate_number(&angle_set, conf_now->joint_pos_min, conf_now->joint_pos_max);

	if (motor->m_control_mode != CONTROL_MODE_POS) {
		motor->m_pos_i_term = 0.0f;
		motor->m_pos_prev_error = 0.0f;
		motor->m_pos_prev_proc = angle_now;
		motor->m_pos_d_filter = 0.0f;
		motor->m_pos_d_filter_proc = 0.0f;
		return;
	}

	float error = angle_set - angle_now;

	float kp = conf_now->p_pid_kp;
	float ki = conf_now->p_pid_ki;
	float kd = conf_now->p_pid_kd;
	float kd_proc = conf_now->p_pid_kd_proc;

	float p_term = error * kp;
	motor->m_pos_i_term += error * (ki * dt);

	// D-term calculation
	float d_term = 0.0f;
	motor->m_pos_dt_int += dt;
	if (error != motor->m_pos_prev_error) {
		d_term = (error - motor->m_pos_prev_error) * (kd / motor->m_pos_dt_int);
		motor->m_pos_dt_int = 0.0f;
	}
	UTILS_LP_FAST(motor->m_pos_d_filter, d_term, conf_now->p_pid_kd_filter);
	d_term = motor->m_pos_d_filter;

	// Process D-term (D on measurement to prevent derivative kick)
	float d_term_proc = 0.0f;
	motor->m_pos_dt_int_proc += dt;
	if (angle_now != motor->m_pos_prev_proc) {
		d_term_proc = -(angle_now - motor->m_pos_prev_proc) * (kd_proc / motor->m_pos_dt_int_proc);
		motor->m_pos_dt_int_proc = 0.0f;
	}
	UTILS_LP_FAST(motor->m_pos_d_filter_proc, d_term_proc, conf_now->p_pid_kd_filter);
	d_term_proc = motor->m_pos_d_filter_proc;

	// Anti-windup protection
	float p_tmp = p_term;
	utils_truncate_number_abs(&p_tmp, 1.0f);
	utils_truncate_number_abs((float*)&motor->m_pos_i_term, 1.0f - fabsf(p_tmp));

	motor->m_pos_prev_error = error;
	motor->m_pos_prev_proc = angle_now;

	float output = p_term + motor->m_pos_i_term + d_term + d_term_proc;
	utils_truncate_number(&output, -1.0f, 1.0f);

	// Position PID calculates target Iq current (Amperes)
	motor->m_iq_set = output * conf_now->l_current_max;
}

/**
  * @brief  VESC Speed Controller Loop (PID in Current-Mode FOC)
  */
void foc_run_pid_control_speed(bool index_found, float dt, motor_all_state_t *motor) {
	mc_configuration *conf_now = motor->m_conf;

	if (motor->m_control_mode != CONTROL_MODE_SPEED) {
		motor->m_speed_i_term = 0.0f;
		motor->m_speed_prev_error = 0.0f;
		motor->m_speed_d_filter = 0.0f;
		return;
	}

	if (conf_now->s_pid_ramp_erpms_s > 0.0f) {
		utils_step_towards((float*)&motor->m_speed_pid_set_rpm, motor->m_speed_command_rpm, conf_now->s_pid_ramp_erpms_s * dt);
		utils_truncate_number(&motor->m_speed_pid_set_rpm, conf_now->l_min_erpm, conf_now->l_max_erpm);
	}

	float erpm = RADPS2RPM_f(motor->m_speed_est_fast);
	float target_erpm = (conf_now->s_pid_ramp_erpms_s > 0.0f) ? motor->m_speed_pid_set_rpm : motor->m_speed_command_rpm; // Ramped ERPM target
	float error = target_erpm - erpm;

	if (fabsf(target_erpm) < conf_now->s_pid_min_erpm) {
		motor->m_speed_i_term = 0.0f;
		motor->m_speed_prev_error = error;
		motor->m_iq_set = 0.0f;
		return;
	}

	float p_term = error * conf_now->s_pid_kp;
	float d_term = (error - motor->m_speed_prev_error) * (conf_now->s_pid_kd / dt);

	UTILS_LP_FAST(motor->m_speed_d_filter, d_term, conf_now->s_pid_kd_filter);
	d_term = motor->m_speed_d_filter;

	motor->m_speed_prev_error = error;

	float output = p_term + motor->m_speed_i_term + d_term;
	utils_truncate_number_abs(&output, 1.0f);

	// Conditional Integration Anti-Windup: Pause integral accumulation during output saturation
	if (!((output >= 1.0f && error > 0.0f) || (output <= -1.0f && error < 0.0f))) {
		motor->m_speed_i_term += error * conf_now->s_pid_ki * dt;
		utils_truncate_number_abs(&motor->m_speed_i_term, 1.0f);
	}

	// Speed PID calculates target Iq current (Amperes) clamped to motor max current
	motor->m_iq_set = output * conf_now->l_current_max;
}

/**
  * @brief  VESC Field Weakening Controller
  */
void foc_run_fw(motor_all_state_t *motor, float dt) {
	mc_configuration *conf = motor->m_conf;
	motor_state_t *state_m = &motor->m_motor_state;

	if (conf->foc_fw_current_max < 0.001f) return;

	if (motor->m_state == MC_STATE_RUNNING &&
			(motor->m_control_mode == CONTROL_MODE_CURRENT ||
			 motor->m_control_mode == CONTROL_MODE_SPEED ||
			 motor->m_control_mode == CONTROL_MODE_POS)) {
		
		float fw_current_now = 0.0f;
		float duty_abs = fabsf(state_m->duty_now);

		if (conf->foc_fw_duty_start < 0.99f && duty_abs > conf->foc_fw_duty_start * conf->l_max_duty) {
			float i_fw_max = conf->foc_fw_current_max;

			if (conf->foc_fw_backoff > 0.001f) {
				float i_err_backoff = SIGN(motor->m_speed_est_fast) * (state_m->iq - state_m->iq_target) / i_fw_max;
				i_err_backoff *= conf->foc_fw_backoff;
				utils_truncate_number(&i_err_backoff, 0.0f, 1.0f);
				i_fw_max *= (1.0f - i_err_backoff);
			}

			fw_current_now = utils_map(duty_abs,
					conf->foc_fw_duty_start * conf->l_max_duty,
					conf->l_max_duty,
					0.0f, i_fw_max);
		}

		utils_step_towards((float*)&motor->m_i_fw_set, fw_current_now,
				(dt / conf->foc_fw_ramp_time) * conf->foc_fw_current_max);
	}
}

/**
  * @brief  Precalculate inductance & voltage norms
  */
void foc_precalc_values(motor_all_state_t *motor) {
	const mc_configuration *conf_now = motor->m_conf;
	motor->p_lq = conf_now->foc_motor_l + conf_now->foc_motor_ld_lq_diff * 0.5f;
	motor->p_ld = conf_now->foc_motor_l - conf_now->foc_motor_ld_lq_diff * 0.5f;
	motor->m_observer_state.lambda_est = conf_now->foc_motor_flux_linkage;
	motor->p_duty_norm = TWO_BY_SQRT3 / conf_now->foc_overmod_factor;
	motor->p_fs = conf_now->foc_f_zv;
	motor->p_dt = 1.0f / motor->p_fs;
}

/**
  * @brief  Update Multi-turn Motor Shaft & 1:17 Cycloidal Joint Angle Accumulator
  */
void foc_update_cycloidal_joint_angle(motor_all_state_t *motor, float raw_mech_angle_rad) {
	if (motor == NULL || motor->m_conf == NULL) return;

	motor->m_mech_angle_single = raw_mech_angle_rad;

	// Detect 0 <-> 2PI boundary rollover
	float d_angle = motor->m_mech_angle_single - motor->m_prev_mech_angle;

	if (d_angle < -(float)M_PI) {
		motor->m_turn_count++;
	} else if (d_angle > (float)M_PI) {
		motor->m_turn_count--;
	}
	motor->m_prev_mech_angle = motor->m_mech_angle_single;

	// Total motor angle in Radians
	motor->m_total_mech_angle = ((float)motor->m_turn_count * 2.0f * (float)M_PI) + motor->m_mech_angle_single;

	// Output Joint Angle after Cycloidal reduction (1:17 gear ratio)
	motor->m_joint_angle = (motor->m_total_mech_angle / motor->m_conf->gear_ratio) * (float)motor->m_conf->encoder_direction;
}
