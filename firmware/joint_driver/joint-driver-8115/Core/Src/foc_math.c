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
  * @brief  Set Current Physical Position as Mechanical Home Zero (0.0 rad / 0.0 deg)
  */
void foc_set_home_position(motor_all_state_t *motor) {
	if (motor == NULL) return;
	motor->m_mech_home_offset = motor->m_mech_angle_single;
	motor->m_home_calibrated = true;
	motor->m_turn_count = 0;
	motor->m_prev_mech_angle = motor->m_mech_angle_single;
	motor->m_total_mech_angle = 0.0f;
	motor->m_joint_angle = 0.0f;
	motor->m_pos_pid_set = 0.0f;
	motor->m_traj_active = false;
}

/**
  * @brief  Start a smooth S-Curve trajectory move for the robot joint
  * @param  target_angle_rad: Target joint angle in Radians
  * @param  duration_s: Time to move from current position to target in seconds
  * @param  max_current_a: Maximum current limit for movement & holding (Amperes)
  */
void foc_start_trajectory(motor_all_state_t *motor, float target_angle_rad, float duration_s, float max_current_a) {
	if (motor == NULL || motor->m_conf == NULL) return;
	if (duration_s < 0.05f) duration_s = 0.05f; // Min 50ms to prevent infinite acceleration
	if (max_current_a < 0.2f) max_current_a = 0.2f;
	if (max_current_a > 10.0f) max_current_a = 10.0f; // Max 10A safety limit

	motor->m_traj_start_angle = motor->m_joint_angle;
	motor->m_traj_target_angle = target_angle_rad;
	motor->m_traj_duration = duration_s;
	motor->m_traj_time = 0.0f;
	motor->m_pos_holding_current_limit = max_current_a;
	motor->m_traj_active = true;
	motor->m_pos_pid_set = motor->m_joint_angle;
	motor->m_control_mode = CONTROL_MODE_POS;
	motor->m_state = MC_STATE_RUNNING;
}

/**
  * @brief  VESC Position Controller Loop (Cascaded Position -> Velocity -> Voltage FOC)
  */
void foc_run_pid_control_pos(bool index_found, float dt, motor_all_state_t *motor) {
	mc_configuration *conf_now = motor->m_conf;

	float angle_now = motor->m_joint_angle; // Controlled on Joint Output Angle

	if (motor->m_control_mode != CONTROL_MODE_POS) {
		motor->m_pos_i_term = 0.0f;
		motor->m_pos_prev_error = 0.0f;
		motor->m_pos_prev_proc = angle_now;
		motor->m_pos_d_filter = 0.0f;
		motor->m_pos_d_filter_proc = 0.0f;
		motor->m_traj_active = false;
		return;
	}

	// 1. S-Curve Trajectory Interpolation (Quintic Minimum-Jerk Polynomial)
	if (motor->m_traj_active) {
		motor->m_traj_time += dt;
		if (motor->m_traj_time >= motor->m_traj_duration) {
			motor->m_traj_time = motor->m_traj_duration;
			motor->m_traj_active = false; // Đến đích -> Khóa cứng vị trí (Rigid Hold)
			motor->m_pos_pid_set = motor->m_traj_target_angle;
		} else {
			// Polynomial: s(tau) = 10*tau^3 - 15*tau^4 + 6*tau^5
			float tau = motor->m_traj_time / motor->m_traj_duration;
			float tau3 = tau * tau * tau;
			float tau4 = tau3 * tau;
			float tau5 = tau4 * tau;
			float s = 10.0f * tau3 - 15.0f * tau4 + 6.0f * tau5;
			motor->m_pos_pid_set = motor->m_traj_start_angle + s * (motor->m_traj_target_angle - motor->m_traj_start_angle);
		}
	}

	float angle_set = motor->m_pos_pid_set;

	// Clamp target within software joint limits (-PI to +PI rad = -180 to +180 deg)
	utils_truncate_number(&angle_set, conf_now->joint_pos_min, conf_now->joint_pos_max);

	float error = angle_set - angle_now;

	// 2. Cascaded Velocity Demand: Sinh tốc độ quay mượt mà (Max 120 RPM) kéo rotor tới đích
	float target_rpm = error * 40.0f; // 40 RPM per radian of error
	utils_truncate_number(&target_rpm, -120.0f, 120.0f);

	if (fabsf(error) < 0.015f) { // Nếu sai số < ~0.8 độ, cho target_rpm = 0
		target_rpm = 0.0f;
	}

	motor->m_speed_command_rpm = target_rpm * (float)conf_now->foc_motor_pole_pairs;

	// Chạy bộ điều khiển Vận tốc đã có tính năng mồi 0-RPM tự động
	foc_run_pid_control_speed(index_found, dt, motor);

	// 3. Stiff Holding Torque: Bơm thêm mô-men phục hồi vị trí trực tiếp để ghim cứng tại chỗ
	float hold_limit_a = (motor->m_pos_holding_current_limit > 0.1f) ? motor->m_pos_holding_current_limit : 3.0f;
	float max_hold_v = hold_limit_a * conf_now->foc_motor_r;
	if (max_hold_v > 12.0f) max_hold_v = 12.0f;
	if (max_hold_v < 2.0f) max_hold_v = 2.0f;

	float hold_v = error * 15.0f; // Độ cứng ghì trục 15V/rad
	utils_truncate_number_abs(&hold_v, max_hold_v);

	motor->m_iq_set += hold_v;
	utils_truncate_number_abs(&motor->m_iq_set, max_hold_v);
}

/**
  * @brief  VESC Speed Controller Loop (PID in Voltage-Mode FOC with Back-EMF Feedforward)
  */
void foc_run_pid_control_speed(bool index_found, float dt, motor_all_state_t *motor) {
	mc_configuration *conf_now = motor->m_conf;

	if (motor->m_control_mode != CONTROL_MODE_SPEED && motor->m_control_mode != CONTROL_MODE_POS) {
		motor->m_speed_i_term = 0.0f;
		motor->m_speed_prev_error = 0.0f;
		motor->m_speed_d_filter = 0.0f;
		return;
	}
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

	// Maximum voltage available: Vmax = Vbus / sqrt(3) * l_max_duty
	float max_v = ONE_BY_SQRT3 * conf_now->l_max_duty * motor->m_motor_state.v_bus;
	if (max_v < 1.0f) max_v = 12.0f;

	// Smooth Zero-Speed Breakaway Boost (2.5V ~ 0.64A): Bơm đủ lực kéo nhẹ lúc đứng yên, tự động giảm về 0 khi có trớn
	float v_boost = 0.0f;
	if (fabsf(erpm) < 300.0f) {
		float fade = 1.0f - (fabsf(erpm) / 300.0f);
		v_boost = (target_erpm > 0.0f) ? (2.5f * fade) : (-2.5f * fade);
	}

	// Back-EMF Feedforward: Vq_ff = omega_e * lambda
	float vq_ff = motor->m_speed_est_fast * conf_now->foc_motor_flux_linkage;

	// Dynamic Voltage Authority: Khóa điện áp tương ứng với dải tốc độ, chống vọt dòng quá tải nhiệt 5s
	float v_limit = 3.5f + (fabsf(target_erpm) * 0.0025f);
	if (v_limit > max_v) v_limit = max_v;

	// Speed PID calculates Vq voltage output (Volts)
	float vq_out = v_boost + (output * v_limit) + vq_ff;
	utils_truncate_number_abs(&vq_out, v_limit);
	motor->m_iq_set = vq_out;
}

/**
  * @brief  VESC Field Weakening Controller
  */
void foc_run_fw(motor_all_state_t *motor, float dt) {
	mc_configuration *conf = motor->m_conf;
	motor_state_t *state_m = &motor->m_motor_state;

	if (conf->foc_fw_current_max < 0.001f) return;

	float current_max = conf->l_current_max;
	float i_mag = NORM2_f(state_m->id, state_m->iq);

	if (i_mag > current_max) {
		motor->m_i_fw_set = 0.0f;
		return;
	}

	float duty = state_m->duty_now;
	float duty_target = conf->foc_fw_duty_start;

	if (duty > duty_target) {
		float duty_err = duty - duty_target;
		motor->m_i_fw_set += duty_err * conf->foc_fw_current_max * dt * 2.0f;
		if (motor->m_i_fw_set > conf->foc_fw_current_max) {
			motor->m_i_fw_set = conf->foc_fw_current_max;
		}
	} else {
		motor->m_i_fw_set -= (duty_target - duty) * conf->foc_fw_current_max * dt * 4.0f;
		if (motor->m_i_fw_set < 0.0f) {
			motor->m_i_fw_set = 0.0f;
		}
	}
}

/**
  * @brief  Pre-calculate frequent FOC constants from configuration
  */
void foc_precalc_values(motor_all_state_t *motor) {
	mc_configuration *conf_now = motor->m_conf;
	motor->p_lq = conf_now->foc_motor_l;
	motor->p_ld = conf_now->foc_motor_l;
	motor->p_duty_norm = TWO_BY_SQRT3 / conf_now->foc_overmod_factor;
	motor->p_fs = conf_now->foc_f_zv;
	motor->p_dt = 1.0f / motor->p_fs;
}

/**
  * @brief  Update Multi-turn Motor Shaft & 1:17 Cycloidal Joint Angle Accumulator from Home (0.0)
  */
void foc_update_cycloidal_joint_angle(motor_all_state_t *motor, float raw_mech_angle_rad) {
	if (motor == NULL || motor->m_conf == NULL) return;

	// Lần đầu bật nguồn: Tự động ghi nhận góc bật nguồn làm Home 0.0 nếu chưa được Set Home thủ công
	if (!motor->m_home_calibrated) {
		motor->m_mech_home_offset = raw_mech_angle_rad;
		motor->m_home_calibrated = true;
		motor->m_prev_mech_angle = raw_mech_angle_rad;
		motor->m_turn_count = 0;
	}

	motor->m_mech_angle_single = raw_mech_angle_rad;

	// Phát hiện tràn ranh giới 0 <-> 2PI
	float d_angle = motor->m_mech_angle_single - motor->m_prev_mech_angle;

	if (d_angle < -(float)M_PI) {
		motor->m_turn_count++;
	} else if (d_angle > (float)M_PI) {
		motor->m_turn_count--;
	}
	motor->m_prev_mech_angle = motor->m_mech_angle_single;

	// Góc cơ học thực tế tương đối so với vị trí Home
	float angle_rel = motor->m_mech_angle_single - motor->m_mech_home_offset;

	// Tổng góc trục động cơ (Radians) tính từ vị trí Home
	motor->m_total_mech_angle = ((float)motor->m_turn_count * 2.0f * (float)M_PI) + angle_rel;

	// Góc đầu ra của khớp sau tỉ số truyền (mặc định 1:1 cho Direct Drive hoặc 1:17 cho hộp số)
	motor->m_joint_angle = (motor->m_total_mech_angle / motor->m_conf->gear_ratio) * (float)motor->m_conf->encoder_direction;
}
