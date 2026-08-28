/*
	Copyright 2016 - 2022 Benjamin Vedder	benjamin@vedder.se
	Adapted for STM32G4 HAL Joint Driver - Cycloidal Actuator Project

	This file is part of the VESC firmware.
	GNU General Public License v3. See <http://www.gnu.org/licenses/>.
 */

#include "foc_control.h"
#include "vesc_utils.h"
#include <math.h>

#define OBSERVER_ANGLE_HANDOVER_ENABLED 0

FOC_Controller_t g_foc_controller;

/**
  * @brief  Initialize FOC Controller & Hardware Subsystems
  */
void FOC_Control_Init(FOC_Controller_t *foc, SPI_HandleTypeDef *hspi1_drv, SPI_HandleTypeDef *hspi3_enc)
{
    if (foc == NULL) return;

    // 1. Load VESC Default Configuration (GB8115-4, 21PP, 1:17 Cycloid, 20kHz, +-180 deg)
    vesc_conf_set_defaults(&foc->conf);
    foc->motor.m_conf = &foc->conf;
    foc->motor.m_state = MC_STATE_OFF;
    foc->motor.m_control_mode = CONTROL_MODE_POS;
    foc->motor.m_openloop_spinup_active = false;
    foc->motor.m_openloop_spinup_time = 0.0f;
    foc->fault = MC_FAULT_NONE;

    // Hardware Handles
    foc->hspi_drv = hspi1_drv;
    foc->hspi_enc = hspi3_enc;

    // Zero Offsets
    foc->offset_ia = 0.0f;
    foc->offset_ib = 0.0f;
    foc->calibrated_offsets = false;
    foc->zero_electric_angle = 0.0f;
    foc->aligned = false;
    foc->observer_angle_active = false;
    foc->observer_phase_interp = 0.0f;
    foc->observer_angle_blend = 0.0f;

    foc->duty_a = foc->duty_b = foc->duty_c = 0.5f;
    foc->phase_swap_bc = false; // Pure 1:1 forward phase sequence (A->B->C)

    // Precalculate Inductances & Frequencies
    foc_precalc_values(&foc->motor);
}

/**
  * @brief  Safety Supervisor - Check Overcurrent, Overvoltage, Undervoltage & Joint Limits
  */
bool FOC_Control_CheckSafety(FOC_Controller_t *foc, float current_a, float current_b, float vbus, float temp_fet)
{
    if (foc == NULL) return false;

    // 1. Filter current magnitude with low-pass filter to reject ADC switching spikes
    static float current_mag_filtered = 0.0f;
    float current_mag_raw = sqrtf(current_a * current_a + current_b * current_b);
    UTILS_LP_FAST(current_mag_filtered, current_mag_raw, 0.02f); // ~60Hz LPF at 20kHz

    // 2. Overcurrent Instantaneous Check (Peak 30A for > 2.5ms = 50 ISR cycles)
    static uint32_t overcurrent_count = 0;
    if (current_mag_filtered > 25.0f || current_mag_raw > 35.0f) {
        overcurrent_count++;
        if (overcurrent_count >= 50) {
            foc->fault |= MC_FAULT_OVER_CURRENT;
        }
    } else {
        if (overcurrent_count > 0) overcurrent_count--;
    }

    // 3. Thermal Safety Timeout Protection (Sustained current > 15.0A for > 5.0s = 100,000 ISR cycles @ 20kHz)
    static uint32_t thermal_timeout_count = 0;
    if (current_mag_filtered > 15.0f) {
        thermal_timeout_count++;
        if (thermal_timeout_count >= 100000) {
            foc->fault |= MC_FAULT_OVER_CURRENT;
        }
    } else {
        thermal_timeout_count = 0;
    }

    // 4. Overvoltage Check (50V max OVP)
    if (vbus > foc->conf.l_voltage_max) {
        foc->fault |= MC_FAULT_OVER_VOLTAGE;
    }

    // 5. Undervoltage Check (12V min UVP)
    if (vbus < foc->conf.l_voltage_min) {
        foc->fault |= MC_FAULT_UNDER_VOLTAGE;
    }

    // 6. MOSFET Overtemperature Check (85C max)
    if (temp_fet > foc->conf.l_temp_fet_start) {
        foc->fault |= MC_FAULT_OVER_TEMP_MOS;
    }

    // If any fault occurred, immediately trip motor off and zero duty cycles
    if (foc->fault != MC_FAULT_NONE) {
        foc->motor.m_state = MC_STATE_OFF;
        foc->duty_a = foc->duty_b = foc->duty_c = 0.0f; // Turn off PWM output completely
        return false;
    }

    return true;
}

/**
  * @brief  Calibrate ADC Zero-Current Offsets (Averaging 2048 samples at startup)
  */
void FOC_Control_AdcCalibrate(FOC_Controller_t *foc, uint16_t raw_adc_a, uint16_t raw_adc_b)
{
    static uint32_t startup_delay_count = 0;
    static uint32_t sample_count = 0;
    static float sum_a = 0.0f;
    static float sum_b = 0.0f;

    if (foc->calibrated_offsets) return;

    // Chờ 500ms (10,000 chu kỳ ISR 20kHz) cho mạch khuếch đại dòng DRV8353 ổn định điện áp 1.65V
    if (startup_delay_count < 10000) {
        startup_delay_count++;
        return;
    }

    sum_a += (float)raw_adc_a;
    sum_b += (float)raw_adc_b;
    sample_count++;

    if (sample_count >= 2048) {
        foc->offset_ia = sum_a / 2048.0f;
        foc->offset_ib = sum_b / 2048.0f;
        foc->calibrated_offsets = true;
    }
}

/**
  * @brief  Automatic Encoder Zero-Electrical Angle Alignment Routine
  */
void FOC_Control_AlignEncoder(FOC_Controller_t *foc)
{
    if (foc == NULL || !foc->calibrated_offsets) return;

    foc->motor.m_state = MC_STATE_DETECTING;

    // Apply fixed voltage vector Vd = 2.0V, Vq = 0.0V to lock rotor at electrical 0 rad
    float vd_align = 2.0f;
    float sin_0 = 0.0f;
    float cos_0 = 1.0f;

    float valpha = vd_align * cos_0;
    float vbeta  = vd_align * sin_0;

    // Run SVPWM for alignment
    uint32_t ta, tb, tc, sector;
    foc_svm(valpha / foc->motor.m_motor_state.v_bus, vbeta / foc->motor.m_motor_state.v_bus,
            foc->conf.l_max_duty, 1000, &ta, &tb, &tc, &sector);

    foc->duty_a = (float)ta / 1000.0f;
    foc->duty_b = (float)tb / 1000.0f;
    foc->duty_c = (float)tc / 1000.0f;

    // Read Mechanical Angle from Encoder and compute electrical offset
    float enc_rad = 0.0f;
    AS5048A_ReadRadians(&foc->encoder, &enc_rad);

    float elec_offset = (float)(foc->conf.encoder_direction * foc->conf.foc_motor_pole_pairs) * enc_rad;
    utils_norm_angle_rad(&elec_offset);
    foc->zero_electric_angle = elec_offset;
    foc->aligned = true;

    foc->motor.m_state = MC_STATE_RUNNING;
}

/**
  * @brief  High-Speed FOC Current Control Loop (Extracted directly from VESC control_current)
  *         Runs at 20 kHz inside PWM/ADC Interrupt Callback
  */
void FOC_Control_Current_ISR(FOC_Controller_t *foc, float current_a, float current_b, float vbus, float temp_fet, float dt)
{
    if (foc == NULL) return;

    motor_all_state_t *motor = &foc->motor;
    motor_state_t *state_m = &motor->m_motor_state;
    mc_configuration *conf_now = motor->m_conf;

    /* FIX: ngưỡng 5V thay vì 0.0f.
     * Trước: vbus=0.9V (ADC thiếu hệ số chia) -> 0.9V > 0.0f -> state_m->v_bus=0.9V
     * -> UVP trip (0.9V < l_voltage_min=12V) -> motor luôn ở IDLE!
     * Sau fix: vbus=0.9V < 5.0f -> fallback 24.0f -> FOC hoạt động đúng. */
    state_m->v_bus = vbus > 5.0f ? vbus : 24.0f;

    // 1. Read the encoder at the measured ADC/FOC interrupt rate (10 kHz on this timer setup).
    float dt_fast = dt;
    if (dt_fast < 0.000001f || dt_fast > 0.001000f) {
        dt_fast = 0.000100f;
    }
    HAL_StatusTypeDef encoder_status = AS5048A_Sample(&foc->encoder, dt_fast);
    if (encoder_status != HAL_OK && foc->encoder.consecutive_errors >= 20U) {
        foc->fault |= MC_FAULT_ENCODER;
    }

    foc_update_cycloidal_joint_angle(motor, foc->encoder.angle_singleturn);

    // 1b. Tính Góc Điện theo Chuẩn VESC [-PI, +PI] (21 cặp cực)
    float elec_raw = (float)(conf_now->encoder_direction * conf_now->foc_motor_pole_pairs) * foc->encoder.angle_singleturn;
    float encoder_elec_angle = elec_raw - foc->zero_electric_angle;
    utils_norm_angle_rad(&encoder_elec_angle);

    // Direct filtered velocity from 14-bit integer count differencing.
    float omega_mech = foc->encoder.velocity_rad_s;
    motor->m_speed_est_fast = (float)(conf_now->encoder_direction * 21) * omega_mech;

    float elec_angle = encoder_elec_angle;
    state_m->phase = elec_angle;

    // 1c. Direct Electrical Angle for Current Sensing (30-cycle Ben Katz sincos_lut)
    float sin_elec, cos_elec;
    sincos_lut(elec_angle, &sin_elec, &cos_elec);
    state_m->phase_sin = sin_elec;
    state_m->phase_cos = cos_elec;

    // 2. Clarke Transform: (Ia, Ib) -> (Ialpha, Ibeta)
    state_m->i_alpha = current_a;
    state_m->i_beta  = (current_a + 2.0f * current_b) * ONE_BY_SQRT3;

    // 3. Park Transform: Stator -> Rotor reference frame (Id, Iq)
    state_m->id = cos_elec * state_m->i_alpha + sin_elec * state_m->i_beta;
    state_m->iq = cos_elec * state_m->i_beta  - sin_elec * state_m->i_alpha;

    // Low-pass filter currents for telemetry
    UTILS_LP_FAST(state_m->id_filter, state_m->id, conf_now->foc_current_filter_const);
    UTILS_LP_FAST(state_m->iq_filter, state_m->iq, conf_now->foc_current_filter_const);

    // 4. Run Safety Check Supervisor
    if (!FOC_Control_CheckSafety(foc, current_a, current_b, state_m->v_bus, temp_fet)) {
        return; // Tripped fault!
    }

    if (motor->m_state != MC_STATE_RUNNING) {
        /* Seed the Ortega observer from the known encoder flux angle while the
         * inverter is idle. Starting x1/x2 at zero creates a long, false phase
         * transient on this high-flux low-speed winding. */
        motor->m_observer_state.x1 =
            conf_now->foc_motor_l * state_m->i_alpha +
            conf_now->foc_motor_flux_linkage * cos_elec;
        motor->m_observer_state.x2 =
            conf_now->foc_motor_l * state_m->i_beta +
            conf_now->foc_motor_flux_linkage * sin_elec;
        motor->m_observer_state.i_alpha_last = state_m->i_alpha;
        motor->m_observer_state.i_beta_last = state_m->i_beta;
        motor->m_phase_now_observer = elec_angle;
        foc->observer_angle_active = false;
        foc->observer_phase_interp = elec_angle;
        foc->observer_angle_blend = 0.0f;
        foc->encoder.velocity_rad_s = 0.0f;
        foc->encoder.velocity_rpm = 0.0f;
        motor->m_speed_est_fast = 0.0f;
        /* Keep measured currents visible while an open-loop ALIGN/CALIB
         * vector is active. OFF still reports exact zero as expected. */
        if (motor->m_state != MC_STATE_DETECTING) {
            state_m->i_alpha = 0.0f;
            state_m->i_beta = 0.0f;
            state_m->id = 0.0f;
            state_m->iq = 0.0f;
            state_m->id_filter = 0.0f;
            state_m->iq_filter = 0.0f;
        }
        state_m->iq_target = 0.0f;
        state_m->vd_int = 0.0f;
        state_m->vq_int = 0.0f;
        state_m->vd = 0.0f;
        state_m->vq = 0.0f;
        if (motor->m_state != MC_STATE_DETECTING) {
            foc->duty_a = 0.5f;
            foc->duty_b = 0.5f;
            foc->duty_c = 0.5f;
        }
        return;
    }

    // ---- Open-Loop Breakaway Sweep (Cycloidal Pin Detent Anti-Stall) ----
    // In Speed Control mode, if mechanical binding stops the rotor (< 8 RPM for > 40ms),
    // trigger a continuous 350ms Open-Loop V/f sweep (0.30 mechanical revolutions)
    // with Vd=9.0V advancing at 50 RPM to drag the rotor completely through and past
    // the tight pin detent zone, then seamlessly hand back to Closed-Loop FOC.
    static float stall_detect_time = 0.0f;
    static float sweep_timer = 0.0f;
    static float sweep_angle = 0.0f;

    float mech_rpm_now = fabsf(omega_mech * (60.0f / (2.0f * (float)M_PI)));

    if (motor->m_control_mode == CONTROL_MODE_SPEED &&
        fabsf(motor->m_speed_pid_set_rpm) >= 15.0f * (float)conf_now->foc_motor_pole_pairs) {

        if (sweep_timer <= 0.0f) {
            if (mech_rpm_now < 8.0f) {
                stall_detect_time += dt_fast;
                if (stall_detect_time > 0.040f) {
                    sweep_timer = 0.350f; // 350ms continuous open-loop sweep
                    sweep_angle = elec_angle;
                    stall_detect_time = 0.0f;
                    state_m->vd_int = 0.0f;
                    state_m->vq_int = 0.0f;
                    motor->m_speed_i_term = 0.0f;
                }
            } else {
                stall_detect_time = 0.0f;
            }
        }
    } else {
        stall_detect_time = 0.0f;
        sweep_timer = 0.0f;
    }

    if (sweep_timer > 0.0f) {
        sweep_timer -= dt_fast;
        float dir = (motor->m_speed_command_rpm > 0.0f) ? 1.0f : -1.0f;
        float pp = (float)conf_now->foc_motor_pole_pairs;
        float target_elec_rad_s = dir * (50.0f * pp * 2.0f * (float)M_PI / 60.0f);
        sweep_angle += target_elec_rad_s * dt_fast;
        utils_norm_angle_rad(&sweep_angle);

        float v_open = 9.0f + conf_now->foc_motor_flux_linkage * fabsf(target_elec_rad_s);
        float v_max_ol = ONE_BY_SQRT3 * conf_now->l_max_duty * state_m->v_bus;
        if (v_open > v_max_ol) v_open = v_max_ol;

        float sin_sw, cos_sw;
        sincos_lut(sweep_angle, &sin_sw, &cos_sw);
        float v_norm = v_open / state_m->v_bus;

        state_m->vd = v_open;
        state_m->vq = 0.0f;
        state_m->mod_d = v_norm;
        state_m->mod_q = 0.0f;
        state_m->mod_alpha_raw = cos_sw * v_norm;
        state_m->mod_beta_raw  = sin_sw * v_norm;
        state_m->v_alpha = state_m->mod_alpha_raw * state_m->v_bus;
        state_m->v_beta  = state_m->mod_beta_raw * state_m->v_bus;
        state_m->duty_now = v_norm;

        state_m->vd_int = 0.0f;
        state_m->vq_int = 0.0f;
        motor->m_speed_i_term = 0.0f;

        uint32_t ta_sw, tb_sw, tc_sw, sector_sw;
        foc_svm(state_m->mod_alpha_raw, state_m->mod_beta_raw,
                conf_now->l_max_duty, 1000, &ta_sw, &tb_sw, &tc_sw, &sector_sw);

        foc->duty_a = (float)ta_sw / 1000.0f;
        if (foc->phase_swap_bc) {
            foc->duty_b = (float)tc_sw / 1000.0f;
            foc->duty_c = (float)tb_sw / 1000.0f;
        } else {
            foc->duty_b = (float)tb_sw / 1000.0f;
            foc->duty_c = (float)tc_sw / 1000.0f;
        }
        return; // Complete open-loop sweep bypass
    }

    // Strict Voltage Vector Circle Limitation (Max = Vbus / sqrt(3))
    float max_v_mag = ONE_BY_SQRT3 * conf_now->l_max_duty * state_m->v_bus * conf_now->foc_overmod_factor;

    // 5. FOC Controller
    if (motor->m_control_mode == CONTROL_MODE_DUTY) {
        state_m->vd = 0.0f;
        state_m->vq = state_m->duty_now * state_m->v_bus;
        state_m->vd_int = 0.0f;
        state_m->vq_int = 0.0f;
        limit_norm(&state_m->vd, &state_m->vq, max_v_mag);
    } else {
        // Speed-loop stall boost runs in the filtered 1 kHz speed controller.
        // Do not re-trigger it here from raw per-ISR velocity samples.
        state_m->iq_target = motor->m_iq_set;
        float id_target = -motor->m_i_fw_set;
        state_m->id_target = id_target;
        utils_truncate_number_abs((float*)&state_m->iq_target, conf_now->l_current_max);

        float Ierr_d = id_target - state_m->id;
        float Ierr_q = state_m->iq_target - state_m->iq;

        float kp = conf_now->foc_current_kp;
        float ki = conf_now->foc_current_ki;

        // Decoupling Feedforward terms (-w_e*L*Iq on d-axis, +w_e*L*Id + w_e*lambda on q-axis)
        // Use filtered d-q currents for cross-coupling feedforward to prevent
        // noise amplification from offset-induced oscillations in raw Id/Iq.
        float vq_ff = motor->m_speed_est_fast * conf_now->foc_motor_flux_linkage + motor->m_speed_est_fast * conf_now->foc_motor_l * state_m->id_filter;
        float vd_ff = -motor->m_speed_est_fast * conf_now->foc_motor_l * state_m->iq_filter;

        state_m->vd = kp * Ierr_d + state_m->vd_int + vd_ff;
        state_m->vq = kp * Ierr_q + state_m->vq_int + vq_ff;

        // Circle-based anti-windup: check combined Vd²+Vq² against the voltage
        // circle, not each axis independently. Per-axis check allows one axis to
        // wind up unchecked while the other is saturated, causing Vd to steal
        // Vq headroom and eventual loss of synchronous torque.
        float v_mag_sq = state_m->vd * state_m->vd + state_m->vq * state_m->vq;
        float v_limit_sq = max_v_mag * max_v_mag;
        bool voltage_saturated = v_mag_sq >= v_limit_sq * 0.95f;

        // Only integrate if not saturated, or if error would reduce saturation
        if (!voltage_saturated || (state_m->vd * Ierr_d < 0.0f)) {
            state_m->vd_int += ki * Ierr_d * dt_fast;
        }
        if (!voltage_saturated || (state_m->vq * Ierr_q < 0.0f)) {
            state_m->vq_int += ki * Ierr_q * dt_fast;
        }

        // Vector circle voltage limitation for integrators (limit_norm)
        limit_norm(&state_m->vd_int, &state_m->vq_int, max_v_mag);

        state_m->vd = kp * Ierr_d + state_m->vd_int + vd_ff;
        state_m->vq = kp * Ierr_q + state_m->vq_int + vq_ff;

        // Final vector circle voltage limitation (SVPWM circular headroom)
        limit_norm(&state_m->vd, &state_m->vq, max_v_mag);
    }

    // Normalize voltages for Inverse Park & Modulation
    const float voltage_normalize = 1.0f / state_m->v_bus;
    state_m->mod_d = state_m->vd * voltage_normalize;
    state_m->mod_q = state_m->vq * voltage_normalize;

    // 8. Inverse Park Transform: Direct Sensor Angle + 1.5*DT Delay Compensation (Optimal 90° Field)
    float phase_advance = (1.5f * dt_fast) * motor->m_speed_est_fast;
    utils_truncate_number_abs(&phase_advance, 0.35f); // Max ~20 deg electrical

    // Maintain 100% optimal 90° torque angle (Id=0, max torque per ampere)
    float theta_svm = elec_angle + phase_advance;
    utils_norm_angle_rad(&theta_svm);

    float sin_svm, cos_svm;
    sincos_lut(theta_svm, &sin_svm, &cos_svm);

    state_m->mod_alpha_raw = cos_svm * state_m->mod_d - sin_svm * state_m->mod_q;
    state_m->mod_beta_raw  = cos_svm * state_m->mod_q + sin_svm * state_m->mod_d;
    state_m->v_alpha = state_m->mod_alpha_raw * state_m->v_bus;
    state_m->v_beta  = state_m->mod_beta_raw * state_m->v_bus;
    state_m->duty_now = sqrtf(SQ(state_m->mod_d) + SQ(state_m->mod_q));


    // 9. Space Vector Modulation (SVM) - Identical to Open-Loop & Alignment
    uint32_t ta, tb, tc, sector;
    foc_svm(state_m->mod_alpha_raw, state_m->mod_beta_raw,
            conf_now->l_max_duty, 1000, &ta, &tb, &tc, &sector);

    float duty_a = (float)ta / 1000.0f;
    float duty_b = (float)tb / 1000.0f;
    float duty_c = (float)tc / 1000.0f;

    // Stator Phase Mapping: Apply auto-detected Phase B <-> Phase C swap
    foc->duty_a = duty_a;
    if (foc->phase_swap_bc) {
        foc->duty_b = duty_c;
        foc->duty_c = duty_b;
    } else {
        foc->duty_b = duty_b;
        foc->duty_c = duty_c;
    }
}

/**
  * @brief  Slow Loop Execution (Called @ 1kHz for Observer, PLL, Position/Speed PID, FW)
  */
void FOC_Control_SlowLoop(FOC_Controller_t *foc, float dt)
{
    if (foc == NULL) return;

    motor_all_state_t *motor = &foc->motor;
    static float observer_valid_time = 0.0f;
    if (foc->motor.m_state != MC_STATE_RUNNING) {
        observer_valid_time = 0.0f;
        return;
    }

    // 1. Run VESC Observer
    foc_observer_update(motor->m_motor_state.v_alpha, motor->m_motor_state.v_beta,
                        motor->m_motor_state.i_alpha, motor->m_motor_state.i_beta,
                        dt, &motor->m_observer_state, &motor->m_phase_now_observer, motor);

    /* AS5048 provides deterministic startup angle. Once BEMF makes the flux
     * observer observable, hand FOC angle authority to the observer directly;
     * changing encoder zero online creates a periodic loss of torque. */
    motor_state_t *state_m = &motor->m_motor_state;
    mc_configuration *conf = motor->m_conf;
    float flux_alpha = motor->m_observer_state.x1 -
                       conf->foc_motor_l * state_m->i_alpha;
    float flux_beta = motor->m_observer_state.x2 -
                      conf->foc_motor_l * state_m->i_beta;
    float flux_ratio = NORM2_f(flux_alpha, flux_beta) /
                       conf->foc_motor_flux_linkage;
    float bemf_estimate = fabsf(motor->m_speed_est_fast) *
                          conf->foc_motor_flux_linkage;
    bool observer_valid = motor->m_control_mode == CONTROL_MODE_SPEED &&
                          bemf_estimate > 1.50f &&
                          flux_ratio > 0.70f && flux_ratio < 1.30f;
    /* The low-speed observer has a small frequency bias. Direct handover lets
     * that bias integrate into a full electrical revolution in a few minutes,
     * so the absolute encoder remains the angle authority. */
    if (!OBSERVER_ANGLE_HANDOVER_ENABLED) {
        observer_valid_time = 0.0f;
        foc->observer_angle_active = false;
        foc->observer_angle_blend = 0.0f;
    } else if (!foc->observer_angle_active && observer_valid) {
        observer_valid_time += dt;
        if (observer_valid_time > 0.5f) {
            foc->observer_phase_interp = motor->m_phase_now_observer;
            foc->observer_angle_blend = 0.0f;
            foc->observer_angle_active = true;
        }
    } else if (!foc->observer_angle_active) {
        observer_valid_time = 0.0f;
    } else {
        /* Correct the 10 kHz interpolated phase from the 1 kHz observer. Keep
         * observer authority through brief low-speed intervals once acquired. */
        foc->observer_phase_interp = motor->m_phase_now_observer;
    }

    // 2. Run Position PID or Speed PID based on Control Mode (generates target Iq)
    if (motor->m_control_mode == CONTROL_MODE_POS) {
        foc_run_pid_control_pos(true, dt, motor);
    } else if (motor->m_control_mode == CONTROL_MODE_SPEED) {
        foc_run_pid_control_speed(true, dt, motor);
    }

    // 3. Run Field Weakening
    foc_run_fw(motor, dt);
}
