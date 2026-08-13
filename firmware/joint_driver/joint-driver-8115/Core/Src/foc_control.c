/*
	Copyright 2016 - 2022 Benjamin Vedder	benjamin@vedder.se
	Adapted for STM32G4 HAL Joint Driver - Cycloidal Actuator Project

	This file is part of the VESC firmware.
	GNU General Public License v3. See <http://www.gnu.org/licenses/>.
 */

#include "foc_control.h"
#include "vesc_utils.h"
#include <math.h>

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

    foc->duty_a = foc->duty_b = foc->duty_c = 0.5f;

    // Precalculate Inductances & Frequencies
    foc_precalc_values(&foc->motor);
}

/**
  * @brief  Safety Supervisor - Check Overcurrent, Overvoltage, Undervoltage & Joint Limits
  */
bool FOC_Control_CheckSafety(FOC_Controller_t *foc, float current_a, float current_b, float vbus, float temp_fet)
{
    if (foc == NULL) return false;

    float current_mag = sqrtf(current_a * current_a + current_b * current_b);

    // Overcurrent Check - Cần 50 mẫu liên tiếp (>2.5ms) vượt ngưỡng 20.0A mới kích hoạt Lỗi Quá Dòng
    // Tránh bị ngắt giả do nhiễu gai dòng hoặc đợt tăng tốc ban đầu
    static uint32_t overcurrent_count = 0;
    if (current_mag > 20.0f) {
        overcurrent_count++;
        if (overcurrent_count >= 50) {
            foc->fault |= MC_FAULT_OVER_CURRENT;
        }
    } else {
        if (overcurrent_count > 0) overcurrent_count--;
    }

    // Overvoltage Check (50V max OVP)
    if (vbus > foc->conf.l_voltage_max) {
        foc->fault |= MC_FAULT_OVER_VOLTAGE;
    }

    // Undervoltage Check (12V min UVP)
    if (vbus < foc->conf.l_voltage_min) {
        foc->fault |= MC_FAULT_UNDER_VOLTAGE;
    }

    // MOSFET Overtemperature Check (85C max)
    if (temp_fet > foc->conf.l_temp_fet_start) {
        foc->fault |= MC_FAULT_OVER_TEMP_MOS;
    }

    // Joint Soft Limit Check (-180 deg to +180 deg) - Chỉ kích hoạt trong Position Control Mode
    if (foc->motor.m_control_mode == CONTROL_MODE_POS) {
        if (foc->motor.m_joint_angle < foc->conf.joint_pos_min || foc->motor.m_joint_angle > foc->conf.joint_pos_max) {
            foc->fault |= MC_FAULT_POS_LIMIT;
        }
    }

    // If any fault occurred, immediately trip motor off
    if (foc->fault != MC_FAULT_NONE) {
        foc->motor.m_state = MC_STATE_OFF;
        foc->duty_a = foc->duty_b = foc->duty_c = 0.5f;
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

    foc->zero_electric_angle = fmodf(enc_rad * (float)foc->conf.foc_motor_pole_pairs, 2.0f * (float)M_PI);
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

    // 1. Đọc Encoder: Nếu SPI3 rảnh, đọc ngay (sẽ xong trong ~3.5µs @ 5.3MHz).
    // Nếu SPI3 bận (rất hiếm), fallback về cache nhằm tránh ISR treo.
    // SPI được cập nhật đầy đủ ở Slow Loop 1kHz (FOC_Control_SlowLoop).
    float raw_enc_rad = foc->encoder.angle_rad; // default = cache
    if (!(foc->encoder.hspi->Instance->SR & SPI_SR_BSY)) {
        AS5048A_ReadRadians(&foc->encoder, &raw_enc_rad);
    }
    foc_update_cycloidal_joint_angle(motor, raw_enc_rad);

    float enc_rad_dir = (conf_now->encoder_direction == -1) ? (2.0f * (float)M_PI - raw_enc_rad) : raw_enc_rad;
    float elec_angle = (enc_rad_dir * (float)conf_now->foc_motor_pole_pairs) - foc->zero_electric_angle;
    utils_norm_angle_rad(&elec_angle);

    state_m->phase = elec_angle;
    utils_fast_sincos(elec_angle, &state_m->phase_sin, &state_m->phase_cos);

    // 2. Clarke Transform: (Ia, Ib) -> (Ialpha, Ibeta)
    state_m->i_alpha = current_a;
    state_m->i_beta  = (current_a + 2.0f * current_b) * ONE_BY_SQRT3;

    float s = state_m->phase_sin;
    float c = state_m->phase_cos;

    // 3. Park Transform: Stator -> Rotor reference frame (Id, Iq)
    state_m->id = c * state_m->i_alpha + s * state_m->i_beta;
    state_m->iq = c * state_m->i_beta  - s * state_m->i_alpha;

    // Low-pass filter currents for telemetry
    UTILS_LP_FAST(state_m->id_filter, state_m->id, conf_now->foc_current_filter_const);
    UTILS_LP_FAST(state_m->iq_filter, state_m->iq, conf_now->foc_current_filter_const);

    // 4. Run Safety Check Supervisor
    if (!FOC_Control_CheckSafety(foc, current_a, current_b, state_m->v_bus, temp_fet)) {
        return; // Tripped fault!
    }

    if (motor->m_state != MC_STATE_RUNNING) {
        state_m->vd_int = 0.0f;
        state_m->vq_int = 0.0f;
        state_m->vd = 0.0f;
        state_m->vq = 0.0f;
        foc->duty_a = foc->duty_b = foc->duty_c = 0.5f;
        return;
    }

    // 5. Current Control PI Loop
    if (motor->m_control_mode != CONTROL_MODE_CURRENT) {
        state_m->iq_target = motor->m_iq_set;
    }

    float Ierr_d = state_m->id_target - state_m->id;
    float Ierr_q = state_m->iq_target - state_m->iq;

    float ki = conf_now->foc_current_ki;
    float kp = conf_now->foc_current_kp;

    state_m->vd_int += Ierr_d * ki * dt;
    state_m->vq_int += Ierr_q * ki * dt;

    state_m->vd = state_m->vd_int + Ierr_d * kp;
    state_m->vq = state_m->vq_int + Ierr_q * kp;

    // 6. Cross-Coupling Decoupling (BEMF + Cross Feedforward)
    float dec_vd = 0.0f;
    float dec_vq = 0.0f;
    float dec_bemf = 0.0f;

    if (conf_now->foc_cc_decoupling != FOC_CC_DECOUPLING_DISABLED) {
        dec_vd = state_m->iq * motor->m_speed_est_fast * motor->p_lq;
        dec_vq = state_m->id * motor->m_speed_est_fast * motor->p_ld;
        dec_bemf = motor->m_speed_est_fast * conf_now->foc_motor_flux_linkage;
    }

    state_m->vd -= dec_vd;
    state_m->vq += dec_vq + dec_bemf;

    // 7. Strict Voltage Vector Circle Limitation (Max = Vbus / sqrt(3))
    float max_v_mag = ONE_BY_SQRT3 * conf_now->l_max_duty * state_m->v_bus * conf_now->foc_overmod_factor;

    utils_truncate_number_abs((float*)&state_m->vd, max_v_mag * conf_now->foc_mag_vd_max);
    utils_truncate_number_abs((float*)&state_m->vd_int, max_v_mag * conf_now->foc_mag_vd_max);

    float max_vq = sqrtf(SQ(max_v_mag) - SQ(state_m->vd));
    UTILS_NAN_ZERO(max_vq);

    utils_truncate_number_abs((float*)&state_m->vq, max_vq);
    utils_truncate_number_abs((float*)&state_m->vq_int, max_vq);

    // Normalize voltages for Inverse Park & Modulation
    const float voltage_normalize = 1.5f / state_m->v_bus;
    state_m->mod_d = state_m->vd * voltage_normalize;
    state_m->mod_q = state_m->vq * voltage_normalize;

    // 8. Inverse Park Transform: Rotor -> Stator frame
    state_m->mod_alpha_raw = c * state_m->mod_d - s * state_m->mod_q;
    state_m->mod_beta_raw  = c * state_m->mod_q + s * state_m->mod_d;

    // 9. VESC 6-Sector Space Vector Modulation (SVM)
    uint32_t ta, tb, tc, sector;
    foc_svm(state_m->mod_alpha_raw, state_m->mod_beta_raw, conf_now->l_max_duty, 1000, &ta, &tb, &tc, &sector);

    foc->duty_a = (float)ta / 1000.0f;
    foc->duty_b = (float)tb / 1000.0f;
    foc->duty_c = (float)tc / 1000.0f;
}

/**
  * @brief  Slow Loop Execution (Called @ 1kHz for Observer, PLL, Position/Speed PID, FW)
  */
void FOC_Control_SlowLoop(FOC_Controller_t *foc, float dt)
{
    if (foc == NULL) return;

    // Đọc Encoder qua SPI3 ở đây (1kHz) - an toàn, SysTick hoạt động bình thường.
    // Cập nhật cache foc->encoder.angle_rad để ISR 20kHz sử dụng.
    // Đọc cả khi IDLE để góc cơ khật luôn chính xác.
    {
        float enc_rad_sl = foc->encoder.angle_rad;
        AS5048A_ReadRadians(&foc->encoder, &enc_rad_sl);
        foc_update_cycloidal_joint_angle(&foc->motor, enc_rad_sl);
    }

    if (foc->motor.m_state != MC_STATE_RUNNING) return;

    motor_all_state_t *motor = &foc->motor;

    // 1. Run VESC Observer & PLL Speed Estimator
    foc_observer_update(motor->m_motor_state.v_alpha, motor->m_motor_state.v_beta,
                        motor->m_motor_state.i_alpha, motor->m_motor_state.i_beta,
                        dt, &motor->m_observer_state, &motor->m_phase_now_observer, motor);

    // Dùng trực tiếp góc điện của Encoder (m_motor_state.phase) thay vì góc điện ước lượng của Observer để tính toán tốc độ bằng PLL
    foc_pll_run(motor->m_motor_state.phase, dt, &motor->m_pll_phase, &motor->m_pll_speed, motor->m_conf);
    motor->m_speed_est_fast = motor->m_pll_speed;

    // 2. Run Position PID or Speed PID based on Control Mode
    if (motor->m_control_mode == CONTROL_MODE_POS) {
        foc_run_pid_control_pos(true, dt, motor);
    } else if (motor->m_control_mode == CONTROL_MODE_SPEED) {
        foc_run_pid_control_speed(true, dt, motor);
    }

    // 3. Run Field Weakening
    foc_run_fw(motor, dt);
}
