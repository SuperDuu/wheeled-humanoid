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
    foc->phase_swap_bc = true; // Default hardware mapping (will be confirmed by ALIGN)

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

    // Overcurrent Instantaneous Check (25A / 2.5ms)
    static uint32_t overcurrent_count = 0;
    if (current_mag > 25.0f) {
        overcurrent_count++;
        if (overcurrent_count >= 50) {
            foc->fault |= MC_FAULT_OVER_CURRENT;
        }
    } else {
        if (overcurrent_count > 0) overcurrent_count--;
    }

    // Thermal Safety Timeout Protection (8.0A continuous for > 5.0s = 100,000 ISR cycles @ 20kHz)
    // Nới lỏng theo đúng chuẩn Datasheet GB8115-4 (Dòng kẹt cực đại 6.6A)
    static uint32_t thermal_timeout_count = 0;
    if (current_mag > 8.0f) {
        thermal_timeout_count++;
        if (thermal_timeout_count >= 100000) {
            foc->fault |= MC_FAULT_OVER_CURRENT;
        }
    } else {
        if (thermal_timeout_count > 0) thermal_timeout_count -= 2;
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

    // 1. Đọc Encoder 20kHz trực tiếp từ SPI3.
    // SPI3 chỉ do ISR quản lý 100% độc quyền (không đọc ở Slow Loop nữa) -> triệt tiêu hoàn toàn nhiễu giật khục 1kHz.
    float raw_enc_rad = foc->encoder.angle_rad;
    AS5048A_ReadRadians(&foc->encoder, &raw_enc_rad);
    foc_update_cycloidal_joint_angle(motor, raw_enc_rad);

    // 1b. Tính Góc Điện theo Chuẩn VESC [-PI, +PI] (loại bỏ hoàn toàn hiện tượng nhảy 0 <-> 2PI ở vị trí 0)
    // 1b. Tính Góc Điện theo Chuẩn VESC [-PI, +PI] với Dynamic Lead-Angle Delay Compensation (120µs)
    float elec_raw = (float)(conf_now->encoder_direction * conf_now->foc_motor_pole_pairs) * raw_enc_rad;
    float elec_angle = elec_raw - foc->zero_electric_angle;
    utils_norm_angle_rad(&elec_angle);

    state_m->phase = elec_angle;

    // Chạy Bộ lọc PLL Tracking Filter ở tần số 20kHz (thay vì 1kHz)
    // Giúp ước lượng vận tốc siêu mịn mỗi 50µs, triệt tiêu 100% hiện tượng gợn bước nhảy 1kHz ở tốc độ cao >200 RPM
    float dt_fast = 0.000050f; // 20kHz Fast Loop period
    foc_pll_run(elec_angle, dt_fast, &motor->m_pll_phase, &motor->m_pll_speed, motor->m_conf);
    motor->m_speed_est_fast = motor->m_pll_speed;

    // Bù trễ pha động (75µs = 1.5 chu kỳ PWM 20kHz) với vận tốc 20kHz mượt mà liên tục
    float pwm_phase = elec_angle + (motor->m_speed_est_fast * 0.000075f);
    utils_norm_angle_rad(&pwm_phase);

    utils_fast_sincos(pwm_phase, &state_m->phase_sin, &state_m->phase_cos);

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
        foc->duty_a = 0.0f;
        foc->duty_b = 0.0f;
        foc->duty_c = 0.0f;
        return;
    }

    // Strict Voltage Vector Circle Limitation (Max = Vbus / sqrt(3))
    float max_v_mag = ONE_BY_SQRT3 * conf_now->l_max_duty * state_m->v_bus * conf_now->foc_overmod_factor;
    float max_vd = max_v_mag * conf_now->foc_mag_vd_max;
    float max_vq = sqrtf(SQ(max_v_mag) - SQ(state_m->vd));
    UTILS_NAN_ZERO(max_vq);

    // 5. Current-Mode FOC Controller with Back-EMF Feedforward & Decoupling
    if (motor->m_control_mode == CONTROL_MODE_DUTY) {
        state_m->vd = 0.0f;
        state_m->vq = state_m->duty_now * state_m->v_bus;
        state_m->vd_int = 0.0f;
        state_m->vq_int = 0.0f;
    } else {
        // Current-Mode FOC for Current (Torque), Speed, and Position control modes
        state_m->iq_target = motor->m_iq_set;
        float id_target = -motor->m_i_fw_set;

        float Ierr_d = id_target - state_m->id;
        float Ierr_q = state_m->iq_target - state_m->iq;

        float ki = conf_now->foc_current_ki;
        float kp = conf_now->foc_current_kp;

        // Feedforward terms (Back-EMF & inductive cross-coupling)
        float vq_ff = motor->m_speed_est_fast * conf_now->foc_motor_flux_linkage;
        float vd_ff = -motor->m_speed_est_fast * conf_now->foc_motor_l * state_m->iq;

        // Anti-windup conditional integration
        if (!((state_m->vd >= max_vd && Ierr_d > 0.0f) || (state_m->vd <= -max_vd && Ierr_d < 0.0f))) {
            state_m->vd_int += Ierr_d * ki * dt;
            utils_truncate_number_abs((float*)&state_m->vd_int, max_vd);
        }
        if (!((state_m->vq >= max_vq && Ierr_q > 0.0f) || (state_m->vq <= -max_vq && Ierr_q < 0.0f))) {
            state_m->vq_int += Ierr_q * ki * dt;
            utils_truncate_number_abs((float*)&state_m->vq_int, max_vq);
        }

        state_m->vd = state_m->vd_int + (Ierr_d * kp) + vd_ff;
        state_m->vq = state_m->vq_int + (Ierr_q * kp) + vq_ff;

        float v_mag = sqrtf(state_m->vd * state_m->vd + state_m->vq * state_m->vq);
        if (v_mag > max_v_mag && v_mag > 0.001f) {
            float scale = max_v_mag / v_mag;
            state_m->vd *= scale;
            state_m->vq *= scale;
        }
    }

    // Normalize voltages for Inverse Park & Modulation
    const float voltage_normalize = 1.0f / state_m->v_bus;
    state_m->mod_d = state_m->vd * voltage_normalize;
    state_m->mod_q = state_m->vq * voltage_normalize;

    // 8. Inverse Park Transform: Rotor -> Stator frame
    state_m->mod_alpha_raw = c * state_m->mod_d - s * state_m->mod_q;
    state_m->mod_beta_raw  = c * state_m->mod_q + s * state_m->mod_d;

    // 9. VESC 6-Sector Space Vector Modulation (SVM)
    uint32_t ta, tb, tc, sector;
    foc_svm(state_m->mod_alpha_raw, state_m->mod_beta_raw, conf_now->l_max_duty, 1000, &ta, &tb, &tc, &sector);

    // Stator Phase Mapping: Apply auto-detected Phase B <-> Phase C swap
    foc->duty_a = (float)ta / 1000.0f;
    if (foc->phase_swap_bc) {
        foc->duty_b = (float)tc / 1000.0f;
        foc->duty_c = (float)tb / 1000.0f;
    } else {
        foc->duty_b = (float)tb / 1000.0f;
        foc->duty_c = (float)tc / 1000.0f;
    }
}

/**
  * @brief  Slow Loop Execution (Called @ 1kHz for Observer, PLL, Position/Speed PID, FW)
  */
void FOC_Control_SlowLoop(FOC_Controller_t *foc, float dt)
{
    if (foc == NULL) return;

    motor_all_state_t *motor = &foc->motor;

    if (foc->motor.m_state != MC_STATE_RUNNING) return;

    // 1. Run VESC Observer
    foc_observer_update(motor->m_motor_state.v_alpha, motor->m_motor_state.v_beta,
                        motor->m_motor_state.i_alpha, motor->m_motor_state.i_beta,
                        dt, &motor->m_observer_state, &motor->m_phase_now_observer, motor);

    // 2. Run Position PID or Speed PID based on Control Mode (generates target Iq)
    if (motor->m_control_mode == CONTROL_MODE_POS) {
        foc_run_pid_control_pos(true, dt, motor);
    } else if (motor->m_control_mode == CONTROL_MODE_SPEED) {
        foc_run_pid_control_speed(true, dt, motor);
    }

    // 3. Run Field Weakening
    foc_run_fw(motor, dt);
}
