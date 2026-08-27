/**
  ******************************************************************************
  * @file    comm_telemetry.c
  * @brief   High-speed Real-Time Telemetry & Command Protocol implementation
  *          Supports both Native USB CDC (Virtual COM Port) and USART1 UART.
  ******************************************************************************
  */

#include "comm_telemetry.h"
#include "main.h"
#include "vesc_utils.h"
#include "foc_math.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
<<<<<<< HEAD
#include <ctype.h>
=======
>>>>>>> 8e44a795456836680c75c6d0526c6dd48d62f00d

/* USB Device Handle */
extern USBD_HandleTypeDef hUsbDeviceFS;

/* Global references */
static uint32_t s_last_telemetry_tx_ms = 0;
static char s_rx_cmd_buffer[64];
static uint8_t s_rx_cmd_idx = 0;

<<<<<<< HEAD
/* Open-Loop Test Run Control Globals */
volatile uint8_t run_open_loop = 0;
volatile float open_loop_target_rpm = 100.0f;
volatile float open_loop_current_rpm = 0.0f;
volatile float open_loop_angle = 0.0f;
volatile float open_loop_voltage = 9.0f; // 9.0V (~2.3A) - High torque for 1:17 Cycloid gearbox load

=======
>>>>>>> 8e44a795456836680c75c6d0526c6dd48d62f00d
/* Checksum calculation */
static uint16_t CalculateChecksum(const uint8_t *data, uint16_t len)
{
    uint16_t sum = 0;
    for (uint16_t i = 0; i < len; i++) {
        sum += data[i];
    }
    return sum;
}

/**
  * @brief  Initialize telemetry communication (Native USB CDC)
  */
void Comm_Telemetry_Init(void)
{
    s_last_telemetry_tx_ms = HAL_GetTick();
    s_rx_cmd_idx = 0;
}

/**
  * @brief  Transmit high-speed binary telemetry frame over Native USB CDC
  */
<<<<<<< HEAD
bool Comm_Telemetry_Send(FOC_Controller_t *foc)
{
    if (foc == NULL) return false;
=======
void Comm_Telemetry_Send(FOC_Controller_t *foc)
{
    if (foc == NULL) return;
>>>>>>> 8e44a795456836680c75c6d0526c6dd48d62f00d

    motor_all_state_t *motor = &foc->motor;
    motor_state_t *state_m = &motor->m_motor_state;
    mc_configuration *conf = motor->m_conf;

<<<<<<< HEAD
    /* CRITICAL: packet PHẢI là static vì CDC_Transmit_FS chỉ lưu CON TRỎ.
     * Packet 78 bytes > 64 bytes (USB FS max packet) → cần 2 USB transactions.
     * Transaction thứ 2 (14 bytes cuối) xảy ra trong USB IRQ SAU KHI hàm return.
     * Nếu packet trên stack → pointer trỏ vào rác → 14 bytes cuối corrupt → checksum fail 95%. */
    static telemetry_packet_t packet;
=======
    telemetry_packet_t packet;
>>>>>>> 8e44a795456836680c75c6d0526c6dd48d62f00d
    memset(&packet, 0, sizeof(packet));

    packet.magic1 = TELEMETRY_MAGIC_BYTE1;
    packet.magic2 = TELEMETRY_MAGIC_BYTE2;
    packet.packet_type = TELEMETRY_PACKET_TYPE;
    packet.payload_len = (uint8_t)(sizeof(telemetry_packet_t) - 4); // Exclude header (4 bytes)
    packet.timestamp_ms = HAL_GetTick();

    // 1. Calculate 3-Phase Currents (Amperes)
    // i_alpha = Ia, i_beta = (Ia + 2*Ib)/sqrt(3)
    // Therefore: Ia = i_alpha, Ib = (sqrt(3)*i_beta - i_alpha)/2, Ic = -Ia - Ib
    float ia = state_m->i_alpha;
    float ib = ((float)SQRT3_BY_2 * state_m->i_beta) - (0.5f * ia);
    float ic = -ia - ib;

    packet.i_a = ia;
    packet.i_b = ib;
    packet.i_c = ic;

<<<<<<< HEAD
    // 2. FOC Vector Currents (Filtered DC for smooth telemetry display)
    packet.i_d = state_m->id_filter;
    packet.i_q = state_m->iq_filter;
=======
    // 2. FOC Vector Currents
    packet.i_d = state_m->id;
    packet.i_q = state_m->iq;
>>>>>>> 8e44a795456836680c75c6d0526c6dd48d62f00d
    packet.i_q_target = state_m->iq_target;

    // 3. 3-Phase PWM Duty Cycles
    packet.duty_a = foc->duty_a;
    packet.duty_b = foc->duty_b;
    packet.duty_c = foc->duty_c;

    // 4. Angles
    packet.phase_elec = state_m->phase;
    packet.mech_angle = motor->m_mech_angle_single;
    packet.joint_angle = motor->m_joint_angle;

    // 5. Speeds (Mechanical RPM)
    float pole_pairs = (conf != NULL && conf->foc_motor_pole_pairs > 0) ? (float)conf->foc_motor_pole_pairs : 21.0f;
    float erpm = RADPS2RPM_f(motor->m_speed_est_fast);
    packet.speed_rpm = erpm / pole_pairs;
    packet.speed_target_rpm = motor->m_speed_command_rpm / pole_pairs;

    // 6. System Status
    extern volatile ADC_Readings_t g_adc_readings;
    packet.v_bus = (g_adc_readings.vbus > 5.0f) ? g_adc_readings.vbus : ((state_m->v_bus > 5.0f) ? state_m->v_bus : 24.0f);
    packet.temp_fet = 25.0f;
    packet.control_mode = (uint8_t)motor->m_control_mode;
    packet.motor_state = (uint8_t)motor->m_state;
    packet.fault_code = (uint8_t)foc->fault;
<<<<<<< HEAD
    packet.encoder_dir = (int8_t)conf->encoder_direction;

    // FOC Diagnostic Fields
    packet.vd = state_m->vd;
    packet.vq = state_m->vq;
    packet.zero_elec_angle = foc->zero_electric_angle;
    packet.id_target = state_m->id_target;
=======
    packet.reserved = 0;
>>>>>>> 8e44a795456836680c75c6d0526c6dd48d62f00d

    // Calculate Checksum over payload (excluding magic & checksum itself)
    uint8_t *raw_buf = (uint8_t*)&packet;
    packet.checksum = CalculateChecksum(&raw_buf[4], sizeof(telemetry_packet_t) - 6);

    // Transmit via Native USB CDC (Virtual COM Port /dev/ttyACM*)
    if (hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED && hUsbDeviceFS.pClassData != NULL) {
        USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData;
        if (hcdc->TxState == 0) {
            CDC_Transmit_FS((uint8_t*)&packet, sizeof(packet));
<<<<<<< HEAD
            return true;
        }
    }
    return false;
=======
        }
    }
>>>>>>> 8e44a795456836680c75c6d0526c6dd48d62f00d
}

/**
  * @brief  Parse incoming commands from desktop app / terminal
  * Commands:
  *   MODE <0..4>    (0:Off, 1:Current, 2:Brake, 3:Speed, 4:Pos)
  *   SPEED <rpm>    (e.g., SPEED 200)
  *   IQ <amps>      (e.g., IQ 1.5)
  *   POS <rad>      (e.g., POS 1.57)
  *   STOP           (Emergency stop)
  *   ALIGN          (Run encoder alignment)
  */
extern volatile int run_foc_mode;
extern volatile float speed_target_dbg;
extern volatile float iq_target_dbg;
extern volatile float pos_target_dbg;

static void ProcessCommand(FOC_Controller_t *foc, char *cmd)
{
    if (foc == NULL || cmd == NULL) return;
    motor_all_state_t *motor = &foc->motor;

    // Strip trailing newline / carriage return
    char *p = cmd;
    while (*p) {
        if (*p == '\r' || *p == '\n') {
            *p = '\0';
            break;
        }
        p++;
    }

<<<<<<< HEAD
    if (strncmp(cmd, "STOP", 4) == 0 || strncmp(cmd, "OFF", 3) == 0) {
=======
    if (strncmp(cmd, "STOP", 4) == 0) {
>>>>>>> 8e44a795456836680c75c6d0526c6dd48d62f00d
        motor->m_state = MC_STATE_OFF;
        motor->m_iq_set = 0.0f;
        motor->m_speed_command_rpm = 0.0f;
        motor->m_speed_pid_set_rpm = 0.0f;
<<<<<<< HEAD
        motor->m_speed_i_term = 0.0f;
        motor->m_motor_state.duty_now = 0.0f;
        motor->m_motor_state.vd = 0.0f;
        motor->m_motor_state.vq = 0.0f;
        foc->fault = MC_FAULT_NONE;
        speed_target_dbg = 0.0f;
        iq_target_dbg = 0.0f;
        run_foc_mode = 0;
        run_open_loop = 0;
=======
        motor->m_pos_pid_set = motor->m_joint_angle;
        run_foc_mode = 0;
        speed_target_dbg = 0.0f;
        iq_target_dbg = 0.0f;
>>>>>>> 8e44a795456836680c75c6d0526c6dd48d62f00d
    }
    else if (strncmp(cmd, "ALIGN", 5) == 0) {
        extern volatile int run_alignment;
        run_alignment = 1;
<<<<<<< HEAD
        run_open_loop = 0;
        run_foc_mode = 0;
    }
    else if (strncmp(cmd, "OPENLOOP", 8) == 0 || strncmp(cmd, "TEST", 4) == 0 || strncmp(cmd, "RUN", 3) == 0) {
        float rpm = 100.0f;
        float v_custom = 0.0f;
        const char *arg = (strncmp(cmd, "OPENLOOP", 8) == 0) ? &cmd[8] :
                          (strncmp(cmd, "TEST", 4) == 0) ? &cmd[4] : &cmd[3];
        while (*arg == ' ') arg++;
        if (*arg != '\0') {
            int n = sscanf(arg, "%f %f", &rpm, &v_custom);
            if (n >= 2 && v_custom > 0.5f) {
                open_loop_voltage = v_custom;
            }
        }
        open_loop_target_rpm = rpm;
        open_loop_current_rpm = 0.0f;
        run_open_loop = 1;
        motor->m_state = MC_STATE_RUNNING;
        foc->fault = MC_FAULT_NONE;
        TIM1_EnsureMoeEnabled();
    }
    else if (strncmp(cmd, "LOCK_ANGLE ", 11) == 0 || strncmp(cmd, "LOCK ", 5) == 0) {
        float angle = 0.0f;
        float volt = 4.0f;
        const char *arg = (strncmp(cmd, "LOCK_ANGLE ", 11) == 0) ? &cmd[11] : &cmd[5];
        sscanf(arg, "%f %f", &angle, &volt);
        open_loop_angle = angle;
        open_loop_voltage = (volt > 0.5f) ? volt : 4.0f;
        open_loop_current_rpm = 0.0f;
        open_loop_target_rpm = 0.0f;
        run_open_loop = 1;
        motor->m_state = MC_STATE_RUNNING;
        foc->fault = MC_FAULT_NONE;
        TIM1_EnsureMoeEnabled();
    }
    else if (strncmp(cmd, "V_OPEN ", 7) == 0 || strncmp(cmd, "VOPEN ", 6) == 0) {
        float v = atof((strncmp(cmd, "V_OPEN ", 7) == 0) ? &cmd[7] : &cmd[6]);
        if (v >= 1.0f && v <= 20.0f) {
            open_loop_voltage = v;
        }
    }
    else if (strncmp(cmd, "MODE ", 5) == 0) {
        int m = atoi(&cmd[5]);
        run_open_loop = 0;
        foc->fault = MC_FAULT_NONE;
=======
    }
    else if (strncmp(cmd, "MODE ", 5) == 0) {
        int m = atoi(&cmd[5]);
>>>>>>> 8e44a795456836680c75c6d0526c6dd48d62f00d
        if (m == 0) {
            motor->m_state = MC_STATE_OFF;
            motor->m_iq_set = 0.0f;
            motor->m_speed_command_rpm = 0.0f;
<<<<<<< HEAD
            motor->m_speed_pid_set_rpm = 0.0f;
            motor->m_speed_i_term = 0.0f;
            motor->m_motor_state.duty_now = 0.0f;
            speed_target_dbg = 0.0f;
            iq_target_dbg = 0.0f;
            run_foc_mode = 0;
        } else if (m >= 1 && m <= 5) {
            motor->m_state = MC_STATE_RUNNING;
            TIM1_EnsureMoeEnabled();
=======
            run_foc_mode = 0;
        } else if (m >= 1 && m <= 4) {
            motor->m_state = MC_STATE_RUNNING;
>>>>>>> 8e44a795456836680c75c6d0526c6dd48d62f00d
            if (m == 1) { motor->m_control_mode = CONTROL_MODE_CURRENT; run_foc_mode = 1; }
            else if (m == 2) { motor->m_control_mode = CONTROL_MODE_CURRENT_BRAKE; run_foc_mode = 1; }
            else if (m == 3) { motor->m_control_mode = CONTROL_MODE_SPEED; run_foc_mode = 3; }
            else if (m == 4) { motor->m_control_mode = CONTROL_MODE_POS; run_foc_mode = 2; }
<<<<<<< HEAD
            else if (m == 5) { motor->m_control_mode = CONTROL_MODE_DUTY; run_foc_mode = 4; }
        }
    }
    else if (strncmp(cmd, "CLOSELOOP", 9) == 0 || strncmp(cmd, "CLOSE_LOOP", 10) == 0 || strcmp(cmd, "START") == 0) {
        float rpm = 100.0f;
        if (strncmp(cmd, "CLOSELOOP ", 10) == 0) rpm = atof(&cmd[10]);
        else if (strncmp(cmd, "CLOSE_LOOP ", 11) == 0) rpm = atof(&cmd[11]);
        speed_target_dbg = rpm;
        motor->m_speed_command_rpm = rpm * 21.0f;
        motor->m_speed_pid_set_rpm = RADPS2RPM_f(motor->m_speed_est_fast);
        motor->m_speed_d_filter = motor->m_speed_pid_set_rpm;
        motor->m_speed_i_term = 0.0f;
        motor->m_speed_prev_error = 0.0f;
        motor->m_speed_d_filter_proc = 0.0f;
        motor->m_control_mode = CONTROL_MODE_SPEED;
        motor->m_state = MC_STATE_RUNNING;
        run_foc_mode = 3;
        foc->fault = MC_FAULT_NONE;
        TIM1_EnsureMoeEnabled();
    }
=======
        }
    }
>>>>>>> 8e44a795456836680c75c6d0526c6dd48d62f00d
    else if (strncmp(cmd, "SPEED ", 6) == 0) {
        float mech_rpm = atof(&cmd[6]);
        float pole_pairs = (motor->m_conf != NULL) ? (float)motor->m_conf->foc_motor_pole_pairs : 21.0f;
        speed_target_dbg = mech_rpm;
<<<<<<< HEAD
        run_open_loop = 0;
        motor->m_speed_command_rpm = mech_rpm * pole_pairs;
        motor->m_speed_pid_set_rpm = RADPS2RPM_f(motor->m_speed_est_fast);
        motor->m_speed_d_filter = motor->m_speed_pid_set_rpm;
        motor->m_speed_i_term = 0.0f;
        motor->m_speed_prev_error = 0.0f;
        motor->m_speed_d_filter_proc = 0.0f;
        motor->m_control_mode = CONTROL_MODE_SPEED;
        motor->m_state = MC_STATE_RUNNING;
        run_foc_mode = 3;
        foc->fault = MC_FAULT_NONE;
        TIM1_EnsureMoeEnabled();
    }
    else if (strncmp(cmd, "IQ ", 3) == 0 || strncmp(cmd, "CURRENT ", 8) == 0 || strncmp(cmd, "TORQUE ", 7) == 0 || strncmp(cmd, "FORCE ", 6) == 0) {
        float iq = 0.0f;
        if (strncmp(cmd, "IQ ", 3) == 0) iq = atof(&cmd[3]);
        else if (strncmp(cmd, "CURRENT ", 8) == 0) iq = atof(&cmd[8]);
        else if (strncmp(cmd, "FORCE ", 6) == 0) iq = atof(&cmd[6]);
        else if (strncmp(cmd, "TORQUE ", 7) == 0) {
            float tau = atof(&cmd[7]); // Nm
            float kt = (motor->m_conf != NULL && motor->m_conf->foc_motor_flux_linkage > 0.0f) ? (1.5f * 21.0f * motor->m_conf->foc_motor_flux_linkage) : 0.67f;
            iq = (kt > 0.01f) ? (tau / kt) : tau;
        }
        iq_target_dbg = iq;
        motor->m_iq_set = iq;
        // BUMPLESS TRANSFER: Initialize PI integrators to current voltage output
        // so that the output doesn't jump on mode switch (prevents motor jerk).
        // In DUTY mode, vd=0 and vq=duty*vbus. Starting the integrator at the
        // current vq ensures continuity: first-cycle output ≈ vq_prev + small_correction.
        motor->m_motor_state.vq_int = motor->m_motor_state.vq;
        motor->m_motor_state.vd_int = motor->m_motor_state.vd;
        motor->m_control_mode = CONTROL_MODE_CURRENT;
        motor->m_state = MC_STATE_RUNNING;
        run_foc_mode = 1;
        foc->fault = MC_FAULT_NONE;
        TIM1_EnsureMoeEnabled();
    }
    else if (strncmp(cmd, "VQ ", 3) == 0 || strncmp(cmd, "VOLT ", 5) == 0) {
        float vq = atof((strncmp(cmd, "VQ ", 3) == 0) ? &cmd[3] : &cmd[5]);
        extern volatile ADC_Readings_t g_adc_readings;
        float vbus = (g_adc_readings.vbus > 5.0f) ? g_adc_readings.vbus : 24.0f;
        motor->m_motor_state.duty_now = vq / vbus;
        motor->m_control_mode = CONTROL_MODE_DUTY;
        motor->m_state = MC_STATE_RUNNING;
        run_foc_mode = 4;
        foc->fault = MC_FAULT_NONE;
        TIM1_EnsureMoeEnabled();
    }
    else if (strncmp(cmd, "SETHOME", 7) == 0 || strncmp(cmd, "SET_HOME", 8) == 0 || strncmp(cmd, "ZERO", 4) == 0) {
        // CĂN CHỈNH VỊ TRÍ HIỆN TẠI LÀM HOME (0.0 ĐỘ)
        foc_set_home_position(motor);
        pos_target_dbg = 0.0f;
    }
    else if (strcmp(cmd, "RESETERR") == 0 || strcmp(cmd, "RESET_ERR") == 0 || strcmp(cmd, "CLEAR_ERR") == 0 || strcmp(cmd, "RESET_PID") == 0 || strcmp(cmd, "CLEAR") == 0) {
        // RESET TOÀN BỘ SAI SỐ VÀ TÍCH PHÂN PID VỀ 0
        motor->m_speed_i_term = 0.0f;
        motor->m_speed_prev_error = 0.0f;
        motor->m_speed_d_filter = 0.0f;
        motor->m_speed_d_filter_proc = 0.0f;
        motor->m_pos_i_term = 0.0f;
        motor->m_pos_prev_error = 0.0f;
        motor->m_pos_d_filter = 0.0f;
        motor->m_motor_state.vd_int = 0.0f;
        motor->m_motor_state.vq_int = 0.0f;
        motor->m_motor_state.vd = 0.0f;
        motor->m_motor_state.vq = 0.0f;
        motor->m_iq_set = 0.0f;
        foc->fault = MC_FAULT_NONE;
        if (motor->m_control_mode == CONTROL_MODE_POS) {
            motor->m_pos_pid_set = motor->m_joint_angle;
            motor->m_traj_active = false;
            pos_target_dbg = motor->m_joint_angle;
        }
    }
    else if (strncmp(cmd, "GOHOME", 6) == 0 || strncmp(cmd, "HOME", 4) == 0) {
        // QUAY VỀ VỊ TRÍ HOME (0.0 ĐỘ)
        float duration_s = 1.0f;
        if (strncmp(cmd, "GOHOME ", 7) == 0) duration_s = atof(&cmd[7]);
        foc_start_trajectory(motor, 0.0f, duration_s, 3.0f);
        pos_target_dbg = 0.0f;
        run_foc_mode = 2;
        foc->fault = MC_FAULT_NONE;
        TIM1_EnsureMoeEnabled();
    }
    else if (strncmp(cmd, "MOVE ", 5) == 0 || (isdigit((unsigned char)cmd[0]) || (cmd[0] == '-' && isdigit((unsigned char)cmd[1])))) {
        // KHUNG TRUYỀN TOÀN DIỆN CHO CÁNH TAY ROBOT:
        // Cú pháp 1: MOVE <Góc_Target_Độ> <Thời_Gian_s> [Lực_Ghim_A_hoặc_Nm] (Vd: MOVE 90 5 3.0)
        // Cú pháp 2: <Góc_Target_Độ> <Thời_Gian_s> [Lực_Ghim_A_hoặc_Nm]     (Vd: 90 5 150 hoặc 90 5 3.0)
        const char *p = (strncmp(cmd, "MOVE ", 5) == 0) ? &cmd[5] : cmd;
        float target_deg = 0.0f;
        float duration_s = 1.0f;       // Mặc định 1.0 giây
        float hold_limit = 3.0f;       // Mặc định 3.0A
        int count = sscanf(p, "%f %f %f", &target_deg, &duration_s, &hold_limit);

        if (count >= 1) {
            // Nếu người dùng nhập lực dạng Nm lớn (> 15Nm) -> quy đổi sang dòng Ampe (I = Tau / (Kt * Gear))
            float current_a = hold_limit;
            if (hold_limit > 15.0f) {
                // 150 Nm tại đầu ra hộp số 1:17 -> Mô-men động cơ = 150 / 17 = 8.8 Nm -> Dòng điện = 8.8 / 0.67 = 13.1A (kẹp an toàn 5.0A)
                current_a = (hold_limit / 17.0f) / 0.67f;
                if (current_a > 6.0f) current_a = 6.0f; // Kẹp dòng an toàn 6A
            }
            if (duration_s < 0.05f) duration_s = 0.05f;

            float target_rad = DEG2RAD_f(target_deg);
            foc_start_trajectory(motor, target_rad, duration_s, current_a);
            pos_target_dbg = target_rad;
            run_foc_mode = 2;
            foc->fault = MC_FAULT_NONE;
            TIM1_EnsureMoeEnabled();
        }
    }
    else if (strncmp(cmd, "REL ", 4) == 0 || strncmp(cmd, "STEP ", 5) == 0) {
        // QUAY TƯƠNG ĐỐI (VD: REL 30 quay tới thêm 30 độ, REL -30 quay lùi 30 độ)
        float rel_deg = atof((strncmp(cmd, "REL ", 4) == 0) ? &cmd[4] : &cmd[5]);
        float target_rad = motor->m_joint_angle + DEG2RAD_f(rel_deg);
        foc_start_trajectory(motor, target_rad, 1.2f, 4.0f);
        pos_target_dbg = target_rad;
        run_foc_mode = 2;
        run_open_loop = 0;
        foc->fault = MC_FAULT_NONE;
        TIM1_EnsureMoeEnabled();
    }
    else if (strncmp(cmd, "POS ", 4) == 0 || strncmp(cmd, "ANGLE ", 6) == 0) {
        float pos_deg = atof((strncmp(cmd, "POS ", 4) == 0) ? &cmd[4] : &cmd[6]);
        float pos_rad = DEG2RAD_f(pos_deg);
        foc_start_trajectory(motor, pos_rad, 0.0f, 4.0f); // 0.0s = Tốc độ tối đa tức thì
        pos_target_dbg = pos_rad;
        run_foc_mode = 2;
        run_open_loop = 0;
        foc->fault = MC_FAULT_NONE;
        TIM1_EnsureMoeEnabled();
    }
    else if (strncmp(cmd, "SLOT ", 5) == 0) {
        // 12 VỊ TRÍ GHIM KHỚP TAY ROBOT (1 đến 12 cách nhau 30 độ)
        int slot = atoi(&cmd[5]);
        if (slot >= 1 && slot <= 12) {
            float slot_angles_deg[12] = {0.0f, 30.0f, 60.0f, 90.0f, 120.0f, 150.0f, 180.0f, -150.0f, -120.0f, -90.0f, -60.0f, -30.0f};
            float target_rad = DEG2RAD_f(slot_angles_deg[slot - 1]);
            foc_start_trajectory(motor, target_rad, 0.0f, 4.0f); // Tốc độ tối đa tức thì
            pos_target_dbg = target_rad;
            run_foc_mode = 2;
            foc->fault = MC_FAULT_NONE;
            TIM1_EnsureMoeEnabled();
        }
    }
    else if (strcmp(cmd, "HOLD") == 0 || strcmp(cmd, "LOCK") == 0) {
        // GHIM GÓC TỨC THÌ TẠI VỊ TRÍ HIỆN TẠI
        float current_angle = motor->m_joint_angle;
        motor->m_traj_active = false;
        pos_target_dbg = current_angle;
        motor->m_pos_pid_set = current_angle;
        motor->m_control_mode = CONTROL_MODE_POS;
        motor->m_state = MC_STATE_RUNNING;
        run_foc_mode = 2;
        foc->fault = MC_FAULT_NONE;
        TIM1_EnsureMoeEnabled();
    }
    else if (strcmp(cmd, "FREE") == 0 || strcmp(cmd, "RELEASE") == 0) {
        // NHẢ LỰC ĐỂ XOAY TAY TỰ DO
        motor->m_state = MC_STATE_OFF;
        run_foc_mode = 0;
        motor->m_iq_set = 0.0f;
    }
    else if (strcmp(cmd, "RESET") == 0 || strcmp(cmd, "REBOOT") == 0) {
        // RESET VI ĐIỀU KHIỂN & DRIVER
        motor->m_state = MC_STATE_OFF;
        run_foc_mode = 0;
        motor->m_iq_set = 0.0f;
        foc->fault = MC_FAULT_NONE;
        NVIC_SystemReset();
    }
    else if (strcmp(cmd, "CLEAR") == 0 || strcmp(cmd, "CLEAR_FAULT") == 0) {
        foc->fault = MC_FAULT_NONE;
        motor->m_state = MC_STATE_OFF;
        run_foc_mode = 0;
        TIM1_EnsureMoeEnabled();
    }
    else if (strncmp(cmd, "DIR ", 4) == 0) {
        int dir = atoi(&cmd[4]);
        if (dir == 1 || dir == -1) {
            foc->conf.encoder_direction = dir;
            if (motor->m_conf != NULL) motor->m_conf->encoder_direction = dir;
        }
    }
    else if (strncmp(cmd, "SWAPBC ", 7) == 0 || strncmp(cmd, "SWAP ", 5) == 0) {
        int swap = atoi((strncmp(cmd, "SWAPBC ", 7) == 0) ? &cmd[7] : &cmd[5]);
        foc->phase_swap_bc = (swap != 0);
    }
    else if (strncmp(cmd, "OFFSET ", 7) == 0) {
        float off = atof(&cmd[7]);
        foc->zero_electric_angle = off;
    }
    else if (strncmp(cmd, "SET_SPEED_PID ", 14) == 0 || strncmp(cmd, "SPID ", 5) == 0) {
        float kp = 0.0015f, ki = 0.0010f, ramp = 3000.0f;
        const char *arg = (strncmp(cmd, "SET_SPEED_PID ", 14) == 0) ? &cmd[14] : &cmd[5];
        if (sscanf(arg, "%f %f %f", &kp, &ki, &ramp) >= 2) {
            foc->conf.s_pid_kp = kp;
            foc->conf.s_pid_ki = ki;
            if (ramp > 0.0f) foc->conf.s_pid_ramp_erpms_s = ramp;
            if (motor->m_conf != NULL) {
                motor->m_conf->s_pid_kp = kp;
                motor->m_conf->s_pid_ki = ki;
                if (ramp > 0.0f) motor->m_conf->s_pid_ramp_erpms_s = ramp;
            }
        }
    }
    else if (strncmp(cmd, "SET_POS_PID ", 12) == 0 || strncmp(cmd, "PPID ", 5) == 0) {
        float kp = 20.0f, kd = 0.10f;
        const char *arg = (strncmp(cmd, "SET_POS_PID ", 12) == 0) ? &cmd[12] : &cmd[5];
        if (sscanf(arg, "%f %f", &kp, &kd) >= 2) {
            foc->conf.p_pid_kp = kp;
            foc->conf.p_pid_kd = kd;
            if (motor->m_conf != NULL) {
                motor->m_conf->p_pid_kp = kp;
                motor->m_conf->p_pid_kd = kd;
            }
        }
    }
    else if (strncmp(cmd, "KP_S ", 5) == 0 || strncmp(cmd, "SET_SKP ", 8) == 0) {
        float val = atof((strncmp(cmd, "KP_S ", 5) == 0) ? &cmd[5] : &cmd[8]);
        foc->conf.s_pid_kp = val;
        if (motor->m_conf != NULL) motor->m_conf->s_pid_kp = val;
    }
    else if (strncmp(cmd, "KI_S ", 5) == 0 || strncmp(cmd, "SET_SKI ", 8) == 0) {
        float val = atof((strncmp(cmd, "KI_S ", 5) == 0) ? &cmd[5] : &cmd[8]);
        foc->conf.s_pid_ki = val;
        if (motor->m_conf != NULL) motor->m_conf->s_pid_ki = val;
    }
    else if (strncmp(cmd, "FLUX ", 5) == 0 || strncmp(cmd, "LAMBDA ", 7) == 0) {
        float val = atof((strncmp(cmd, "FLUX ", 5) == 0) ? &cmd[5] : &cmd[7]);
        if (val > 0.001f && val < 0.1f) {
            foc->conf.foc_motor_flux_linkage = val;
            if (motor->m_conf != NULL) motor->m_conf->foc_motor_flux_linkage = val;
        }
    }
    else if (strncmp(cmd, "KD_S ", 5) == 0 || strncmp(cmd, "SET_SKD ", 8) == 0) {
        float val = atof((strncmp(cmd, "KD_S ", 5) == 0) ? &cmd[5] : &cmd[8]);
        foc->conf.s_pid_kd = val;
        if (motor->m_conf != NULL) motor->m_conf->s_pid_kd = val;
    }
    else if (strncmp(cmd, "KP_P ", 5) == 0 || strncmp(cmd, "SET_PKP ", 8) == 0) {
        float val = atof((strncmp(cmd, "KP_P ", 5) == 0) ? &cmd[5] : &cmd[8]);
        foc->conf.p_pid_kp = val;
        if (motor->m_conf != NULL) motor->m_conf->p_pid_kp = val;
    }
    else if (strncmp(cmd, "KI_P ", 5) == 0 || strncmp(cmd, "SET_PKI ", 8) == 0) {
        float val = atof((strncmp(cmd, "KI_P ", 5) == 0) ? &cmd[5] : &cmd[8]);
        foc->conf.p_pid_ki = val;
        if (motor->m_conf != NULL) motor->m_conf->p_pid_ki = val;
    }
    else if (strncmp(cmd, "KD_P ", 5) == 0 || strncmp(cmd, "SET_PKD ", 8) == 0) {
        float val = atof((strncmp(cmd, "KD_P ", 5) == 0) ? &cmd[5] : &cmd[8]);
        foc->conf.p_pid_kd = val;
        if (motor->m_conf != NULL) motor->m_conf->p_pid_kd = val;
    }
    else if (strncmp(cmd, "DIR ", 4) == 0) {
        int d = atoi(&cmd[4]);
        if (d == 1 || d == -1) {
            foc->conf.encoder_direction = d;
            if (motor->m_conf != NULL) motor->m_conf->encoder_direction = d;
        }
    }
    else if (strncmp(cmd, "SWAP ", 5) == 0) {
        int s = atoi(&cmd[5]);
        foc->phase_swap_bc = (s != 0);
    }
    else if (strncmp(cmd, "OFFSET ", 7) == 0) {
        float off = atof(&cmd[7]);
        foc->zero_electric_angle = off;
        foc->aligned = true;
    }
    else if (strncmp(cmd, "DUTY ", 5) == 0) {
        float d = atof(&cmd[5]);
        if (d > 0.05f && d <= 0.95f) {
            foc->conf.l_max_duty = d;
            if (motor->m_conf != NULL) motor->m_conf->l_max_duty = d;
        }
    }
    else if (strncmp(cmd, "RAMP ", 5) == 0 || strncmp(cmd, "SET_RAMP ", 9) == 0) {
        float r = atof((strncmp(cmd, "RAMP ", 5) == 0) ? &cmd[5] : &cmd[9]);
        if (r >= 0.0f) {
            foc->conf.s_pid_ramp_erpms_s = r;
            if (motor->m_conf != NULL) motor->m_conf->s_pid_ramp_erpms_s = r;
        }
    }
    else if (strncmp(cmd, "LUT ", 4) == 0) {
        int idx = 0;
        int val = 0;
        if (sscanf(&cmd[4], "%d %d", &idx, &val) >= 2) {
            if (idx >= 0 && idx < ENCODER_LUT_SIZE) {
                foc->encoder_lut[idx] = (int16_t)val;
                foc->use_encoder_lut = true;
            }
        }
    }
    else if (strncmp(cmd, "USE_LUT ", 8) == 0 || strncmp(cmd, "ENABLE_LUT ", 11) == 0) {
        int en = atoi((strncmp(cmd, "USE_LUT ", 8) == 0) ? &cmd[8] : &cmd[11]);
        foc->use_encoder_lut = (en != 0);
    }
    else if (strncmp(cmd, "TEST_VQ ", 8) == 0 || strncmp(cmd, "STATIC_TEST ", 12) == 0) {
        float val = atof((strncmp(cmd, "TEST_VQ ", 8) == 0) ? &cmd[8] : &cmd[12]);
        motor->m_control_mode = CONTROL_MODE_DUTY;
        motor->m_motor_state.duty_now = val / motor->m_motor_state.v_bus;
        motor->m_state = MC_STATE_RUNNING;
=======
        motor->m_speed_command_rpm = mech_rpm * pole_pairs;
        motor->m_control_mode = CONTROL_MODE_SPEED;
        motor->m_state = MC_STATE_RUNNING;
        run_foc_mode = 3;
    }
    else if (strncmp(cmd, "IQ ", 3) == 0 || strncmp(cmd, "CURRENT ", 8) == 0) {
        float iq = atof((strncmp(cmd, "IQ ", 3) == 0) ? &cmd[3] : &cmd[8]);
        iq_target_dbg = iq;
        motor->m_iq_set = iq;
        motor->m_control_mode = CONTROL_MODE_CURRENT;
        motor->m_state = MC_STATE_RUNNING;
        run_foc_mode = 1;
    }
    else if (strncmp(cmd, "POS ", 4) == 0) {
        float pos = atof(&cmd[4]);
        pos_target_dbg = pos;
        motor->m_pos_pid_set = pos;
        motor->m_control_mode = CONTROL_MODE_POS;
        motor->m_state = MC_STATE_RUNNING;
        run_foc_mode = 2;
>>>>>>> 8e44a795456836680c75c6d0526c6dd48d62f00d
    }
}

/**
  * @brief  Periodic process function called from main while(1) loop
  */
void Comm_Telemetry_Process(FOC_Controller_t *foc)
{
    if (foc == NULL) return;

    // Transmit telemetry at 100Hz (every 10ms)
    uint32_t now = HAL_GetTick();
    if (now - s_last_telemetry_tx_ms >= 10) {
<<<<<<< HEAD
        if (Comm_Telemetry_Send(foc)) {
            s_last_telemetry_tx_ms = now;
        }
=======
        s_last_telemetry_tx_ms = now;
        Comm_Telemetry_Send(foc);
>>>>>>> 8e44a795456836680c75c6d0526c6dd48d62f00d
    }
}

/**
  * @brief  Single-byte reception handler
  */
void Comm_Telemetry_RxByte(uint8_t rx_byte)
{
    if (rx_byte == '\n' || rx_byte == '\r') {
        if (s_rx_cmd_idx > 0) {
            s_rx_cmd_buffer[s_rx_cmd_idx] = '\0';
            extern FOC_Controller_t g_foc_controller;
            ProcessCommand(&g_foc_controller, s_rx_cmd_buffer);
            s_rx_cmd_idx = 0;
        }
    } else {
        if (s_rx_cmd_idx < sizeof(s_rx_cmd_buffer) - 1) {
            s_rx_cmd_buffer[s_rx_cmd_idx++] = (char)rx_byte;
        }
    }
}

/**
  * @brief  Buffer reception handler (called from USB CDC RX callback)
  */
void Comm_Telemetry_RxBuffer(const uint8_t *buf, uint32_t len)
{
    if (buf == NULL || len == 0) return;
    for (uint32_t i = 0; i < len; i++) {
        Comm_Telemetry_RxByte(buf[i]);
    }
}
