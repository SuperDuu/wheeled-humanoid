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

/* USB Device Handle */
extern USBD_HandleTypeDef hUsbDeviceFS;

/* Global references */
static uint32_t s_last_telemetry_tx_ms = 0;
static char s_rx_cmd_buffer[64];
static uint8_t s_rx_cmd_idx = 0;

/* Open-Loop Test Run Control Globals */
volatile uint8_t run_open_loop = 0;
volatile float open_loop_target_rpm = 100.0f;
volatile float open_loop_current_rpm = 0.0f;
volatile float open_loop_angle = 0.0f;

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
bool Comm_Telemetry_Send(FOC_Controller_t *foc)
{
    if (foc == NULL) return false;

    motor_all_state_t *motor = &foc->motor;
    motor_state_t *state_m = &motor->m_motor_state;
    mc_configuration *conf = motor->m_conf;

    /* CRITICAL: packet PHẢI là static vì CDC_Transmit_FS chỉ lưu CON TRỎ.
     * Packet 78 bytes > 64 bytes (USB FS max packet) → cần 2 USB transactions.
     * Transaction thứ 2 (14 bytes cuối) xảy ra trong USB IRQ SAU KHI hàm return.
     * Nếu packet trên stack → pointer trỏ vào rác → 14 bytes cuối corrupt → checksum fail 95%. */
    static telemetry_packet_t packet;
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

    // 2. FOC Vector Currents (Filtered DC for smooth telemetry display)
    packet.i_d = state_m->id_filter;
    packet.i_q = state_m->iq_filter;
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
    packet.encoder_dir = (int8_t)conf->encoder_direction;

    // FOC Diagnostic Fields
    packet.vd = state_m->vd;
    packet.vq = state_m->vq;
    packet.zero_elec_angle = foc->zero_electric_angle;
    packet.id_target = state_m->id_target;

    // Calculate Checksum over payload (excluding magic & checksum itself)
    uint8_t *raw_buf = (uint8_t*)&packet;
    packet.checksum = CalculateChecksum(&raw_buf[4], sizeof(telemetry_packet_t) - 6);

    // Transmit via Native USB CDC (Virtual COM Port /dev/ttyACM*)
    if (hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED && hUsbDeviceFS.pClassData != NULL) {
        USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData;
        if (hcdc->TxState == 0) {
            CDC_Transmit_FS((uint8_t*)&packet, sizeof(packet));
            return true;
        }
    }
    return false;
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

    if (strncmp(cmd, "STOP", 4) == 0 || strncmp(cmd, "OFF", 3) == 0) {
        motor->m_state = MC_STATE_OFF;
        motor->m_iq_set = 0.0f;
        motor->m_speed_command_rpm = 0.0f;
        motor->m_speed_pid_set_rpm = 0.0f;
        motor->m_speed_i_term = 0.0f;
        motor->m_motor_state.duty_now = 0.0f;
        motor->m_motor_state.vd = 0.0f;
        motor->m_motor_state.vq = 0.0f;
        foc->fault = MC_FAULT_NONE;
        speed_target_dbg = 0.0f;
        iq_target_dbg = 0.0f;
        run_foc_mode = 0;
        run_open_loop = 0;
    }
    else if (strncmp(cmd, "ALIGN", 5) == 0) {
        extern volatile int run_alignment;
        run_alignment = 1;
        run_open_loop = 0;
        run_foc_mode = 0;
    }
    else if (strncmp(cmd, "OPENLOOP", 8) == 0 || strncmp(cmd, "TEST", 4) == 0 || strncmp(cmd, "RUN", 3) == 0) {
        float rpm = 100.0f;
        if (strlen(cmd) > 8 && strncmp(cmd, "OPENLOOP", 8) == 0) {
            float val = atof(&cmd[8]);
            if (val != 0.0f) rpm = val;
        } else if (strlen(cmd) > 4 && strncmp(cmd, "TEST", 4) == 0) {
            float val = atof(&cmd[4]);
            if (val != 0.0f) rpm = val;
        }
        open_loop_target_rpm = rpm;
        open_loop_current_rpm = 0.0f;
        run_open_loop = 1;
        motor->m_state = MC_STATE_RUNNING;
        foc->fault = MC_FAULT_NONE;
        TIM1_EnsureMoeEnabled();
    }
    else if (strncmp(cmd, "MODE ", 5) == 0) {
        int m = atoi(&cmd[5]);
        run_open_loop = 0;
        foc->fault = MC_FAULT_NONE;
        if (m == 0) {
            motor->m_state = MC_STATE_OFF;
            motor->m_iq_set = 0.0f;
            motor->m_speed_command_rpm = 0.0f;
            motor->m_speed_pid_set_rpm = 0.0f;
            motor->m_speed_i_term = 0.0f;
            motor->m_motor_state.duty_now = 0.0f;
            speed_target_dbg = 0.0f;
            iq_target_dbg = 0.0f;
            run_foc_mode = 0;
        } else if (m >= 1 && m <= 5) {
            motor->m_state = MC_STATE_RUNNING;
            TIM1_EnsureMoeEnabled();
            if (m == 1) { motor->m_control_mode = CONTROL_MODE_CURRENT; run_foc_mode = 1; }
            else if (m == 2) { motor->m_control_mode = CONTROL_MODE_CURRENT_BRAKE; run_foc_mode = 1; }
            else if (m == 3) { motor->m_control_mode = CONTROL_MODE_SPEED; run_foc_mode = 3; }
            else if (m == 4) { motor->m_control_mode = CONTROL_MODE_POS; run_foc_mode = 2; }
            else if (m == 5) { motor->m_control_mode = CONTROL_MODE_DUTY; run_foc_mode = 4; }
        }
    }
    else if (strncmp(cmd, "SPEED ", 6) == 0) {
        float mech_rpm = atof(&cmd[6]);
        float pole_pairs = (motor->m_conf != NULL) ? (float)motor->m_conf->foc_motor_pole_pairs : 21.0f;
        speed_target_dbg = mech_rpm;
        foc->fault = MC_FAULT_NONE;
        if (run_open_loop == 1) {
            open_loop_target_rpm = mech_rpm;
        } else {
            motor->m_speed_command_rpm = mech_rpm * pole_pairs;
            motor->m_control_mode = CONTROL_MODE_SPEED;
            motor->m_state = MC_STATE_RUNNING;
            run_foc_mode = 3;
            TIM1_EnsureMoeEnabled();
        }
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
    else if (strncmp(cmd, "MOVE ", 5) == 0) {
        // Cú pháp: MOVE <góc_độ> <thời_gian_ms> (Vd: MOVE 90 1000 = quay 90 độ trong 1.0 giây rồi ghim cứng)
        float target_deg = 0.0f;
        float duration_ms = 800.0f; // Mặc định 800ms nếu không nhập thời gian
        sscanf(&cmd[5], "%f %f", &target_deg, &duration_ms);
        if (duration_ms < 50.0f) duration_ms = 50.0f;

        float target_rad = DEG2RAD_f(target_deg);
        foc_start_trajectory(motor, target_rad, duration_ms / 1000.0f);
        pos_target_dbg = target_rad;
        run_foc_mode = 2;
        foc->fault = MC_FAULT_NONE;
        TIM1_EnsureMoeEnabled();
    }
    else if (strncmp(cmd, "POS ", 4) == 0 || strncmp(cmd, "ANGLE ", 6) == 0) {
        float pos_deg = atof((strncmp(cmd, "POS ", 4) == 0) ? &cmd[4] : &cmd[6]);
        float pos_rad = DEG2RAD_f(pos_deg);
        foc_start_trajectory(motor, pos_rad, 0.8f); // Dốc S-Curve mượt mà 800ms tới đích rồi ghim cứng
        pos_target_dbg = pos_rad;
        run_foc_mode = 2;
        foc->fault = MC_FAULT_NONE;
        TIM1_EnsureMoeEnabled();
    }
    else if (strncmp(cmd, "SLOT ", 5) == 0) {
        // 12 VỊ TRÍ GHIM KHỚP TAY ROBOT (1 đến 12 cách nhau 30 độ)
        int slot = atoi(&cmd[5]);
        if (slot >= 1 && slot <= 12) {
            float slot_angles_deg[12] = {0.0f, 30.0f, 60.0f, 90.0f, 120.0f, 150.0f, 180.0f, -150.0f, -120.0f, -90.0f, -60.0f, -30.0f};
            float target_rad = DEG2RAD_f(slot_angles_deg[slot - 1]);
            foc_start_trajectory(motor, target_rad, 0.8f);
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
    else if (strncmp(cmd, "KP_S ", 5) == 0 || strncmp(cmd, "SET_SKP ", 8) == 0) {
        float val = atof((strncmp(cmd, "KP_S ", 5) == 0) ? &cmd[5] : &cmd[8]);
        foc->conf.s_pid_kp = val;
    }
    else if (strncmp(cmd, "KI_S ", 5) == 0 || strncmp(cmd, "SET_SKI ", 8) == 0) {
        float val = atof((strncmp(cmd, "KI_S ", 5) == 0) ? &cmd[5] : &cmd[8]);
        foc->conf.s_pid_ki = val;
    }
    else if (strncmp(cmd, "KD_S ", 5) == 0 || strncmp(cmd, "SET_SKD ", 8) == 0) {
        float val = atof((strncmp(cmd, "KD_S ", 5) == 0) ? &cmd[5] : &cmd[8]);
        foc->conf.s_pid_kd = val;
    }
    else if (strncmp(cmd, "KP_P ", 5) == 0 || strncmp(cmd, "SET_PKP ", 8) == 0) {
        float val = atof((strncmp(cmd, "KP_P ", 5) == 0) ? &cmd[5] : &cmd[8]);
        foc->conf.p_pid_kp = val;
    }
    else if (strncmp(cmd, "KI_P ", 5) == 0 || strncmp(cmd, "SET_PKI ", 8) == 0) {
        float val = atof((strncmp(cmd, "KI_P ", 5) == 0) ? &cmd[5] : &cmd[8]);
        foc->conf.p_pid_ki = val;
    }
    else if (strncmp(cmd, "KD_P ", 5) == 0 || strncmp(cmd, "SET_PKD ", 8) == 0) {
        float val = atof((strncmp(cmd, "KD_P ", 5) == 0) ? &cmd[5] : &cmd[8]);
        foc->conf.p_pid_kd = val;
    }
    else if (strncmp(cmd, "DIR ", 4) == 0) {
        int d = atoi(&cmd[4]);
        if (d == 1 || d == -1) foc->conf.encoder_direction = d;
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
        if (d > 0.05f && d <= 0.95f) foc->conf.l_max_duty = d;
    }
    else if (strncmp(cmd, "RAMP ", 5) == 0 || strncmp(cmd, "SET_RAMP ", 9) == 0) {
        float r = atof((strncmp(cmd, "RAMP ", 5) == 0) ? &cmd[5] : &cmd[9]);
        if (r >= 0.0f) foc->conf.s_pid_ramp_erpms_s = r;
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
        if (Comm_Telemetry_Send(foc)) {
            s_last_telemetry_tx_ms = now;
        }
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
