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

    // 2. FOC Vector Currents
    packet.i_d = state_m->id;
    packet.i_q = state_m->iq;
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
    packet.reserved = 0;

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
        run_foc_mode = 0;
        run_open_loop = 0;
    }
    else if (strncmp(cmd, "ALIGN", 5) == 0) {
        extern volatile int run_alignment;
        run_alignment = 1;
        run_open_loop = 0;
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
        run_open_loop = 1;
        motor->m_state = MC_STATE_RUNNING;
        TIM1_EnsureMoeEnabled();
    }
    else if (strncmp(cmd, "MODE ", 5) == 0) {
        int m = atoi(&cmd[5]);
        run_open_loop = 0;
        if (m == 0) {
            motor->m_state = MC_STATE_OFF;
            motor->m_iq_set = 0.0f;
            motor->m_speed_command_rpm = 0.0f;
            run_foc_mode = 0;
        } else if (m >= 1 && m <= 4) {
            motor->m_state = MC_STATE_RUNNING;
            if (m == 1) { motor->m_control_mode = CONTROL_MODE_CURRENT; run_foc_mode = 1; }
            else if (m == 2) { motor->m_control_mode = CONTROL_MODE_CURRENT_BRAKE; run_foc_mode = 1; }
            else if (m == 3) { motor->m_control_mode = CONTROL_MODE_SPEED; run_foc_mode = 3; }
            else if (m == 4) { motor->m_control_mode = CONTROL_MODE_POS; run_foc_mode = 2; }
        }
    }
    else if (strncmp(cmd, "SPEED ", 6) == 0) {
        float mech_rpm = atof(&cmd[6]);
        float pole_pairs = (motor->m_conf != NULL) ? (float)motor->m_conf->foc_motor_pole_pairs : 21.0f;
        speed_target_dbg = mech_rpm;
        if (run_open_loop == 1) {
            open_loop_target_rpm = mech_rpm;
        } else {
            motor->m_speed_command_rpm = mech_rpm * pole_pairs;
            motor->m_control_mode = CONTROL_MODE_SPEED;
            motor->m_state = MC_STATE_RUNNING;
            run_foc_mode = 3;
        }
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
