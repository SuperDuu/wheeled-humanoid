/*
 * comm_can.c - VESC Protocol Extended CAN Communication Driver
 * Adapted for STM32G4 Joint Driver 8115
 */

#include "comm_can.h"
#include "vesc_utils.h"
#include <string.h>

static FDCAN_HandleTypeDef *g_hfdcan = NULL;
static uint8_t g_node_id = DEFAULT_CAN_NODE_ID;

/* Big-Endian Buffer Helpers (VESC Protocol) */
static int32_t buffer_get_int32(const uint8_t *buffer, int32_t *index) {
    int32_t res = ((uint32_t)buffer[*index] << 24) |
                  ((uint32_t)buffer[*index + 1] << 16) |
                  ((uint32_t)buffer[*index + 2] << 8) |
                  ((uint32_t)buffer[*index + 3]);
    *index += 4;
    return res;
}

static void buffer_append_int32(uint8_t *buffer, int32_t number, int32_t *index) {
    buffer[(*index)++] = (uint8_t)(number >> 24);
    buffer[(*index)++] = (uint8_t)(number >> 16);
    buffer[(*index)++] = (uint8_t)(number >> 8);
    buffer[(*index)++] = (uint8_t)(number);
}

static void buffer_append_int16(uint8_t *buffer, int16_t number, int32_t *index) {
    buffer[(*index)++] = (uint8_t)(number >> 8);
    buffer[(*index)++] = (uint8_t)(number);
}

/**
  * @brief  Initialize FDCAN2 Driver & Filter for VESC Extended CAN protocol
  */
void comm_can_init(FDCAN_HandleTypeDef *hfdcan, uint8_t node_id)
{
    if (hfdcan == NULL) return;
    g_hfdcan = hfdcan;
    g_node_id = node_id;

    /* Configure FDCAN Global Filter to accept all extended frames into FIFO0 */
    FDCAN_FilterTypeDef sFilterConfig;
    sFilterConfig.IdType = FDCAN_EXTENDED_ID;
    sFilterConfig.FilterIndex = 0;
    sFilterConfig.FilterType = FDCAN_FILTER_MASK;
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    sFilterConfig.FilterID1 = 0x00000000;
    sFilterConfig.FilterID2 = 0x00000000; // Mask = 0 -> Accept all IDs

    HAL_FDCAN_ConfigFilter(g_hfdcan, &sFilterConfig);
    HAL_FDCAN_ConfigGlobalFilter(g_hfdcan, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);

    /* Start FDCAN Peripheral */
    HAL_FDCAN_Start(g_hfdcan);

    /* Activate Rx FIFO0 New Message Interrupt */
    HAL_FDCAN_ActivateNotification(g_hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
}

/**
  * @brief  Set CAN Node ID
  */
void comm_can_set_node_id(uint8_t node_id)
{
    g_node_id = node_id;
}

/**
  * @brief  Get CAN Node ID
  */
uint8_t comm_can_get_node_id(void)
{
    return g_node_id;
}

/**
  * @brief  Process Received CAN Frame (Supports both MIT Mini Cheetah Standard & VESC Extended)
  */
void comm_can_process_rx_frame(FDCAN_RxHeaderTypeDef *rx_header, uint8_t *rx_data)
{
    if (rx_header == NULL || rx_data == NULL) return;

    /* =========================================================================
     * 1. MIT MINI CHEETAH STANDARD 11-BIT CAN PROTOCOL
     * ========================================================================= */
    if (rx_header->IdType == FDCAN_STANDARD_ID) {
        uint32_t std_id = rx_header->Identifier;
        if (std_id != g_node_id && std_id != CAN_BROADCAST_ID) {
            return;
        }

        /* Check for Special MIT Mode Switching Payloads */
        if (rx_data[0] == 0xFF && rx_data[1] == 0xFF && rx_data[2] == 0xFF &&
            rx_data[3] == 0xFF && rx_data[4] == 0xFF && rx_data[5] == 0xFF && rx_data[6] == 0xFF) {
            if (rx_data[7] == 0xFC) {
                /* Enter Motor Mode */
                g_foc_controller.motor.m_state = MC_STATE_RUNNING;
                g_foc_controller.motor.m_control_mode = CONTROL_MODE_CURRENT;
                g_foc_controller.fault = MC_FAULT_NONE;
                TIM1_EnsureMoeEnabled();
                return;
            } else if (rx_data[7] == 0xFD) {
                /* Exit Motor Mode */
                g_foc_controller.motor.m_state = MC_STATE_OFF;
                g_foc_controller.motor.m_iq_set = 0.0f;
                return;
            } else if (rx_data[7] == 0xFE) {
                /* Zero Position (Set Home) */
                foc_set_home_position(&g_foc_controller.motor);
                return;
            }
        }

        /* Unpack MIT 8-byte Impedance Control Payload */
        int p_int  = (rx_data[0] << 8) | rx_data[1];
        int v_int  = (rx_data[2] << 4) | (rx_data[3] >> 4);
        int kp_int = ((rx_data[3] & 0x0F) << 8) | rx_data[4];
        int kd_int = (rx_data[5] << 4) | (rx_data[6] >> 4);
        int t_int  = ((rx_data[6] & 0x0F) << 8) | rx_data[7];

        float p_des = uint_to_float(p_int,  MIT_P_MIN,  MIT_P_MAX,  16);
        float v_des = uint_to_float(v_int,  MIT_V_MIN,  MIT_V_MAX,  12);
        float kp    = uint_to_float(kp_int, MIT_KP_MIN, MIT_KP_MAX, 12);
        float kd    = uint_to_float(kd_int, MIT_KD_MIN, MIT_KD_MAX, 12);
        float t_ff  = uint_to_float(t_int,  MIT_T_MIN,  MIT_T_MAX,  12);

        /* Real-Time Impedance Control: Tau = Kp*(p_des - p) + Kd*(v_des - v) + t_ff */
        float p_actual = g_foc_controller.motor.m_joint_angle; // Output rad
        float gear_ratio = (g_foc_controller.conf.gear_ratio > 0.1f) ? g_foc_controller.conf.gear_ratio : 17.0f;
        float pole_pairs = (float)g_foc_controller.conf.foc_motor_pole_pairs;
        float v_actual = (g_foc_controller.motor.m_speed_est_fast / pole_pairs) / gear_ratio; // Output rad/s

        float torque_cmd = kp * (p_des - p_actual) + kd * (v_des - v_actual) + t_ff;
        float kt_joint = g_foc_controller.conf.foc_motor_flux_linkage * 1.5f * pole_pairs * gear_ratio;
        float iq_cmd = torque_cmd / (kt_joint > 0.1f ? kt_joint : 6.2f);

        utils_truncate_number_abs(&iq_cmd, g_foc_controller.conf.l_current_max);
        g_foc_controller.motor.m_iq_set = iq_cmd;
        g_foc_controller.motor.m_control_mode = CONTROL_MODE_CURRENT;
        g_foc_controller.motor.m_state = MC_STATE_RUNNING;
        TIM1_EnsureMoeEnabled();

        /* Immediate Packed Reply (p_actual, v_actual, torque_actual from measured Iq) */
        float t_actual = g_foc_controller.motor.m_motor_state.iq_filter * kt_joint;
        comm_can_send_mit_reply(p_actual, v_actual, t_actual);
        return;
    }

    /* =========================================================================
     * 2. VESC EXTENDED 29-BIT CAN PROTOCOL
     * ========================================================================= */
    uint32_t can_id = rx_header->Identifier;
    uint8_t cmd_id = (can_id >> 8) & 0xFF;
    uint8_t target_id = can_id & 0xFF;

    if (target_id != g_node_id && target_id != CAN_BROADCAST_ID) {
        return;
    }

    int32_t idx = 0;

    switch (cmd_id) {
    case CAN_PACKET_SET_POS: {
        int32_t pos_scaled = buffer_get_int32(rx_data, &idx);
        float pos_deg = (float)pos_scaled / 100000.0f;
        motor_set_position(pos_deg);
        break;
    }

    case CAN_PACKET_SET_CURRENT: {
        int32_t current_scaled = buffer_get_int32(rx_data, &idx);
        float current_amps = (float)current_scaled / 1000.0f;
        motor_set_current(current_amps);
        break;
    }

    case CAN_PACKET_SET_RPM: {
        int32_t rpm = buffer_get_int32(rx_data, &idx);
        motor_set_speed((float)rpm);
        break;
    }

    case CAN_PACKET_SET_DUTY: {
        int32_t duty_scaled = buffer_get_int32(rx_data, &idx);
        float duty = (float)duty_scaled / 100000.0f;
        g_foc_controller.motor.m_control_mode = CONTROL_MODE_CURRENT;
        g_foc_controller.motor.m_motor_state.iq_target = duty * g_foc_controller.conf.l_current_max;
        break;
    }

    default:
        break;
    }
}

/**
  * @brief  Transmit MIT Mini Cheetah Packed Reply Packet (6 Bytes)
  */
void comm_can_send_mit_reply(float p_actual, float v_actual, float t_actual)
{
    if (g_hfdcan == NULL) return;

    FDCAN_TxHeaderTypeDef tx_header;
    tx_header.Identifier = CAN_MASTER_ID;
    tx_header.IdType = FDCAN_STANDARD_ID;
    tx_header.TxFrameType = FDCAN_DATA_FRAME;
    tx_header.DataLength = FDCAN_DLC_BYTES_6;
    tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx_header.BitRateSwitch = FDCAN_BRS_OFF;
    tx_header.FDFormat = FDCAN_CLASSIC_CAN;
    tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    tx_header.MessageMarker = 0;

    int p_int = float_to_uint(p_actual, MIT_P_MIN, MIT_P_MAX, 16);
    int v_int = float_to_uint(v_actual, MIT_V_MIN, MIT_V_MAX, 12);
    int t_int = float_to_uint(t_actual, MIT_T_MIN, MIT_T_MAX, 12);

    uint8_t data[6];
    data[0] = g_node_id;
    data[1] = (uint8_t)(p_int >> 8);
    data[2] = (uint8_t)(p_int & 0xFF);
    data[3] = (uint8_t)(v_int >> 4);
    data[4] = (uint8_t)(((v_int & 0x0F) << 4) | (t_int >> 8));
    data[5] = (uint8_t)(t_int & 0xFF);

    HAL_FDCAN_AddMessageToTxFifoQ(g_hfdcan, &tx_header, data);
}

/**
  * @brief  Transmit VESC CAN Status 1 Packet (RPM, Current, Duty) @ 100Hz
  */
void comm_can_send_status1(void)
{
    if (g_hfdcan == NULL) return;

    FDCAN_TxHeaderTypeDef tx_header;
    tx_header.Identifier = ((uint32_t)CAN_PACKET_STATUS << 8) | (uint32_t)g_node_id;
    tx_header.IdType = FDCAN_EXTENDED_ID;
    tx_header.TxFrameType = FDCAN_DATA_FRAME;
    tx_header.DataLength = FDCAN_DLC_BYTES_8;
    tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx_header.BitRateSwitch = FDCAN_BRS_OFF;
    tx_header.FDFormat = FDCAN_CLASSIC_CAN;
    tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    tx_header.MessageMarker = 0;

    uint8_t data[8];
    int32_t idx = 0;

    /* 1. Joint RPM (int32) */
    int32_t rpm = (int32_t)motor_get_speed();
    buffer_append_int32(data, rpm, &idx);

    /* 2. Current Iq in Amperes * 10 (int16) */
    int16_t current = (int16_t)(motor_get_current() * 10.0f);
    buffer_append_int16(data, current, &idx);

    /* 3. Duty Cycle * 1000 (int16) */
    int16_t duty = (int16_t)(g_foc_controller.duty_a * 1000.0f);
    buffer_append_int16(data, duty, &idx);

    HAL_FDCAN_AddMessageToTxFifoQ(g_hfdcan, &tx_header, data);
}

/**
  * @brief  Transmit VESC CAN Status 5 Packet (Joint Position, VBUS, Temp) @ 100Hz
  */
void comm_can_send_status5(void)
{
    if (g_hfdcan == NULL) return;

    FDCAN_TxHeaderTypeDef tx_header;
    tx_header.Identifier = ((uint32_t)CAN_PACKET_STATUS_5 << 8) | (uint32_t)g_node_id;
    tx_header.IdType = FDCAN_EXTENDED_ID;
    tx_header.TxFrameType = FDCAN_DATA_FRAME;
    tx_header.DataLength = FDCAN_DLC_BYTES_8;
    tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx_header.BitRateSwitch = FDCAN_BRS_OFF;
    tx_header.FDFormat = FDCAN_CLASSIC_CAN;
    tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    tx_header.MessageMarker = 0;

    uint8_t data[8];
    int32_t idx = 0;

    /* 1. Joint Position in Degrees * 100,000 (int32) */
    int32_t pos = (int32_t)(motor_get_position() * 100000.0f);
    buffer_append_int32(data, pos, &idx);

    /* 2. VBUS Voltage * 10 (int16) */
    int16_t vbus = (int16_t)(motor_get_vbus() * 10.0f);
    buffer_append_int16(data, vbus, &idx);

    /* 3. MOSFET Temp * 10 (int16) */
    int16_t temp = (int16_t)(25.0f * 10.0f); // FET Temp
    buffer_append_int16(data, temp, &idx);

    HAL_FDCAN_AddMessageToTxFifoQ(g_hfdcan, &tx_header, data);
}

/**
  * @brief  HAL FDCAN Rx FIFO 0 Callback (Triggered when new CAN message arrives)
  */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != 0) {
        FDCAN_RxHeaderTypeDef rx_header;
        uint8_t rx_data[8];

        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_header, rx_data) == HAL_OK) {
            comm_can_process_rx_frame(&rx_header, rx_data);
        }
    }
}
