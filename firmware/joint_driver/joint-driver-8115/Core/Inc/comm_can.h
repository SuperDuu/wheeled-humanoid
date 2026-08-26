/*
 * comm_can.h - VESC Protocol Extended CAN Communication Driver
 * Adapted for STM32G4 Joint Driver 8115
 */

#ifndef COMM_CAN_H_
#define COMM_CAN_H_

#include "main.h"
#include "motor_interface.h"
#include <stdbool.h>

/* VESC Standard CAN Packet IDs (Extended 29-bit CAN ID) */
typedef enum {
	CAN_PACKET_SET_DUTY = 0,
	CAN_PACKET_SET_CURRENT = 1,
	CAN_PACKET_SET_CURRENT_BRAKE = 2,
	CAN_PACKET_SET_RPM = 3,
	CAN_PACKET_SET_POS = 4,
	CAN_PACKET_FILL_RX_BUFFER = 5,
	CAN_PACKET_FILL_RX_BUFFER_LONG = 6,
	CAN_PACKET_PROCESS_RX_BUFFER = 7,
	CAN_PACKET_PROCESS_SHORT_BUFFER = 8,
	CAN_PACKET_STATUS = 9,
	CAN_PACKET_SET_CURRENT_REL = 10,
	CAN_PACKET_SET_CURRENT_BRAKE_REL = 11,
	CAN_PACKET_SET_CURRENT_HANDBRAKE = 12,
	CAN_PACKET_SET_CURRENT_HANDBRAKE_REL = 13,
	CAN_PACKET_STATUS_2 = 14,
	CAN_PACKET_STATUS_3 = 15,
	CAN_PACKET_STATUS_4 = 16,
	CAN_PACKET_PING = 17,
	CAN_PACKET_PONG = 18,
	CAN_PACKET_STATUS_5 = 27,
} CAN_PACKET_ID;

/* Node Configuration */
#define DEFAULT_CAN_NODE_ID     1       /* Default Joint Node ID (1..254) */
#define CAN_BROADCAST_ID        255     /* Broadcast ID */
#define CAN_MASTER_ID           0x00    /* Host PC / Master Controller ID */

/* MIT Mini Cheetah CAN Protocol Standard Ranges */
#define MIT_P_MIN               (-12.5f)    /* rad */
#define MIT_P_MAX               (+12.5f)    /* rad */
#define MIT_V_MIN               (-45.0f)    /* rad/s */
#define MIT_V_MAX               (+45.0f)    /* rad/s */
#define MIT_KP_MIN              (0.0f)      /* N*m/rad */
#define MIT_KP_MAX              (500.0f)    /* N*m/rad */
#define MIT_KD_MIN              (0.0f)      /* N*m*s/rad */
#define MIT_KD_MAX              (5.0f)      /* N*m*s/rad */
#define MIT_T_MIN               (-18.0f)    /* N*m */
#define MIT_T_MAX               (+18.0f)    /* N*m */

/* Function Prototypes */
void comm_can_init(FDCAN_HandleTypeDef *hfdcan, uint8_t node_id);
void comm_can_process_rx_frame(FDCAN_RxHeaderTypeDef *rx_header, uint8_t *rx_data);
void comm_can_send_status1(void);
void comm_can_send_status5(void);
void comm_can_send_mit_reply(float p_actual, float v_actual, float t_actual);
void comm_can_set_node_id(uint8_t node_id);
uint8_t comm_can_get_node_id(void);

#endif /* COMM_CAN_H_ */
