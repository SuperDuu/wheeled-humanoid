/*
 * can_protocol_def.h - Unified CAN & CAN-FD Protocol & ID Mapping
 * Project: Wheeled Humanoid Robotics - Multi-Node Actuator Bus
 * Standard: ISO 11898-1:2015 (Classical CAN 2.0B & FDCAN)
 */

#ifndef CAN_PROTOCOL_DEF_H_
#define CAN_PROTOCOL_DEF_H_

#include <stdint.h>

/* =========================================================================
 * 1. UNIFIED CAN ARBITRATION ID HIERARCHY (11-bit Standard ID)
 *
 * Lower numerical ID has HIGHEST priority on the physical bus (Wired-AND).
 * Hierarchy ensures safety-critical frames preempt normal motion commands.
 * ========================================================================= */

/* Priority 0 (0x000 - 0x00F): Emergency Stop (Highest Preemptive Priority) */
#define CAN_ID_ESTOP_BROADCAST          0x001   /* Global Emergency Stop (Host -> All Nodes) */
#define CAN_ID_ESTOP_ACK                0x002   /* Node Emergency Stop Acknowledgment */

/* Priority 1 (0x010 - 0x03F): Heartbeat & Safety Watchdog (50 Hz periodic) */
#define CAN_ID_HEARTBEAT_BASE           0x010   /* Heartbeat: 0x010 + Node_ID (1..16) */
#define CAN_ID_HEARTBEAT_MASTER         0x010   /* Master Controller Heartbeat */

/* Priority 2 (0x040 - 0x07F): Hardware Faults & Emergency Notifications */
#define CAN_ID_FAULT_BASE               0x040   /* Fault Event: 0x040 + Node_ID */

/* Priority 3 (0x100 - 0x17F): Master Motion Commands (Jetson/PC -> Nodes, 250Hz - 1kHz) */
#define CAN_ID_CMD_BASE                 0x100   /* Joint Command: 0x100 + Node_ID */
#define CAN_ID_CMD_SYNC_BROADCAST       0x100   /* Synchronous Motion Trigger (All Nodes) */

/* Priority 4 (0x200 - 0x27F): Joint State Telemetry (Nodes -> Jetson/PC, 250Hz - 1kHz) */
#define CAN_ID_FEEDBACK_BASE            0x200   /* Joint Telemetry: 0x200 + Node_ID */

/* Priority 5 (0x700 - 0x77F): Diagnostics, Parameter Config & Calibration */
#define CAN_ID_DIAG_BASE                0x700   /* Diagnostics / VESC Terminal: 0x700 + Node_ID */

/* =========================================================================
 * 2. HUMANOID ROBOT NODE ID ALLOCATION (1..16)
 * ========================================================================= */
#define NODE_ID_MASTER                  0x00    /* Jetson Orin / Host Controller */

/* Right Arm / Primary Arm (7-DOF + Gripper) */
#define NODE_ID_ARM_SHOULDER_PITCH      0x01    /* Joint 1 */
#define NODE_ID_ARM_SHOULDER_ROLL       0x02    /* Joint 2 */
#define NODE_ID_ARM_SHOULDER_YAW        0x03    /* Joint 3 */
#define NODE_ID_ARM_ELBOW_PITCH         0x04    /* Joint 4 */
#define NODE_ID_ARM_WRIST_YAW           0x05    /* Joint 5 */
#define NODE_ID_ARM_WRIST_PITCH         0x06    /* Joint 6 */
#define NODE_ID_ARM_WRIST_ROLL          0x07    /* Joint 7 */
#define NODE_ID_ARM_GRIPPER             0x08    /* End-Effector Gripper */

/* Left Arm / Secondary Arm (Optional Expansion) */
#define NODE_ID_ARM_L_SHOULDER_PITCH    0x09
#define NODE_ID_ARM_L_SHOULDER_ROLL     0x0A
#define NODE_ID_ARM_L_SHOULDER_YAW      0x0B
#define NODE_ID_ARM_L_ELBOW_PITCH       0x0C
#define NODE_ID_ARM_L_WRIST_YAW         0x0D
#define NODE_ID_ARM_L_WRIST_PITCH       0x0E
#define NODE_ID_ARM_L_WRIST_ROLL        0x0F
#define NODE_ID_ARM_L_GRIPPER           0x10

#define CAN_BROADCAST_NODE_ID           0xFF

/* =========================================================================
 * 3. FAIL-SAFE TIMINGS & SAFETY CONSTRAINTS
 * ========================================================================= */
#define HEARTBEAT_INTERVAL_MS           20      /* 50 Hz Nominal Heartbeat */
#define HEARTBEAT_TIMEOUT_WARNING_MS    60      /* 3 missed packets -> Warning */
#define HEARTBEAT_TIMEOUT_ESTOP_MS      100     /* 5 missed packets -> Watchdog Trip / Safe-Hold */

#define ESTOP_MAX_LATENCY_US            500     /* E-Stop must reach all nodes in < 500 us */

/* Node Operating States */
typedef enum {
    NODE_STATE_UNINITIALIZED = 0,
    NODE_STATE_STANDBY       = 1,
    NODE_STATE_RUNNING       = 2,
    NODE_STATE_WARNING       = 3,
    NODE_STATE_FAILSAFE_HOLD = 4,   /* Heartbeat timeout or high BER: Hold with damping */
    NODE_STATE_ESTOP_TRIPPED = 5,   /* Hard E-Stop: PWM disabled, zero current, brake engaged */
    NODE_STATE_BUS_OFF       = 6    /* FDCAN hardware bus-off state (TEC > 255) */
} NodeOperationalState;

/* Node Fault Flags (Bitmask) */
#define FAULT_FLAG_NONE                 0x0000
#define FAULT_FLAG_OVER_CURRENT         0x0001
#define FAULT_FLAG_OVER_VOLTAGE         0x0002
#define FAULT_FLAG_UNDER_VOLTAGE        0x0004
#define FAULT_FLAG_OVER_TEMP_MOSFET     0x0008
#define FAULT_FLAG_OVER_TEMP_MOTOR      0x0010
#define FAULT_FLAG_ENCODER_CRC          0x0020
#define FAULT_FLAG_WATCHDOG_TIMEOUT     0x0040
#define FAULT_FLAG_CAN_BUS_PASSIVE      0x0080
#define FAULT_FLAG_CAN_BUS_OFF          0x0100
#define FAULT_FLAG_ESTOP_ACTIVE         0x0200

#endif /* CAN_PROTOCOL_DEF_H_ */
