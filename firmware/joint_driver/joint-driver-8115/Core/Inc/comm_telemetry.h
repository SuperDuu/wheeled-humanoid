/**
  ******************************************************************************
  * @file    comm_telemetry.h
  * @brief   High-speed Real-Time Telemetry & Command Protocol over USB/UART
  ******************************************************************************
  */

#ifndef __COMM_TELEMETRY_H__
#define __COMM_TELEMETRY_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "foc_control.h"
#include <stdint.h>
#include <stdbool.h>

#define TELEMETRY_MAGIC_BYTE1   0xAA
#define TELEMETRY_MAGIC_BYTE2   0x55
#define TELEMETRY_PACKET_TYPE   0x01

#pragma pack(push, 1)
typedef struct {
    uint8_t  magic1;             // 0xAA
    uint8_t  magic2;             // 0x55
    uint8_t  packet_type;        // 0x01: Telemetry Data
    uint8_t  payload_len;        // Size of data payload
    uint32_t timestamp_ms;       // System uptime in ms
    
    // 3-Phase Currents (Amperes)
    float    i_a;                // Phase A current
    float    i_b;                // Phase B current
    float    i_c;                // Phase C current (-i_a - i_b)
    
    // FOC Vector Currents (Amperes)
    float    i_d;                // D-axis flux current
    float    i_q;                // Q-axis torque current
    float    i_q_target;         // Target torque current
    
    // 3-Phase Duty Cycles (0.0 to 1.0)
    float    duty_a;             // Phase A PWM Duty
    float    duty_b;             // Phase B PWM Duty
    float    duty_c;             // Phase C PWM Duty
    
    // Angles & Phase (Radians)
    float    phase_elec;         // Electrical angle (-PI to +PI)
    float    mech_angle;         // Single-turn rotor mechanical angle
    float    joint_angle;        // Multi-turn joint mechanical angle
    
    // Speeds (Mechanical RPM)
    float    speed_rpm;          // Estimated mechanical RPM
    float    speed_target_rpm;   // Target mechanical RPM
    
    // System Status
    float    v_bus;              // DC Bus Voltage (V)
    float    temp_fet;           // Inverter Temperature (deg C)
    uint8_t  control_mode;       // Current control mode (0:Idle, 1:Current, 2:Brake, 3:Speed, 4:Pos)
    uint8_t  motor_state;        // MC State
    uint8_t  fault_code;         // Fault flags
    uint8_t  reserved;
    
    uint16_t checksum;           // 16-bit XOR/Sum CRC
} telemetry_packet_t;
#pragma pack(pop)

/* Open-Loop Test Run Control Globals (Bypasses PID/Current loops) */
extern volatile uint8_t run_open_loop;
extern volatile float open_loop_target_rpm;
extern volatile float open_loop_angle;

/* Public Functions (Pure USB CDC Telemetry) */
void Comm_Telemetry_Init(void);
void Comm_Telemetry_Process(FOC_Controller_t *foc);
bool Comm_Telemetry_Send(FOC_Controller_t *foc);
void Comm_Telemetry_RxByte(uint8_t rx_byte);
void Comm_Telemetry_RxBuffer(const uint8_t *buf, uint32_t len);

#ifdef __cplusplus
}
#endif

#endif /* __COMM_TELEMETRY_H__ */
