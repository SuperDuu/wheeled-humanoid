"""
Binary Telemetry Parser and NumPy Vector Processing Engine.
Utilizes NumPy (BSD-3-Clause) for vector mathematics, Clarke/Park transformations,
RMS/Peak current tracking, and instantaneous power computation.
"""

import struct
import numpy as np
from typing import Optional, Dict, Any

# Binary Packet Format (94 bytes):
# typedef struct {
#     uint8_t  magic1;             // 0xAA (offset 0)
#     uint8_t  magic2;             // 0x55 (offset 1)
#     uint8_t  packet_type;        // 0x01 (offset 2)
#     uint8_t  payload_len;        // len  (offset 3)
#     uint32_t timestamp_ms;       //      (offset 4)
#     float    i_a, i_b, i_c;      //      (offset 8, 12, 16)
#     float    i_d, i_q, i_q_target;//     (offset 20, 24, 28)
#     float    duty_a, duty_b, duty_c;//   (offset 32, 36, 40)
#     float    phase_elec, mech_angle, joint_angle; // (offset 44, 48, 52)
#     float    speed_rpm, speed_target_rpm; // (offset 56, 60)
#     float    v_bus, temp_fet;    //      (offset 64, 68)
#     uint8_t  control_mode, motor_state, fault_code; // (offset 72, 73, 74)
#     int8_t   encoder_dir;        //      (offset 75)
#     float    vd, vq, zero_elec_angle, id_target; // (offset 76, 80, 84, 88)
#     uint16_t checksum;           //      (offset 92)
# } telemetry_packet_t; Total = 94 bytes.
PACKET_FORMAT_94 = "<BBBB I 16f 3B b 4f H"
PACKET_SIZE_94 = struct.calcsize(PACKET_FORMAT_94)

PACKET_FORMAT_78 = "<BBBB I 16f BBBB H"
PACKET_SIZE_78 = struct.calcsize(PACKET_FORMAT_78)

PACKET_SIZE = PACKET_SIZE_94
MAGIC1 = 0xAA
MAGIC2 = 0x55


class TelemetryParser:
    """High-performance parser utilizing NumPy for vector computations."""

    @staticmethod
    def calculate_checksum(buffer: bytes, length: int) -> int:
        """Calculate simple 16-bit summation checksum over packet buffer payload."""
        return sum(buffer[4:length]) & 0xFFFF

    @classmethod
    def parse_packet(cls, raw_bytes: bytes) -> Optional[Dict[str, Any]]:
        """
        Unpack binary bytes and compute vector/telemetry metrics via NumPy.
        Supports both 94-byte and legacy 78-byte packets.
        """
        if len(raw_bytes) < PACKET_SIZE_78:
            return None

        # Check magic
        if raw_bytes[0] != MAGIC1 or raw_bytes[1] != MAGIC2:
            return None

        if len(raw_bytes) >= PACKET_SIZE_94:
            unpacked = struct.unpack(PACKET_FORMAT_94, raw_bytes[:PACKET_SIZE_94])
            magic1, magic2, pkt_type, payload_len = unpacked[0:4]
            timestamp_ms = unpacked[4]
            (i_a, i_b, i_c,
             i_d, i_q, i_q_target,
             duty_a, duty_b, duty_c,
             phase_elec, mech_angle, joint_angle,
             speed_rpm, speed_target_rpm,
             v_bus, temp_fet) = unpacked[5:21]
            control_mode, motor_state, fault_code = unpacked[21:24]
            encoder_dir = unpacked[24]
            vd, vq, zero_elec_angle, id_target = unpacked[25:29]
            checksum = unpacked[29]

            expected_cs = cls.calculate_checksum(raw_bytes, PACKET_SIZE_94 - 2)
            if checksum != expected_cs:
                return None
        else:
            unpacked = struct.unpack(PACKET_FORMAT_78, raw_bytes[:PACKET_SIZE_78])
            magic1, magic2, pkt_type, payload_len = unpacked[0:4]
            timestamp_ms = unpacked[4]
            (i_a, i_b, i_c,
             i_d, i_q, i_q_target,
             duty_a, duty_b, duty_c,
             phase_elec, mech_angle, joint_angle,
             speed_rpm, speed_target_rpm,
             v_bus, temp_fet) = unpacked[5:21]
            control_mode, motor_state, fault_code, reserved = unpacked[21:25]
            encoder_dir = -1
            vd, vq, zero_elec_angle, id_target = 0.0, 0.0, 0.0, 0.0
            checksum = unpacked[25]

            expected_cs = sum(raw_bytes[:PACKET_SIZE_78 - 2]) & 0xFFFF
            if checksum != expected_cs:
                return None

        # -------------------------------------------------------------
        # NumPy Vector Mathematics & Scientific Processing
        # -------------------------------------------------------------
        i_phases = np.array([i_a, i_b, i_c], dtype=np.float32)
        i_sum = float(np.sum(i_phases))
        i_peak = float(np.max(np.abs(i_phases)))

        # Space Vector Magnitude |I| = sqrt(Id^2 + Iq^2)
        id_iq = np.array([i_d, i_q], dtype=np.float32)
        i_vector_mag = float(np.linalg.norm(id_iq))

        # Instantaneous Electrical Power: P = V_bus * I_bus ~ V_bus * sqrt(3/2)*|I|*cos(phi)
        power_watts = float(v_bus * i_vector_mag * 0.816)

        return {
            "timestamp_ms": int(timestamp_ms),
            "i_a": round(float(i_a), 4),
            "i_b": round(float(i_b), 4),
            "i_c": round(float(i_c), 4),
            "i_sum": round(i_sum, 4),
            "i_peak": round(i_peak, 4),
            "i_d": round(float(i_d), 4),
            "i_q": round(float(i_q), 4),
            "i_q_target": round(float(i_q_target), 4),
            "i_vector_mag": round(i_vector_mag, 4),
            "duty_a": round(float(duty_a), 4),
            "duty_b": round(float(duty_b), 4),
            "duty_c": round(float(duty_c), 4),
            "phase_elec": round(float(phase_elec), 4),
            "mech_angle": round(float(mech_angle), 4),
            "joint_angle": round(float(joint_angle), 4),
            "speed_rpm": round(float(speed_rpm), 2),
            "speed_target_rpm": round(float(speed_target_rpm), 2),
            "v_bus": round(float(v_bus), 2),
            "temp_fet": round(float(temp_fet), 1),
            "power_watts": round(power_watts, 2),
            "control_mode": int(control_mode),
            "motor_state": int(motor_state),
            "fault_code": int(fault_code),
            "encoder_dir": int(encoder_dir),
            "vd": round(float(vd), 4),
            "vq": round(float(vq), 4),
            "zero_elec_angle": round(float(zero_elec_angle), 4),
            "id_target": round(float(id_target), 4),
        }
