"""
CAN & CAN-FD Protocol Definitions and Message Serializers
Matches firmware/joint_driver/joint-driver-8115/Core/Inc/can_protocol_def.h
Supports Classical CAN 2.0B (8 bytes) & CAN-FD (up to 64 bytes).
"""

import struct
from enum import IntEnum
from typing import NamedTuple, Tuple, Optional

# Protocol Priority Identifiers (11-bit Standard CAN IDs)
CAN_ID_ESTOP_BROADCAST    = 0x001  # Priority 0: Global Emergency Stop
CAN_ID_ESTOP_ACK          = 0x002
CAN_ID_HEARTBEAT_BASE     = 0x010  # Priority 1: Heartbeat (0x010 + Node_ID)
CAN_ID_FAULT_BASE         = 0x040  # Priority 2: Fault Notification (0x040 + Node_ID)
CAN_ID_CMD_BASE           = 0x100  # Priority 3: Joint Commands (0x100 + Node_ID)
CAN_ID_FEEDBACK_BASE      = 0x200  # Priority 4: Joint Telemetry Feedback (0x200 + Node_ID)
CAN_ID_DIAG_BASE          = 0x700  # Priority 5: Diagnostics (0x700 + Node_ID)

# Node States
class NodeState(IntEnum):
    UNINITIALIZED = 0
    STANDBY       = 1
    RUNNING       = 2
    WARNING       = 3
    FAILSAFE_HOLD = 4   # Mất heartbeat hoặc lỗi bus -> Chế độ giữ vị trí an toàn với damping
    ESTOP_TRIPPED = 5   # Dập tắt xung PWM ngay lập tức, ngắt mô-men, khóa phanh
    BUS_OFF       = 6   # FDCAN controller rơi vào Bus-Off (TEC > 255)

# Fault Bitmask Flags
class FaultFlags(IntEnum):
    NONE             = 0x0000
    OVER_CURRENT     = 0x0001
    OVER_VOLTAGE     = 0x0002
    UNDER_VOLTAGE    = 0x0004
    OVER_TEMP_MOSFET = 0x0008
    OVER_TEMP_MOTOR  = 0x0010
    ENCODER_CRC      = 0x0020
    WATCHDOG_TIMEOUT = 0x0040
    CAN_BUS_PASSIVE  = 0x0080
    CAN_BUS_OFF      = 0x0100
    ESTOP_ACTIVE     = 0x0200

# Packet Category for Priority Handling & Coloring
class PacketCategory(IntEnum):
    ESTOP    = 0
    HEARTBEAT= 1
    FAULT    = 2
    COMMAND  = 3
    FEEDBACK = 4
    DIAG     = 5

# Joint Names for Humanoid Arm Topology
JOINT_NAMES = {
    1: "J1 Shoulder Pitch",
    2: "J2 Shoulder Roll",
    3: "J3 Shoulder Yaw",
    4: "J4 Elbow Pitch",
    5: "J5 Wrist Yaw",
    6: "J6 Wrist Pitch",
    7: "J7 Wrist Roll",
    8: "J8 Gripper / Tool",
    9: "J9 L-Shoulder Pitch",
    10: "J10 L-Shoulder Roll",
    11: "J11 L-Shoulder Yaw",
    12: "J12 L-Elbow Pitch",
    13: "J13 L-Wrist Yaw",
    14: "J14 L-Wrist Pitch",
    15: "J15 L-Wrist Roll",
    16: "J16 L-Gripper",
}

def get_packet_category(can_id: int) -> PacketCategory:
    if can_id in (CAN_ID_ESTOP_BROADCAST, CAN_ID_ESTOP_ACK):
        return PacketCategory.ESTOP
    elif 0x010 <= can_id <= 0x03F:
        return PacketCategory.HEARTBEAT
    elif 0x040 <= can_id <= 0x07F:
        return PacketCategory.FAULT
    elif 0x100 <= can_id <= 0x17F:
        return PacketCategory.COMMAND
    elif 0x200 <= can_id <= 0x27F:
        return PacketCategory.FEEDBACK
    else:
        return PacketCategory.DIAG

# Packing Helpers
def pack_estop_payload(source_id: int, reason_code: int = 1) -> bytes:
    """4-byte Emergency Stop Broadcast Frame."""
    return struct.pack(">BBH", source_id, reason_code, 0xAA55)

def pack_heartbeat_payload(node_id: int, state: NodeState, fault_bits: int, uptime_ms: int) -> bytes:
    """8-byte Heartbeat Frame (50 Hz)."""
    return struct.pack(">BBHI", node_id, int(state), fault_bits, uptime_ms & 0xFFFFFFFF)

def pack_mit_cmd_payload(p_des: float, v_des: float, kp: float, kd: float, t_ff: float) -> bytes:
    """
    Standard MIT 8-byte Impedance Control Payload:
    P: 16-bit [-12.5, +12.5]
    V: 12-bit [-45.0, +45.0]
    Kp: 12-bit [0, 500]
    Kd: 12-bit [0, 5]
    T_ff: 12-bit [-18, +18]
    """
    def f2u(val, vmin, vmax, bits):
        val = max(vmin, min(vmax, val))
        return int((val - vmin) * ((1 << bits) - 1) / (vmax - vmin))

    p_int  = f2u(p_des, -12.5, +12.5, 16)
    v_int  = f2u(v_des, -45.0, +45.0, 12)
    kp_int = f2u(kp, 0.0, 500.0, 12)
    kd_int = f2u(kd, 0.0, 5.0, 12)
    t_int  = f2u(t_ff, -18.0, +18.0, 12)

    b0 = (p_int >> 8) & 0xFF
    b1 = p_int & 0xFF
    b2 = (v_int >> 4) & 0xFF
    b3 = ((v_int & 0x0F) << 4) | ((kp_int >> 8) & 0x0F)
    b4 = kp_int & 0xFF
    b5 = (kd_int >> 4) & 0xFF
    b6 = ((kd_int & 0x0F) << 4) | ((t_int >> 8) & 0x0F)
    b7 = t_int & 0xFF
    return bytes([b0, b1, b2, b3, b4, b5, b6, b7])

def pack_canfd_telemetry_payload(node_id: int, p_act: float, v_act: float, tau_act: float,
                                 iq_act: float, vbus: float, t_fet: float, t_motor: float,
                                 fault_bits: int, seq_num: int) -> bytes:
    """
    CAN-FD High-Density 32-Byte Joint State Frame (fits comfortably in 32B or 64B DLC):
    Contains full high-precision float state without lossy 12-bit compression.
    """
    # Header: node_id (B), seq_num (H), state (B) = 4B
    # Telemetry: p_act (f), v_act (f), tau_act (f), iq_act (f), vbus (f), t_fet (f), t_motor (f) = 28B
    # Total = 32 Bytes
    return struct.pack(">BHBfffffff",
                       node_id, seq_num & 0xFFFF, 2, # running state
                       p_act, v_act, tau_act, iq_act, vbus, t_fet, t_motor)
