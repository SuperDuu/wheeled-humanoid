"""
CAN / CAN-FD Multi-Node Simulation Package for Humanoid Robotics.
"""

from .can_protocol import (
    CAN_ID_ESTOP_BROADCAST, CAN_ID_HEARTBEAT_BASE, CAN_ID_CMD_BASE,
    CAN_ID_FEEDBACK_BASE, NodeState, PacketCategory
)
from .models import CANFrame, NodeStateModel, SimulationConfig, BusMetrics
from .can_simulator import CANSimulator

__all__ = [
    "CANSimulator", "SimulationConfig", "BusMetrics", "CANFrame",
    "NodeStateModel", "NodeState", "PacketCategory"
]
