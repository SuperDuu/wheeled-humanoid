"""
Data Models for Multi-Node CAN & CAN-FD Humanoid Robotics Simulation.
"""

from dataclasses import dataclass, field
from typing import List, Dict, Optional
import time

from .can_protocol import NodeState, PacketCategory, get_packet_category

@dataclass
class CANFrame:
    can_id: int
    data: bytes
    is_fd: bool = False
    is_brs: bool = False             # Bit Rate Switch active in data phase
    sender_id: int = 0
    target_id: int = 0xFF            # Broadcast by default
    category: PacketCategory = PacketCategory.COMMAND
    seq_num: int = 0
    
    # Accurate Physical Timestamps (Microsecond resolution)
    time_queued_us: float = 0.0
    time_arbitration_win_us: float = 0.0
    time_delivered_us: float = 0.0
    
    # Physical Bit Timing Breakdown
    bit_count_nominal: int = 0
    bit_count_data: int = 0
    duration_us: float = 0.0
    
    # State tracking
    retries: int = 0
    is_corrupted: bool = False
    is_error_frame: bool = False
    
    def __post_init__(self):
        self.category = get_packet_category(self.can_id)
        
    @property
    def queue_latency_us(self) -> float:
        return max(0.0, self.time_arbitration_win_us - self.time_queued_us)
        
    @property
    def end_to_end_latency_us(self) -> float:
        return max(0.0, self.time_delivered_us - self.time_queued_us)

@dataclass
class NodeStateModel:
    node_id: int
    name: str
    state: NodeState = NodeState.RUNNING
    is_online: bool = True
    
    # Joint Dynamics State
    p_actual: float = 0.0       # rad
    v_actual: float = 0.0       # rad/s
    tau_actual: float = 0.0     # Nm
    iq_actual: float = 0.0      # Amperes
    vbus: float = 24.0          # Volts
    temp_fet: float = 28.5      # Celsius
    temp_motor: float = 31.0    # Celsius
    
    # Comm / Safety Counters
    last_heartbeat_recv_s: float = 0.0
    time_since_heartbeat_ms: float = 0.0
    tec: int = 0                # Transmit Error Counter
    rec: int = 0                # Receive Error Counter
    packets_sent: int = 0
    packets_received: int = 0
    packets_dropped: int = 0
    crc_errors: int = 0
    watchdog_trips: int = 0
    
    # Control Target Buffers
    p_des: float = 0.0
    v_des: float = 0.0
    kp: float = 50.0
    kd: float = 2.5
    t_ff: float = 0.0

@dataclass
class SimulationConfig:
    protocol: str = "CANFD"         # "CAN20" or "CANFD"
    nominal_baud: int = 1_000_000   # 1 Mbps Nominal
    data_baud: int = 5_000_000      # 5 Mbps Data Phase (BRS)
    num_nodes: int = 8              # 8 joints (7-DOF arm + Gripper)
    control_rate_hz: int = 1000     # 1 kHz control loop
    heartbeat_rate_hz: int = 50     # 50 Hz Heartbeat
    packet_loss_rate: float = 0.0   # 0.0 to 0.5 (0% to 50%)
    emi_noise_ber: float = 0.0      # Bit Error Rate
    burst_error_active: bool = False
    is_running: bool = True
    is_estop_active: bool = False
    
@dataclass
class BusMetrics:
    timestamp_s: float = 0.0
    bus_load_pct: float = 0.0
    bus_load_peak_pct: float = 0.0
    throughput_kbps: float = 0.0
    frames_per_sec: float = 0.0
    
    total_transmitted: int = 0
    total_delivered: int = 0
    total_dropped: int = 0
    total_error_frames: int = 0
    loss_rate_pct: float = 0.0
    
    # Latency Percentiles (us)
    latency_min_us: float = 0.0
    latency_avg_us: float = 0.0
    latency_p95_us: float = 0.0
    latency_p99_us: float = 0.0
    latency_max_us: float = 0.0
    latency_jitter_us: float = 0.0
    
    # Specific Latencies
    estop_latency_us: float = 0.0
    heartbeat_avg_latency_us: float = 0.0
    command_avg_latency_us: float = 0.0
    telemetry_avg_latency_us: float = 0.0
    
    # Safety Status
    active_nodes_healthy: int = 0
    active_nodes_warning: int = 0
    active_nodes_tripped: int = 0
