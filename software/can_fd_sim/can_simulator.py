"""
CAN / CAN-FD Discrete-Event Physics & Network Simulation Engine.
Implements ISO 11898-1:2015 Physical Timing, Bitwise Arbitration,
Multi-Node Controller State Machines, E-Stop Preemption, and Watchdog Monitoring.
"""

import math
import random
import time
from collections import deque
from typing import List, Dict, Tuple, Optional, Deque

from .can_protocol import (
    CAN_ID_ESTOP_BROADCAST, CAN_ID_ESTOP_ACK, CAN_ID_HEARTBEAT_BASE,
    CAN_ID_CMD_BASE, CAN_ID_FEEDBACK_BASE, CAN_ID_FAULT_BASE,
    NodeState, PacketCategory, JOINT_NAMES,
    pack_estop_payload, pack_heartbeat_payload, pack_mit_cmd_payload,
    pack_canfd_telemetry_payload
)
from .models import CANFrame, NodeStateModel, SimulationConfig, BusMetrics

class CANSimulator:
    def __init__(self, config: Optional[SimulationConfig] = None):
        self.config = config or SimulationConfig()
        
        # Virtual Simulation Time (Microseconds)
        self.sim_time_us: float = 0.0
        self.real_start_wall_s: float = time.time()
        self.sim_start_time_us: float = 0.0
        
        # Bus Physical State
        self.is_bus_busy: bool = False
        self.current_transmitting_frame: Optional[CANFrame] = None
        self.bus_busy_until_us: float = 0.0
        
        # Priority Queues & History
        self.tx_queue: List[CANFrame] = []  # Distributed pending TX buffers from all nodes
        self.recent_delivered_frames: Deque[CANFrame] = deque(maxlen=200)
        self.recent_events: Deque[Dict] = deque(maxlen=100)
        
        # Nodes (1..16)
        self.nodes: Dict[int, NodeStateModel] = {}
        self._init_nodes()
        
        # Metrics & Latency Profiling
        self.metrics = BusMetrics()
        self.latencies_all_us: Deque[float] = deque(maxlen=5000)
        self.latencies_estop_us: Deque[float] = deque(maxlen=200)
        self.latencies_hb_us: Deque[float] = deque(maxlen=1000)
        self.latencies_cmd_us: Deque[float] = deque(maxlen=2000)
        self.latencies_telemetry_us: Deque[float] = deque(maxlen=2000)
        
        # Bus Load Tracking (Sliding Window of 50 ms = 50,000 us)
        self.window_size_us: float = 50_000.0
        self.window_bus_active_us: float = 0.0
        self.window_start_us: float = 0.0
        self.window_bits_count: int = 0
        
        # Periodic Scheduling Clocks
        self.last_cmd_time_us: float = 0.0
        self.last_hb_time_us: float = 0.0
        self.last_metrics_calc_us: float = 0.0
        self.seq_counter: int = 0
        
        # Endurance & Failsafe Audit
        self.total_test_duration_s: float = 0.0
        self.uncontrolled_runaway_detected: bool = False
        self.failsafe_transitions_count: int = 0

    def _init_nodes(self):
        """Initialize humanoid arm actuator nodes."""
        self.nodes.clear()
        for node_id in range(1, self.config.num_nodes + 1):
            name = JOINT_NAMES.get(node_id, f"Node {node_id}")
            self.nodes[node_id] = NodeStateModel(
                node_id=node_id,
                name=name,
                state=NodeState.RUNNING,
                is_online=True,
                p_actual=0.0,
                v_actual=0.0,
                tau_actual=0.0,
                iq_actual=0.0,
                vbus=24.0,
                temp_fet=28.0 + random.uniform(0.0, 1.5),
                temp_motor=30.0 + random.uniform(0.0, 2.0),
                last_heartbeat_recv_s=self.sim_time_us / 1e6
            )

    def calculate_frame_physical_timing(self, frame: CANFrame) -> Tuple[int, int, float]:
        """
        Calculate nominal bits, data bits, and physical wire duration (microseconds).
        Follows ISO 11898-1:2015 specifications accurately.
        """
        dlc = len(frame.data)
        
        if not frame.is_fd or not frame.is_brs:
            # Classical CAN 2.0B (11-bit standard ID)
            # Unstuffed: SOF(1) + ID(11) + RTR(1) + IDE(1) + r0(1) + DLC(4) + Data(8*DLC) + CRC(15) + CRCDel(1) + ACK(1) + ACKDel(1) + EOF(7) + IFS(3)
            unstuffed = 47 + 8 * dlc
            # Average Stuff bits = (34 + 8*DLC) / 5
            stuff_bits = int(round((34 + 8 * dlc) / 5.0))
            total_nominal_bits = unstuffed + stuff_bits
            total_data_bits = 0
            
            # Duration in us
            duration_us = (total_nominal_bits / self.config.nominal_baud) * 1e6
            return total_nominal_bits, total_data_bits, duration_us
            
        else:
            # CAN-FD with Bit Rate Switch (BRS)
            # Nominal Phase: SOF(1) + ID(11) + RRS(1) + IDE(1) + FDF(1) + res(1) + BRS(1) + ESI(1) = 18 bits (start)
            # + ACKDel(1) + EOF(7) + IFS(3) = 11 bits (end) -> Total Nominal = 29 bits
            nominal_bits = 29
            
            # Fast Data Phase: DLC(4) + Data(8*DLC) + StuffCount(4) + CRC + FixedStuff + ACK(1)
            # CRC-17 for DLC <= 16 with 6 fixed stuff bits; CRC-21 for DLC > 16 with 7 fixed stuff bits
            crc_len = 21 if dlc > 16 else 17
            fixed_stuff = 7 if dlc > 16 else 6
            data_bits = 4 + (8 * dlc) + 4 + crc_len + fixed_stuff + 1
            
            duration_nominal_us = (nominal_bits / self.config.nominal_baud) * 1e6
            duration_data_us = (data_bits / self.config.data_baud) * 1e6
            total_duration_us = duration_nominal_us + duration_data_us
            
            return nominal_bits, data_bits, total_duration_us

    def enqueue_frame(self, frame: CANFrame):
        """
        Enqueue a CAN frame from a transmitting node.
        Implements real hardware FDCAN TX Mailbox semantics:
        - Freshest state replacement for cyclic telemetry/commands.
        - Strict capacity bound (max 64 frames on bus), dropping lowest-priority on overflow.
        """
        # 1. State Replacement for Periodic Streams (Commands and Telemetry)
        if frame.category in (PacketCategory.COMMAND, PacketCategory.FEEDBACK):
            for idx, existing in enumerate(self.tx_queue):
                if existing.can_id == frame.can_id:
                    self.metrics.total_dropped += 1
                    self.tx_queue.pop(idx)
                    break

        # 2. Hardware TX FIFO Overflow Drop (Lowest priority dropped first)
        if len(self.tx_queue) >= 64:
            self.tx_queue.sort(key=lambda f: f.can_id)
            dropped = self.tx_queue.pop(-1)
            self.metrics.total_dropped += 1
            sender = self.nodes.get(dropped.sender_id)
            if sender:
                sender.packets_dropped += 1

        frame.time_queued_us = self.sim_time_us
        frame.seq_num = self.seq_counter
        self.seq_counter += 1
        
        # Calculate bit lengths
        nom, dat, dur = self.calculate_frame_physical_timing(frame)
        frame.bit_count_nominal = nom
        frame.bit_count_data = dat
        frame.duration_us = dur
        
        self.tx_queue.append(frame)

    def trigger_estop(self, source_id: int = 0, reason: str = "Operator Triggered E-Stop"):
        """Instantaneous Emergency Stop trigger."""
        payload = pack_estop_payload(source_id, 1)
        estop_frame = CANFrame(
            can_id=CAN_ID_ESTOP_BROADCAST,
            data=payload,
            is_fd=False,  # Classical standard frame ensures universal hardware compatibility
            is_brs=False,
            sender_id=source_id,
            target_id=0xFF
        )
        self.config.is_estop_active = True
        self.enqueue_frame(estop_frame)
        
        self.recent_events.append({
            "timestamp": round(self.sim_time_us / 1e6, 4),
            "type": "ESTOP_TRIGGERED",
            "message": f"🚨 {reason} (Source: Node {source_id})"
        })

    def reset_estop(self):
        """Reset E-Stop and return nodes to STANDBY."""
        self.config.is_estop_active = False
        for node in self.nodes.values():
            if node.is_online and node.state == NodeState.ESTOP_TRIPPED:
                node.state = NodeState.STANDBY
        self.recent_events.append({
            "timestamp": round(self.sim_time_us / 1e6, 4),
            "type": "ESTOP_RESET",
            "message": "✅ E-Stop released. System in STANDBY."
        })

    def disconnect_node(self, node_id: int):
        """Simulate physical disconnection or power loss of a joint node."""
        if node_id in self.nodes:
            node = self.nodes[node_id]
            node.is_online = False
            node.state = NodeState.UNINITIALIZED
            self.recent_events.append({
                "timestamp": round(self.sim_time_us / 1e6, 4),
                "type": "NODE_LOST",
                "message": f"❌ Node {node_id} ({node.name}) disconnected!"
            })

    def reconnect_node(self, node_id: int):
        """Reconnect a previously disconnected node."""
        if node_id in self.nodes:
            node = self.nodes[node_id]
            node.is_online = True
            node.state = NodeState.STANDBY
            node.last_heartbeat_recv_s = self.sim_time_us / 1e6
            self.recent_events.append({
                "timestamp": round(self.sim_time_us / 1e6, 4),
                "type": "NODE_ONLINE",
                "message": f"🟢 Node {node_id} ({node.name}) reconnected."
            })

    def step(self, delta_us: float):
        """
        Advance simulation by delta_us microseconds using high-performance discrete-event jumping.
        """
        target_time_us = self.sim_time_us + delta_us
        
        while self.sim_time_us < target_time_us:
            # 1. Periodic Cyclic Traffic Generation
            self._generate_cyclic_traffic()
            
            # 2. Process bus or jump to next event
            if self.is_bus_busy and self.bus_busy_until_us > self.sim_time_us:
                jump_to = min(self.bus_busy_until_us, target_time_us)
                elapsed = max(1.0, jump_to - self.sim_time_us)
                self.sim_time_us = jump_to
                self.total_test_duration_s = self.sim_time_us / 1e6
                self._update_nodes_physics(elapsed)
                if self.sim_time_us >= self.bus_busy_until_us:
                    self._process_bus_channel()
            elif self.tx_queue:
                # Bus is idle and queue has items -> arbitrate immediately
                self._process_bus_channel()
            else:
                # Bus is idle and no queue -> jump to next cyclic event
                cmd_interval_us = 1e6 / max(10, self.config.control_rate_hz)
                next_cmd = self.last_cmd_time_us + cmd_interval_us
                jump_to = min(next_cmd, target_time_us)
                elapsed = max(1.0, jump_to - self.sim_time_us)
                self.sim_time_us = jump_to
                self.total_test_duration_s = self.sim_time_us / 1e6
                self._update_nodes_physics(elapsed)

        # Update rolling bus load & metrics
        if self.sim_time_us - self.last_metrics_calc_us >= 20_000.0: # Every 20 ms
            self._recalculate_bus_metrics()
            self.last_metrics_calc_us = self.sim_time_us

    def _generate_cyclic_traffic(self):
        """Generate high-rate joint commands (1kHz), joint telemetry (1kHz), and heartbeats (50Hz)."""
        is_fd = (self.config.protocol == "CANFD")
        
        # A. Motion Commands from Master to Joints (Default 1 kHz = every 1000 us)
        cmd_interval_us = 1e6 / max(10, self.config.control_rate_hz)
        if self.sim_time_us - self.last_cmd_time_us >= cmd_interval_us:
            self.last_cmd_time_us = self.sim_time_us
            
            if not self.config.is_estop_active:
                for node_id, node in self.nodes.items():
                    if not node.is_online:
                        continue
                        
                    # Target trajectory (smooth sinusoidal movement for humanoid arm)
                    t = self.sim_time_us / 1e6
                    p_target = 0.5 * math.sin(2.0 * math.pi * 0.5 * t + node_id * 0.4)
                    v_target = 0.5 * 2.0 * math.pi * 0.5 * math.cos(2.0 * math.pi * 0.5 * t + node_id * 0.4)
                    
                    cmd_payload = pack_mit_cmd_payload(p_target, v_target, 50.0, 2.5, 0.0)
                    cmd_frame = CANFrame(
                        can_id=CAN_ID_CMD_BASE + node_id,
                        data=cmd_payload,
                        is_fd=is_fd,
                        is_brs=is_fd,
                        sender_id=0,
                        target_id=node_id
                    )
                    self.enqueue_frame(cmd_frame)

        # B. Heartbeat Packets (50 Hz = every 20,000 us)
        hb_interval_us = 1e6 / max(1, self.config.heartbeat_rate_hz)
        if self.sim_time_us - self.last_hb_time_us >= hb_interval_us:
            self.last_hb_time_us = self.sim_time_us
            
            for node_id, node in self.nodes.items():
                if not node.is_online:
                    continue
                fault_bits = 0
                if node.state == NodeState.FAILSAFE_HOLD:
                    fault_bits |= 0x0040
                elif node.state == NodeState.ESTOP_TRIPPED:
                    fault_bits |= 0x0200
                    
                hb_payload = pack_heartbeat_payload(node_id, node.state, fault_bits, int(self.sim_time_us / 1000))
                hb_frame = CANFrame(
                    can_id=CAN_ID_HEARTBEAT_BASE + node_id,
                    data=hb_payload,
                    is_fd=is_fd,
                    is_brs=is_fd,
                    sender_id=node_id,
                    target_id=0
                )
                self.enqueue_frame(hb_frame)

    def _process_bus_channel(self):
        """
        Simulates physical wire:
        - If bus is busy transmitting, check if current frame has completed.
        - If bus is idle (or just finished), perform Non-Destructive Bitwise Arbitration immediately.
        """
        if self.is_bus_busy:
            if self.sim_time_us >= self.bus_busy_until_us:
                # Transmission finished!
                frame = self.current_transmitting_frame
                self.is_bus_busy = False
                self.current_transmitting_frame = None
                
                if frame:
                    self._deliver_frame(frame)
            else:
                return

        # Bus is IDLE. If we have queued frames, perform bitwise arbitration immediately.
        if not self.tx_queue:
            return

        # NON-DESTRUCTIVE BITWISE ARBITRATION (Wired-AND: Lowest CAN ID wins)
        # Sort queue by CAN ID ascending (lowest numerical ID wins)
        self.tx_queue.sort(key=lambda f: f.can_id)
        
        # Winner frame is the first element
        winner_frame = self.tx_queue.pop(0)
        winner_frame.time_arbitration_win_us = self.sim_time_us
        
        # Check for injected noise / bit error or forced packet drop
        is_corrupted = False
        if self.config.packet_loss_rate > 0.0:
            if random.random() < self.config.packet_loss_rate:
                is_corrupted = True
        
        if not is_corrupted and self.config.emi_noise_ber > 0.0:
            # Bit error rate per bit transmitted
            total_bits = winner_frame.bit_count_nominal + winner_frame.bit_count_data
            error_prob = 1.0 - math.pow(1.0 - self.config.emi_noise_ber, total_bits)
            if random.random() < error_prob:
                is_corrupted = True

        winner_frame.is_corrupted = is_corrupted
        
        # Occupy bus for the physical duration
        self.is_bus_busy = True
        self.current_transmitting_frame = winner_frame
        
        if is_corrupted:
            # Generates Active Error Frame on bus (17 nominal bits = ~17us) + frame duration
            error_frame_dur = (17.0 / self.config.nominal_baud) * 1e6
            self.bus_busy_until_us = self.sim_time_us + winner_frame.duration_us + error_frame_dur
            self.metrics.total_error_frames += 1
            self.window_bus_active_us += winner_frame.duration_us + error_frame_dur
        else:
            self.bus_busy_until_us = self.sim_time_us + winner_frame.duration_us
            self.window_bus_active_us += winner_frame.duration_us

        self.window_bits_count += (winner_frame.bit_count_nominal + winner_frame.bit_count_data)

    def _deliver_frame(self, frame: CANFrame):
        """Process successful delivery or error retransmission."""
        frame.time_delivered_us = self.sim_time_us
        e2e_latency = frame.end_to_end_latency_us
        
        if frame.is_corrupted:
            # Packet suffered error
            self.metrics.total_dropped += 1
            sender = self.nodes.get(frame.sender_id)
            if sender:
                sender.crc_errors += 1
                sender.tec += 8  # FDCAN TEC increment on TX error
                if sender.tec > 255:
                    sender.state = NodeState.BUS_OFF
                    self.recent_events.append({
                        "timestamp": round(self.sim_time_us / 1e6, 4),
                        "type": "NODE_BUS_OFF",
                        "message": f"⚠️ Node {sender.node_id} entered BUS-OFF state (TEC={sender.tec})"
                    })
            
            # Requeue if retries remain (< 8 retries)
            if frame.retries < 8 and (sender is None or sender.state != NodeState.BUS_OFF):
                frame.retries += 1
                frame.is_corrupted = False
                self.tx_queue.append(frame)
            return

        # Frame delivered successfully!
        self.metrics.total_delivered += 1
        self.recent_delivered_frames.append(frame)
        self.latencies_all_us.append(e2e_latency)
        
        # Categorized Latency Tracking
        if frame.category == PacketCategory.ESTOP:
            self.latencies_estop_us.append(e2e_latency)
            self.metrics.estop_latency_us = e2e_latency
            # All active nodes receive E-stop immediately!
            for n in self.nodes.values():
                if n.is_online:
                    n.state = NodeState.ESTOP_TRIPPED
                    n.iq_actual = 0.0
                    n.tau_actual = 0.0
            self.recent_events.append({
                "timestamp": round(self.sim_time_us / 1e6, 4),
                "type": "ESTOP_DELIVERED",
                "message": f"🛑 E-Stop delivered to ALL nodes! Latency: {e2e_latency:.1f} µs"
            })
            
        elif frame.category == PacketCategory.HEARTBEAT:
            self.latencies_hb_us.append(e2e_latency)
            sender = self.nodes.get(frame.sender_id)
            if sender:
                sender.last_heartbeat_recv_s = self.sim_time_us / 1e6
                sender.time_since_heartbeat_ms = 0.0
                if sender.state == NodeState.WARNING:
                    sender.state = NodeState.RUNNING
                    
        elif frame.category == PacketCategory.COMMAND:
            self.latencies_cmd_us.append(e2e_latency)
            target = self.nodes.get(frame.target_id)
            if target and target.is_online:
                target.packets_received += 1
                
                # Upon receiving command, the joint responds with telemetry feedback!
                is_fd = (self.config.protocol == "CANFD")
                if is_fd:
                    # CAN-FD dense 32-byte state
                    fb_data = pack_canfd_telemetry_payload(
                        target.node_id, target.p_actual, target.v_actual, target.tau_actual,
                        target.iq_actual, target.vbus, target.temp_fet, target.temp_motor,
                        0, self.seq_counter
                    )
                else:
                    # Classical CAN 2.0B 8-byte response
                    fb_data = pack_mit_cmd_payload(target.p_actual, target.v_actual, 0, 0, target.tau_actual)
                    
                fb_frame = CANFrame(
                    can_id=CAN_ID_FEEDBACK_BASE + target.node_id,
                    data=fb_data,
                    is_fd=is_fd,
                    is_brs=is_fd,
                    sender_id=target.node_id,
                    target_id=0
                )
                self.enqueue_frame(fb_frame)
                
        elif frame.category == PacketCategory.FEEDBACK:
            self.latencies_telemetry_us.append(e2e_latency)

    def _update_nodes_physics(self, dt_us: float):
        """Update joint mechanics, temperature model, and watchdog timer."""
        dt_s = dt_us / 1e6
        now_s = self.sim_time_us / 1e6
        
        for node_id, node in self.nodes.items():
            if not node.is_online:
                continue
                
            # Watchdog Timer: Time since last received heartbeat
            node.time_since_heartbeat_ms = (now_s - node.last_heartbeat_recv_s) * 1000.0
            
            # Watchdog 3-Tier Safety Logic
            if node.time_since_heartbeat_ms > 100.0: # > 100 ms (5 missed heartbeats)
                if node.state not in (NodeState.FAILSAFE_HOLD, NodeState.ESTOP_TRIPPED, NodeState.BUS_OFF):
                    node.state = NodeState.FAILSAFE_HOLD
                    node.watchdog_trips += 1
                    self.failsafe_transitions_count += 1
                    self.recent_events.append({
                        "timestamp": round(now_s, 4),
                        "type": "WATCHDOG_TRIP",
                        "message": f"⚠️ WATCHDOG TRIP on Node {node_id} ({node.time_since_heartbeat_ms:.1f}ms without HB)! Entering FAILSAFE_HOLD."
                    })
            elif node.time_since_heartbeat_ms > 50.0:
                if node.state == NodeState.RUNNING:
                    node.state = NodeState.WARNING

            # Joint Physical Dynamics
            if node.state == NodeState.RUNNING:
                # Normal tracking motion
                t = now_s
                p_target = 0.5 * math.sin(2.0 * math.pi * 0.5 * t + node_id * 0.4)
                v_target = 0.5 * 2.0 * math.pi * 0.5 * math.cos(2.0 * math.pi * 0.5 * t + node_id * 0.4)
                
                # Closed loop tracking with small inertia lag
                node.p_actual += (p_target - node.p_actual) * min(1.0, dt_s * 25.0)
                node.v_actual += (v_target - node.v_actual) * min(1.0, dt_s * 25.0)
                node.tau_actual = 2.5 * math.sin(node.p_actual) + 0.1 * node.v_actual
                node.iq_actual = node.tau_actual / 0.85 # Motor Kt
                
            elif node.state == NodeState.FAILSAFE_HOLD:
                # Failsafe Hold: Active damping, velocity decays to 0, hold position stably without runaway!
                node.v_actual *= max(0.0, 1.0 - dt_s * 15.0)
                node.iq_actual *= max(0.0, 1.0 - dt_s * 10.0)
                node.tau_actual *= max(0.0, 1.0 - dt_s * 10.0)
                
            elif node.state in (NodeState.ESTOP_TRIPPED, NodeState.BUS_OFF):
                # E-stop / Off: Power cut immediately
                node.v_actual *= max(0.0, 1.0 - dt_s * 30.0)
                node.iq_actual = 0.0
                node.tau_actual = 0.0

            # Thermal model (MOSFET and motor heat under current)
            loss_power = (node.iq_actual ** 2) * 0.05
            node.temp_fet += (loss_power * 0.1 - (node.temp_fet - 25.0) * 0.01) * dt_s
            node.temp_motor += (loss_power * 0.08 - (node.temp_motor - 25.0) * 0.005) * dt_s

    def _recalculate_bus_metrics(self):
        """Update bus load percentage, throughput, and latency percentiles."""
        dt_us = max(1.0, self.sim_time_us - self.window_start_us)
        
        # Bus Load (%) = (Active transmission time on wire / Window duration) * 100
        raw_load = (self.window_bus_active_us / dt_us) * 100.0
        self.metrics.bus_load_pct = min(100.0, max(0.0, raw_load))
        if self.metrics.bus_load_pct > self.metrics.bus_load_peak_pct:
            self.metrics.bus_load_peak_pct = self.metrics.bus_load_pct
            
        # Throughput in kbps
        self.metrics.throughput_kbps = (self.window_bits_count / (dt_us / 1e6)) / 1000.0
        
        # Reset window
        self.window_start_us = self.sim_time_us
        self.window_bus_active_us = 0.0
        self.window_bits_count = 0
        
        # Latency Percentiles
        if self.latencies_all_us:
            sorted_lat = sorted(self.latencies_all_us)
            n = len(sorted_lat)
            self.metrics.latency_min_us = sorted_lat[0]
            self.metrics.latency_avg_us = sum(sorted_lat) / n
            self.metrics.latency_p95_us = sorted_lat[min(n - 1, int(n * 0.95))]
            self.metrics.latency_p99_us = sorted_lat[min(n - 1, int(n * 0.99))]
            self.metrics.latency_max_us = sorted_lat[-1]
            # Standard deviation (Jitter)
            avg = self.metrics.latency_avg_us
            variance = sum((x - avg) ** 2 for x in sorted_lat) / n
            self.metrics.latency_jitter_us = math.sqrt(variance)

        if self.latencies_estop_us:
            self.metrics.estop_latency_us = self.latencies_estop_us[-1]
        if self.latencies_hb_us:
            self.metrics.heartbeat_avg_latency_us = sum(self.latencies_hb_us) / len(self.latencies_hb_us)
        if self.latencies_cmd_us:
            self.metrics.command_avg_latency_us = sum(self.latencies_cmd_us) / len(self.latencies_cmd_us)
        if self.latencies_telemetry_us:
            self.metrics.telemetry_avg_latency_us = sum(self.latencies_telemetry_us) / len(self.latencies_telemetry_us)
            
        # Node Health Tallies
        self.metrics.active_nodes_healthy = sum(1 for n in self.nodes.values() if n.is_online and n.state == NodeState.RUNNING)
        self.metrics.active_nodes_warning = sum(1 for n in self.nodes.values() if n.is_online and n.state in (NodeState.WARNING, NodeState.STANDBY))
        self.metrics.active_nodes_tripped = sum(1 for n in self.nodes.values() if not n.is_online or n.state in (NodeState.FAILSAFE_HOLD, NodeState.ESTOP_TRIPPED, NodeState.BUS_OFF))

        # Overall loss rate
        total = self.metrics.total_delivered + self.metrics.total_dropped
        self.metrics.loss_rate_pct = (self.metrics.total_dropped / total * 100.0) if total > 0 else 0.0

    def get_snapshot(self) -> Dict:
        """Serialize current simulator state for WebSocket and REST API."""
        return {
            "sim_time_s": round(self.sim_time_us / 1e6, 4),
            "config": {
                "protocol": self.config.protocol,
                "nominal_baud": self.config.nominal_baud,
                "data_baud": self.config.data_baud,
                "num_nodes": self.config.num_nodes,
                "control_rate_hz": self.config.control_rate_hz,
                "heartbeat_rate_hz": self.config.heartbeat_rate_hz,
                "packet_loss_rate": self.config.packet_loss_rate,
                "emi_noise_ber": self.config.emi_noise_ber,
                "is_estop_active": self.config.is_estop_active,
            },
            "metrics": {
                "bus_load_pct": round(self.metrics.bus_load_pct, 1),
                "bus_load_peak_pct": round(self.metrics.bus_load_peak_pct, 1),
                "throughput_kbps": round(self.metrics.throughput_kbps, 1),
                "total_delivered": self.metrics.total_delivered,
                "total_dropped": self.metrics.total_dropped,
                "total_error_frames": self.metrics.total_error_frames,
                "loss_rate_pct": round(self.metrics.loss_rate_pct, 2),
                "latency_min_us": round(self.metrics.latency_min_us, 1),
                "latency_avg_us": round(self.metrics.latency_avg_us, 1),
                "latency_p95_us": round(self.metrics.latency_p95_us, 1),
                "latency_p99_us": round(self.metrics.latency_p99_us, 1),
                "latency_max_us": round(self.metrics.latency_max_us, 1),
                "latency_jitter_us": round(self.metrics.latency_jitter_us, 1),
                "estop_latency_us": round(self.metrics.estop_latency_us, 1),
                "heartbeat_avg_latency_us": round(self.metrics.heartbeat_avg_latency_us, 1),
                "command_avg_latency_us": round(self.metrics.command_avg_latency_us, 1),
                "telemetry_avg_latency_us": round(self.metrics.telemetry_avg_latency_us, 1),
                "nodes_healthy": self.metrics.active_nodes_healthy,
                "nodes_warning": self.metrics.active_nodes_warning,
                "nodes_tripped": self.metrics.active_nodes_tripped,
            },
            "nodes": [
                {
                    "node_id": n.node_id,
                    "name": n.name,
                    "state": int(n.state),
                    "state_name": n.state.name,
                    "is_online": n.is_online,
                    "p_actual": round(n.p_actual, 3),
                    "v_actual": round(n.v_actual, 3),
                    "tau_actual": round(n.tau_actual, 2),
                    "iq_actual": round(n.iq_actual, 2),
                    "vbus": round(n.vbus, 1),
                    "temp_fet": round(n.temp_fet, 1),
                    "temp_motor": round(n.temp_motor, 1),
                    "time_since_hb_ms": round(n.time_since_heartbeat_ms, 1),
                    "tec": n.tec,
                    "rec": n.rec,
                    "crc_errors": n.crc_errors,
                    "watchdog_trips": n.watchdog_trips
                }
                for n in self.nodes.values()
            ],
            "recent_packets": [
                {
                    "can_id": hex(f.can_id),
                    "category": f.category.name,
                    "sender_id": f.sender_id,
                    "target_id": f.target_id,
                    "dlc": len(f.data),
                    "latency_us": round(f.end_to_end_latency_us, 1),
                    "duration_us": round(f.duration_us, 1),
                    "is_fd": f.is_fd,
                    "is_corrupted": f.is_corrupted,
                    "time_us": round(f.time_delivered_us, 1)
                }
                for f in list(self.recent_delivered_frames)[-15:]
            ],
            "recent_events": list(self.recent_events)[-10:]
        }
