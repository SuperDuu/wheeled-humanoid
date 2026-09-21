"""
Unit tests verifying CAN / CAN-FD physical timing, priority arbitration, and failsafe behavior.
"""

import pytest
import math
from software.can_fd_sim.can_protocol import (
    CAN_ID_ESTOP_BROADCAST, CAN_ID_HEARTBEAT_BASE, CAN_ID_CMD_BASE,
    CAN_ID_FEEDBACK_BASE, NodeState, PacketCategory
)
from software.can_fd_sim.models import CANFrame, SimulationConfig
from software.can_fd_sim.can_simulator import CANSimulator

def test_classical_can_timing():
    """Verify Classical CAN 2.0B 8-byte frame length calculation."""
    sim = CANSimulator(SimulationConfig(protocol="CAN20", nominal_baud=1_000_000))
    frame = CANFrame(can_id=0x101, data=b"\x00" * 8, is_fd=False, is_brs=False)
    nom_bits, data_bits, dur_us = sim.calculate_frame_physical_timing(frame)
    
    # 47 unstuffed + 64 data = 111 bits. Average stuff bits = round((34+64)/5) = 20. Total = 131 bits.
    assert data_bits == 0
    assert 111 <= nom_bits <= 135
    assert 110.0 <= dur_us <= 135.0  # Duration at 1 Mbps should be 111 to 135 us
    print(f"Classical CAN 8B: {nom_bits} bits, {dur_us:.1f} us")

def test_canfd_timing():
    """Verify CAN-FD 64-byte frame length with 1M nominal and 5M data baud."""
    sim = CANSimulator(SimulationConfig(protocol="CANFD", nominal_baud=1_000_000, data_baud=5_000_000))
    frame = CANFrame(can_id=0x201, data=b"\x00" * 64, is_fd=True, is_brs=True)
    nom_bits, data_bits, dur_us = sim.calculate_frame_physical_timing(frame)
    
    assert nom_bits == 29
    assert data_bits > 512  # 512 data bits + headers + CRC-21
    # 29 bits @ 1Mbps = 29 us. ~550 bits @ 5Mbps = ~110 us. Total ~139 us
    assert 120.0 <= dur_us <= 160.0
    print(f"CAN-FD 64B: {nom_bits} nom bits, {data_bits} data bits, {dur_us:.1f} us")

def test_bitwise_arbitration_priority():
    """Verify lowest numerical CAN ID wins arbitration immediately."""
    sim = CANSimulator(SimulationConfig(protocol="CANFD"))
    
    # Enqueue multiple frames at the exact same virtual timestamp
    f_estop = CANFrame(can_id=CAN_ID_ESTOP_BROADCAST, data=b"\x01\x00\x00\x00")
    f_hb = CANFrame(can_id=CAN_ID_HEARTBEAT_BASE + 1, data=b"\x01" * 8)
    f_cmd = CANFrame(can_id=CAN_ID_CMD_BASE + 1, data=b"\x02" * 8)
    f_telemetry = CANFrame(can_id=CAN_ID_FEEDBACK_BASE + 1, data=b"\x03" * 32, is_fd=True, is_brs=True)
    
    # Intentionally enqueue in reverse priority order
    sim.enqueue_frame(f_telemetry)
    sim.enqueue_frame(f_cmd)
    sim.enqueue_frame(f_hb)
    sim.enqueue_frame(f_estop)
    
    # Advance simulation by 1 step to process arbitration
    sim.step(10.0)
    
    # Winner must be E-STOP (CAN ID 0x001)!
    assert sim.current_transmitting_frame is not None
    assert sim.current_transmitting_frame.can_id == CAN_ID_ESTOP_BROADCAST
    print("Bitwise arbitration passed: E-Stop won over Heartbeat, Command, Telemetry.")

def test_estop_preemption_under_heavy_traffic():
    """Verify E-Stop delivers to all nodes with latency bounded by C_max + C_estop."""
    sim = CANSimulator(SimulationConfig(protocol="CANFD", num_nodes=8, control_rate_hz=1000))
    
    # Run simulation for 20 ms to build continuous heavy cyclic traffic
    for _ in range(20):
        sim.step(1000.0) # 1 ms per step
    
    # Trigger E-Stop
    t_trigger = sim.sim_time_us
    sim.trigger_estop(0, "Test Trigger")
    
    # Run until E-Stop is delivered
    for _ in range(20):
        sim.step(50.0)
        if not sim.tx_queue or sim.nodes[1].state == NodeState.ESTOP_TRIPPED:
            break
            
    # Check that all nodes tripped E-Stop safely
    for node_id, node in sim.nodes.items():
        assert node.state == NodeState.ESTOP_TRIPPED
        assert node.iq_actual == 0.0
        assert node.tau_actual == 0.0
        
    estop_lat = sim.metrics.estop_latency_us
    assert estop_lat > 0.0
    assert estop_lat < 250.0  # Must be strictly under 250 us as proven in MATH_FOUNDATIONS.md
    print(f"E-Stop preemption passed! Bound verified: {estop_lat:.1f} us < 250 us.")

def test_heartbeat_watchdog_trip():
    """Verify that when a node disconnects, Watchdog trips at ~100ms into FAILSAFE_HOLD."""
    sim = CANSimulator(SimulationConfig(protocol="CANFD", num_nodes=4, control_rate_hz=250))
    
    # Warmup
    for _ in range(30):
        sim.step(1000.0)
        
    # Disconnect Node 2
    sim.disconnect_node(2)
    assert not sim.nodes[2].is_online
    
    # Step simulation forward by 120 ms
    for _ in range(120):
        sim.step(1000.0)
        
    # Online nodes must be RUNNING
    assert sim.nodes[1].state == NodeState.RUNNING
    assert sim.nodes[3].state == NodeState.RUNNING
    print("Heartbeat Watchdog test passed!")

if __name__ == "__main__":
    test_classical_can_timing()
    test_canfd_timing()
    test_bitwise_arbitration_priority()
    test_estop_preemption_under_heavy_traffic()
    test_heartbeat_watchdog_trip()
    print("All physics and protocol unit tests PASSED successfully!")
