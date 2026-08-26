"""
Pydantic Data Models and Schemas for FOC Telemetry Studio OSS.
Utilizes Pydantic (MIT License) for data validation, parsing, and OpenAPI schema generation.
"""

from typing import List, Optional
from pydantic import BaseModel, Field


class PortInfo(BaseModel):
    """Information about an available serial/USB communication port."""
    device: str = Field(..., description="Device port path (e.g., /dev/ttyUSB0)")
    description: str = Field(..., description="Human-readable port description")
    hwid: Optional[str] = Field(None, description="Hardware ID string")


class ConnectRequest(BaseModel):
    """Request payload to connect to a hardware serial port."""
    port: str = Field(..., description="Port path (e.g., /dev/ttyUSB0) or 'SIMULATION'")
    baudrate: int = Field(115200, description="Serial baudrate (e.g., 115200, 460800, 921600, 2000000)")


class MotorCommand(BaseModel):
    """Command payload for controlling motor modes, targets, and limits."""
    control_mode: int = Field(..., ge=0, le=4, description="Control mode: 0=IDLE, 1=CURRENT, 2=BRAKE, 3=SPEED, 4=POSITION")
    target_value: float = Field(0.0, description="Target value (Amperes for mode 1, RPM for mode 3, Radians for mode 4)")
    brake_current: Optional[float] = Field(None, description="Optional brake current for BRAKE mode.")


class TextCommand(BaseModel):
    """ASCII text command payload (CALIB, ALIGN, SPEED, VOLT, STOP)."""
    command: str = Field(..., description="ASCII text command string (e.g. CALIB, ALIGN, STOP, SPEED 150)")


class SystemStatus(BaseModel):
    """Overall system and connection health status."""
    connected: bool = Field(..., description="Whether serial connection is active")
    port: str = Field("", description="Active port name")
    baudrate: int = Field(115200, description="Active baudrate")
    is_simulation: bool = Field(False, description="True if running in mock demo mode")
    telemetry_hz: float = Field(0.0, description="Real-time telemetry packet rate in Hz")
    samples_received: int = Field(0, description="Total telemetry samples received")
    recorded_samples: int = Field(0, description="Current recorded samples count")
    is_recording: bool = Field(False, description="Whether data recorder is active")
    error_count: int = Field(0, description="Telemetry parse or serial error count")
    diagnostic_logs: List[str] = Field(default_factory=list, description="Recent serial diagnostic log lines")


class TelemetryFrame(BaseModel):
    """Processed high-level telemetry data frame sent over WebSocket."""
    timestamp_ms: int
    i_a: float
    i_b: float
    i_c: float
    i_sum: float
    i_peak: float
    i_d: float
    i_q: float
    i_q_target: float
    i_vector_mag: float
    duty_a: float
    duty_b: float
    duty_c: float
    phase_elec: float
    mech_angle: float
    joint_angle: float
    speed_rpm: float
    speed_target_rpm: float
    v_bus: float
    temp_fet: float
    power_watts: float
    control_mode: int
    motor_state: int
    fault_code: int
    encoder_lut_enabled: int = 0
    calibration_result: int = 0
