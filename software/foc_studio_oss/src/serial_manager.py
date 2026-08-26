"""
Serial Communication and Hardware/Simulation Manager.
Utilizes PySerial (BSD-3-Clause) for hardware I/O and provides an integrated
dynamic FOC Simulation Engine for offline testing and academic demonstrations.
"""

import sys
import os
import glob
import time
import math
import struct
import subprocess
import threading
from typing import List, Dict, Any, Optional, Callable

try:
    import serial
    import serial.tools.list_ports
    HAS_PYSERIAL = True
except ImportError:
    HAS_PYSERIAL = False

from .models import PortInfo
from .telemetry_parser import TelemetryParser, PACKET_SIZES, MAGIC1, MAGIC2


class SerialManager:
    """Manages physical USB Serial and realistic mathematical FOC simulations."""

    def __init__(self, on_frame_callback: Optional[Callable[[Dict[str, Any]], None]] = None):
        self.on_frame_callback = on_frame_callback
        self.ser: Optional[serial.Serial] = None
        self.port: str = ""
        self.baudrate: int = 115200
        self.is_connected: bool = False
        self.is_simulation: bool = False
        self.running: bool = False
        self.thread: Optional[threading.Thread] = None
        self.lock = threading.RLock()

        # Telemetry metrics
        self.sample_count: int = 0
        self.last_hz_calc_time: float = time.time()
        self.hz_counter: int = 0
        self.telemetry_hz: float = 0.0
        self.error_count: int = 0
        self.diagnostic_logs: List[str] = []
        self.max_diagnostic_logs: int = 100

        # Simulation state variables
        self.sim_theta_e: float = 0.0
        self.sim_mech_angle: float = 0.0
        self.sim_speed_rpm: float = 0.0
        self.sim_target_rpm: float = 800.0
        self.sim_iq_target: float = 2.5
        self.sim_mode: int = 3  # Speed mode default in sim
        self.sim_vbus: float = 24.0
        self.sim_temp: float = 38.5

    def list_available_ports(self) -> List[PortInfo]:
        """Scan system for available serial ports and simulation option."""
        physical_ports: List[PortInfo] = []

        # Check physical ports via PySerial if available
        if HAS_PYSERIAL:
            try:
                for p in serial.tools.list_ports.comports():
                    # Ignore motherboard dummy UARTs on Linux. app_driver did this
                    # and it prevents users from selecting /dev/ttyS0..ttyS31 by mistake.
                    if p.device.startswith("/dev/ttyS"):
                        continue
                    physical_ports.append(PortInfo(
                        device=p.device,
                        description=f"{p.description} ({p.device})",
                        hwid=p.hwid
                    ))
            except Exception:
                pass

        # Also fallback glob scan for Linux
        found_devices = {p.device for p in physical_ports}
        for dev in sorted(glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*")):
            if dev not in found_devices:
                physical_ports.append(PortInfo(
                    device=dev,
                    description=f"Serial Device ({dev})",
                    hwid=dev
                ))

        usb_ports = [
            p for p in physical_ports
            if "USB" in p.description.upper() or "ACM" in p.device.upper() or "USB" in p.device.upper()
        ]
        other_ports = [p for p in physical_ports if p not in usb_ports]

        ports_list = usb_ports + other_ports
        ports_list.append(PortInfo(
            device="SIMULATION",
            description="Virtual FOC Motor Simulation (Demo Mode)",
            hwid="SIM-VIRTUAL-FOC"
        ))
        return ports_list

    def log_diagnostic(self, message: str) -> None:
        timestamp = time.strftime("%H:%M:%S")
        entry = f"[{timestamp}] {message}"
        with self.lock:
            self.diagnostic_logs.append(entry)
            if len(self.diagnostic_logs) > self.max_diagnostic_logs:
                self.diagnostic_logs.pop(0)
        print(entry)

    def connect(self, port: str, baudrate: int = 115200) -> bool:
        """Connect to real hardware serial port or start simulation engine."""
        self.disconnect()

        with self.lock:
            self.port = port
            self.baudrate = baudrate

            if port.upper() == "SIMULATION":
                self.is_simulation = True
                self.is_connected = True
                self.running = True
                self.thread = threading.Thread(target=self._simulation_worker, daemon=True)
                self.thread.start()
                self.log_diagnostic("Simulation mode started.")
                return True

            if not HAS_PYSERIAL:
                raise RuntimeError("PySerial is not installed in the Python environment.")

            try:
                self.ser = serial.Serial(
                    port=port,
                    baudrate=baudrate,
                    timeout=0.1,
                    write_timeout=0.2,
                    exclusive=True
                )
                self.is_simulation = False
                self.is_connected = True
                self.running = True
                self.thread = threading.Thread(target=self._hardware_read_worker, daemon=True)
                self.thread.start()
                self.log_diagnostic(f"Connected to {port} @ {baudrate} baud.")
                return True
            except Exception as e:
                self.is_connected = False
                self.ser = None
                self.log_diagnostic(f"Connection failed to {port}: {e}")
                raise ConnectionError(f"Failed to open port {port}: {e}")

    def disconnect(self) -> None:
        """Disconnect and stop reading thread."""
        self.running = False
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=0.5)

        with self.lock:
            if self.ser:
                try:
                    self.ser.close()
                except Exception:
                    pass
                self.ser = None
            self.is_connected = False
            self.is_simulation = False
            self.telemetry_hz = 0.0
        self.log_diagnostic("Disconnected.")

    def send_command(self, cmd_bytes: bytes) -> bool:
        """Send raw binary command to hardware STM32."""
        if not self.is_connected:
            return False
        if self.is_simulation:
            # Parse simulated command
            return True
        with self.lock:
            if self.ser and self.ser.is_open:
                try:
                    self.ser.write(cmd_bytes)
                    self.ser.flush()
                    return True
                except Exception:
                    return False
        return False

    def send_ascii_command(self, text: str) -> bool:
        """Send ASCII text command like 'CALIB\n' or 'ALIGN\n' to STM32."""
        text = text.strip()
        if self.is_simulation:
            return self._apply_sim_ascii_command(text)
        if not text.endswith("\r\n"):
            text += "\r\n"
        return self.send_command(text.encode("utf-8"))

    def _apply_sim_ascii_command(self, text: str) -> bool:
        """Apply common app_driver ASCII commands to the simulation engine."""
        parts = text.strip().split()
        if not parts:
            return False
        cmd = parts[0].upper()
        try:
            if cmd in ("STOP", "IDLE"):
                self.sim_mode = 0
                self.sim_target_rpm = 0.0
                self.sim_iq_target = 0.0
            elif cmd == "MODE" and len(parts) >= 2:
                self.sim_mode = int(float(parts[1]))
            elif cmd == "SPEED" and len(parts) >= 2:
                self.sim_mode = 3
                self.sim_target_rpm = float(parts[1])
            elif cmd == "IQ" and len(parts) >= 2:
                self.sim_mode = 1
                self.sim_iq_target = float(parts[1])
            elif cmd == "OPENLOOP" and len(parts) >= 2:
                self.sim_mode = 7
                self.sim_target_rpm = float(parts[1])
                if len(parts) >= 3:
                    self.sim_vbus = max(0.0, min(60.0, float(parts[2]) * 2.4))
            elif cmd == "POS" and len(parts) >= 2:
                self.sim_mode = 4
            self.log_diagnostic(f"Simulation command: {text}")
            return True
        except Exception as e:
            self.log_diagnostic(f"Simulation command failed [{text}]: {e}")
            return False

    def send_motor_control(self, mode: int, target_val: float) -> bool:
        """Send motor control in the same ASCII command style as app_driver."""
        if self.is_simulation:
            self.sim_mode = mode
            if mode == 3:
                self.sim_target_rpm = target_val
            elif mode == 1:
                self.sim_iq_target = target_val
            elif mode in (0, 2):
                self.sim_target_rpm = 0.0
                self.sim_iq_target = 0.0
            return True

        if mode == 0:
            return self.send_ascii_command("STOP")
        if mode == 1:
            return self.send_ascii_command(f"IQ {float(target_val):.3f}")
        if mode == 3:
            return self.send_ascii_command(f"SPEED {float(target_val):.1f}")
        if mode == 4:
            return self.send_ascii_command(f"POS {float(target_val):.3f}")
        return self.send_ascii_command(f"MODE {int(mode)}")

    def _hardware_read_worker(self) -> None:
        """Background reader for hardware serial port with sliding window packet search."""
        buffer = bytearray()
        while self.running and self.ser and self.ser.is_open:
            try:
                waiting = self.ser.in_waiting
                if waiting > 0:
                    raw = self.ser.read(min(waiting, 1024))
                    if raw:
                        buffer.extend(raw)

                # Search for packet magic 0xAA 0x55
                while len(buffer) >= 4:
                    if buffer[0] != MAGIC1 or buffer[1] != MAGIC2:
                        del buffer[0]
                        continue

                    packet_size = int(buffer[3]) + 4
                    if packet_size not in PACKET_SIZES:
                        self.error_count += 1
                        del buffer[0]
                        continue
                    if len(buffer) < packet_size:
                        break

                    candidate = bytes(buffer[:packet_size])
                    parsed = TelemetryParser.parse_packet(candidate)
                    if parsed:
                        del buffer[:packet_size]
                        self._handle_new_frame(parsed)
                        continue
                    self.error_count += 1
                    del buffer[0]

                if len(buffer) > 4096:
                    buffer.clear()

                time.sleep(0.002)
            except Exception as e:
                self.log_diagnostic(f"Serial read error: {e}")
                with self.lock:
                    self.running = False
                    self.is_connected = False
                    self.telemetry_hz = 0.0
                    if self.ser:
                        try:
                            self.ser.close()
                        except Exception:
                            pass
                        self.ser = None
                break

    def _simulation_worker(self) -> None:
        """Generates realistic FOC 3-phase sinusoidal waveforms at 100Hz."""
        interval = 0.010  # 100Hz (10ms)
        t_start = time.time()

        while self.running and self.is_simulation:
            t_now = time.time() - t_start
            timestamp_ms = int(t_now * 1000)

            # Smoothly ramp speed towards target
            rpm_diff = self.sim_target_rpm - self.sim_speed_rpm
            self.sim_speed_rpm += rpm_diff * 0.05

            # Electrical angle frequency omega_e (rad/s)
            pole_pairs = 7
            mech_rad_s = (self.sim_speed_rpm * 2 * math.pi) / 60.0
            elec_rad_s = mech_rad_s * pole_pairs

            self.sim_theta_e += elec_rad_s * interval
            self.sim_theta_e = (self.sim_theta_e + math.pi) % (2 * math.pi) - math.pi
            self.sim_mech_angle = (self.sim_mech_angle + mech_rad_s * interval) % (2 * math.pi)

            # Current amplitude proportional to load/acceleration
            i_peak = max(0.2, abs(self.sim_iq_target) * 0.95 + 0.1 * math.sin(t_now * 5.0))
            if self.sim_mode == 0:  # IDLE
                i_peak = 0.0
                self.sim_speed_rpm *= 0.98

            # 3-Phase Sinusoidal currents (120 deg apart)
            i_a = i_peak * math.sin(self.sim_theta_e)
            i_b = i_peak * math.sin(self.sim_theta_e - 2.0 * math.pi / 3.0)
            i_c = i_peak * math.sin(self.sim_theta_e + 2.0 * math.pi / 3.0)

            # Id / Iq
            i_d = 0.04 * math.sin(t_now * 10.0)  # Near 0
            i_q = i_peak * 0.98

            # Duty cycles (PWM centered around 50%)
            duty_a = 0.5 + 0.4 * (i_a / (i_peak + 0.01))
            duty_b = 0.5 + 0.4 * (i_b / (i_peak + 0.01))
            duty_c = 0.5 + 0.4 * (i_c / (i_peak + 0.01))

            sim_frame = {
                "timestamp_ms": timestamp_ms,
                "i_a": round(i_a, 4),
                "i_b": round(i_b, 4),
                "i_c": round(i_c, 4),
                "i_sum": round(i_a + i_b + i_c, 4),
                "i_peak": round(i_peak, 4),
                "i_d": round(i_d, 4),
                "i_q": round(i_q, 4),
                "i_q_target": round(self.sim_iq_target, 4),
                "i_vector_mag": round(math.sqrt(i_d * i_d + i_q * i_q), 4),
                "duty_a": round(duty_a, 4),
                "duty_b": round(duty_b, 4),
                "duty_c": round(duty_c, 4),
                "phase_elec": round(self.sim_theta_e, 4),
                "mech_angle": round(self.sim_mech_angle, 4),
                "joint_angle": round(self.sim_mech_angle / 10.0, 4),
                "speed_rpm": round(self.sim_speed_rpm, 2),
                "speed_target_rpm": round(self.sim_target_rpm, 2),
                "v_bus": round(self.sim_vbus + 0.1 * math.sin(t_now), 2),
                "temp_fet": round(self.sim_temp + 0.02 * (self.sim_speed_rpm / 500.0), 1),
                "power_watts": round(self.sim_vbus * max(0.1, i_peak * 0.8), 2),
                "control_mode": self.sim_mode,
                "motor_state": 1 if self.sim_mode > 0 else 0,
                "fault_code": 0
            }

            self._handle_new_frame(sim_frame)
            time.sleep(interval)

    def _handle_new_frame(self, frame: Dict[str, Any]) -> None:
        """Update rate statistics and forward to consumer."""
        self.sample_count += 1
        self.hz_counter += 1

        now = time.time()
        if now - self.last_hz_calc_time >= 1.0:
            self.telemetry_hz = round(self.hz_counter / (now - self.last_hz_calc_time), 1)
            self.hz_counter = 0
            self.last_hz_calc_time = now

        if self.on_frame_callback:
            self.on_frame_callback(frame)

    def run_auto_tune(self) -> Dict[str, Any]:
        """Run the existing OSS auto-identification script without keeping serial open."""
        was_connected = self.is_connected
        saved_port = self.port
        saved_baud = self.baudrate

        if was_connected:
            self.disconnect()
            time.sleep(0.5)

        root_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        script_path = os.path.join(root_dir, "auto_identify_motor.py")
        profile_path = os.path.join(root_dir, "motor_calib_profile.json")
        if os.path.exists(profile_path):
            try:
                os.remove(profile_path)
            except Exception:
                pass

        profile: Dict[str, Any] = {}
        try:
            cmd_args = [sys.executable, script_path]
            if saved_port and saved_port.upper() != "SIMULATION":
                cmd_args.append(saved_port)
            result = subprocess.run(cmd_args, capture_output=True, text=True, timeout=90)
            output = result.stdout or result.stderr or ""
            if os.path.exists(profile_path):
                import json
                with open(profile_path, "r", encoding="utf-8") as f:
                    profile = json.load(f)
            success = result.returncode == 0 and bool(profile)
            return {"success": success, "message": output, "profile": profile}
        except Exception as e:
            return {"success": False, "message": str(e), "profile": profile}
        finally:
            if was_connected and saved_port:
                time.sleep(0.3)
                try:
                    self.connect(saved_port, saved_baud)
                except Exception:
                    pass
