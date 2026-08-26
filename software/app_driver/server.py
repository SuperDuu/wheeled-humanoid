#!/usr/bin/env python3
"""
Real-time FOC Telemetry & Oscilloscope Server for STM32 Joint Driver
Streams 3-phase currents, Id/Iq vectors, electrical angles, and speeds to a local Web visualizer.
Provides throttled diagnostic logging, MATLAB-ready CSV recording, and USB port control.
"""

import sys
import os
import time
import json
import struct
import glob
import threading
from socketserver import ThreadingMixIn
from http.server import HTTPServer, SimpleHTTPRequestHandler
from urllib.parse import urlparse, parse_qs

class ThreadedHTTPServer(ThreadingMixIn, HTTPServer):
    daemon_threads = True
    allow_reuse_address = True

# Optional dependency: pyserial
try:
    import serial
    import serial.tools.list_ports
    HAS_PYSERIAL = True
except ImportError:
    HAS_PYSERIAL = False

# Telemetry Binary Packet Format:
# typedef struct {
#     uint8_t  magic1;             // 0xAA (offset 0)
#     uint8_t  magic2;             // 0x55 (offset 1)
#     uint8_t  packet_type;        // 0x01 (offset 2)
#     uint8_t  payload_len;        // len (offset 3)
#     uint32_t timestamp_ms;       // (offset 4)
#     float    i_a, i_b, i_c;      // (offset 8, 12, 16)
#     float    i_d, i_q, i_q_target;// (offset 20, 24, 28)
#     float    duty_a, duty_b, duty_c;// (offset 32, 36, 40)
#     float    phase_elec, mech_angle, joint_angle; // (offset 44, 48, 52)
#     float    speed_rpm, speed_target_rpm; // (offset 56, 60)
#     float    v_bus, temp_fet;    // (offset 64, 68)
#     uint8_t  control_mode, motor_state, fault_code; // (offset 72, 73, 74)
#     int8_t   encoder_dir;        // (offset 75)
#     float    vd, vq, zero_elec_angle, id_target; // (offset 76, 80, 84, 88)
#     uint16_t checksum;           // (offset 92)
# } telemetry_packet_t; Total = 94 bytes.
PACKET_FORMAT = "<BBBB I 16f BBBb 4f H"
PACKET_SIZE = struct.calcsize(PACKET_FORMAT)

class TelemetryManager:
    def __init__(self):
        self.ser = None
        self.port = ""
        self.baudrate = 115200
        self.is_connected = False
        self.running = False
        self.thread = None
        self.lock = threading.Lock()
        self.serial_write_lock = threading.Lock()
        self.device_ready = threading.Event()
        
        # Live state
        self.latest_telemetry = None
        self.telemetry_sequence = 0
        self.packet_count = 0
        self.error_count = 0
        self.fps = 0.0
        self.last_fps_time = time.time()
        self.last_fps_packets = 0
        
        # Throttled console logging
        self.last_console_log_time = 0.0
        self.console_log_interval = 0.5  # Log every 500ms max
        self.diagnostic_logs = []
        self.max_diag_logs = 100
        
        # CSV Recording Buffer
        self.is_recording = False
        self.record_buffer = []
        self.record_start_time = 0
        
        # Background auto-reconnector
        self.auto_reconnect_enabled = True
        self.reconnect_thread = threading.Thread(target=self._auto_reconnect_loop, daemon=True)
        self.reconnect_thread.start()

    def _auto_reconnect_loop(self):
        while True:
            time.sleep(1.0)
            if self.auto_reconnect_enabled and not self.is_connected:
                ports = self.get_available_ports()
                if ports:
                    target_port = ports[0]['port']
                    try:
                        self.connect(target_port, self.baudrate or 115200)
                    except Exception:
                        pass

    def get_available_ports(self):
        ports = []
        usb_ports = []
        other_ports = []
        if HAS_PYSERIAL:
            for p in serial.tools.list_ports.comports():
                info = {
                    "port": p.device,
                    "description": p.description or "Serial Device",
                    "hwid": p.hwid or ""
                }
                # Filter out Linux dummy motherboard serial ports (ttyS0..ttyS31)
                if p.device.startswith("/dev/ttyS"):
                    continue
                if "USB" in p.description.upper() or "ACM" in p.device or "USB" in p.device:
                    usb_ports.append(info)
                else:
                    other_ports.append(info)
            ports = usb_ports + other_ports
        else:
            # Fallback for Linux /dev
            dev_list = glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*")
            for d in dev_list:
                ports.append({"port": d, "description": "USB CDC Serial Device", "hwid": ""})
        return ports

    def connect(self, port, baudrate=115200):
        self.disconnect()
        if not HAS_PYSERIAL:
            self.log_diagnostic(f"Error: pyserial not installed. Run 'pip install pyserial'")
            return False, "pyserial not installed"

        try:
            self.ser = serial.Serial(
                port,
                baudrate=int(baudrate),
                timeout=0.05,
                exclusive=True,
            )
            self.port = port
            self.baudrate = int(baudrate)
            self.is_connected = True
            self.running = True
            self.device_ready.clear()
            self.thread = threading.Thread(target=self._reader_loop, daemon=True)
            self.thread.start()
            self.log_diagnostic(f"Connected to {port} @ {baudrate} baud")
            return True, "Connected successfully"
        except Exception as e:
            self.is_connected = False
            self.log_diagnostic(f"Connection failed to {port}: {str(e)}")
            return False, str(e)

    def disconnect(self):
        self.running = False
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=0.5)
        if self.ser and self.ser.is_open:
            try:
                self.ser.close()
            except Exception:
                pass
        self.ser = None
        self.is_connected = False
        self.device_ready.clear()
        self.log_diagnostic("Disconnected from serial port")

    def _command_acknowledged(self, command_name, cmd_str):
        with self.lock:
            telemetry = dict(self.latest_telemetry) if self.latest_telemetry else {}
        if not telemetry:
            return False
        if command_name in {"STOP", "OFF"}:
            return telemetry.get("motor_state") == 0
        if command_name in {"ALIGN", "CALIB", "CALIBRATE"}:
            return telemetry.get("motor_state") == 1
        if command_name == "SPEED":
            try:
                target = float(cmd_str.strip().split(maxsplit=1)[1])
            except (IndexError, ValueError):
                return False
            return abs(telemetry.get("speed_target_rpm", 0.0) - target) < 0.25
        return False

    def send_command(self, cmd_str):
        if not self.is_connected or not self.ser:
            return False, "Not connected"
        # Opening ttyACM can complete before the STM32 reaches its main loop.
        # The first valid telemetry frame proves both USB endpoints and command
        # processing are ready, avoiding a lost first command after reset.
        if not self.device_ready.wait(timeout=2.0):
            return False, "Device is connected but telemetry is not ready"
        try:
            cmd = cmd_str.strip() + '\r\n'
            command_name = cmd_str.strip().split(maxsplit=1)[0].upper() if cmd_str.strip() else ""
            retry_safe = command_name not in {
                "REL", "STEP", "MOVE", "RESET", "REBOOT", "DIRTEST"
            }
            payload = cmd.encode('utf-8')
            with self.serial_write_lock:
                self.ser.write(payload)
                self.ser.flush()
                if retry_safe:
                    ack_capable = command_name in {
                        "STOP", "OFF", "ALIGN", "CALIB", "CALIBRATE", "SPEED"
                    }
                    if ack_capable:
                        deadline = time.monotonic() + 0.35
                        while (time.monotonic() < deadline and
                               not self._command_acknowledged(command_name, cmd_str)):
                            time.sleep(0.01)
                    else:
                        time.sleep(0.15)
                    if not ack_capable or not self._command_acknowledged(command_name, cmd_str):
                        self.ser.write(payload)
                        self.ser.flush()
            self.log_diagnostic(f"Sent Command: {cmd_str.strip()}")
            return True, "Command sent"
        except Exception as e:
            return False, str(e)

    def log_diagnostic(self, message):
        timestamp = time.strftime("%H:%M:%S")
        entry = f"[{timestamp}] {message}"
        with self.lock:
            self.diagnostic_logs.append(entry)
            if len(self.diagnostic_logs) > self.max_diag_logs:
                self.diagnostic_logs.pop(0)
        print(entry)

    def start_recording(self):
        with self.lock:
            self.is_recording = True
            self.record_buffer = []
            self.record_start_time = time.time()
        self.log_diagnostic("Started CSV Telemetry Recording...")
        return True

    def stop_recording(self):
        with self.lock:
            self.is_recording = False
            count = len(self.record_buffer)
        self.log_diagnostic(f"Stopped Recording. Captured {count} samples.")
        return count

    def get_csv_data(self):
        with self.lock:
            headers = [
                "timestamp_ms", "i_a", "i_b", "i_c", "i_d", "i_q", "i_q_target",
                "duty_a", "duty_b", "duty_c", "phase_elec", "mech_angle", "joint_angle",
                "speed_rpm", "speed_target_rpm", "v_bus", "temp_fet", "control_mode",
                "motor_state", "fault_code"
            ]
            lines = [",".join(headers)]
            for pkt in self.record_buffer:
                row = [
                    str(pkt.get("timestamp_ms", 0)),
                    f"{pkt.get('i_a', 0):.4f}", f"{pkt.get('i_b', 0):.4f}", f"{pkt.get('i_c', 0):.4f}",
                    f"{pkt.get('i_d', 0):.4f}", f"{pkt.get('i_q', 0):.4f}", f"{pkt.get('i_q_target', 0):.4f}",
                    f"{pkt.get('duty_a', 0):.4f}", f"{pkt.get('duty_b', 0):.4f}", f"{pkt.get('duty_c', 0):.4f}",
                    f"{pkt.get('phase_elec', 0):.4f}", f"{pkt.get('mech_angle', 0):.4f}", f"{pkt.get('joint_angle', 0):.4f}",
                    f"{pkt.get('speed_rpm', 0):.2f}", f"{pkt.get('speed_target_rpm', 0):.2f}",
                    f"{pkt.get('v_bus', 0):.2f}", f"{pkt.get('temp_fet', 0):.1f}",
                    str(pkt.get("control_mode", 0)), str(pkt.get("motor_state", 0)), str(pkt.get("fault_code", 0))
                ]
                lines.append(",".join(row))
            return "\n".join(lines)

    def _reader_loop(self):
        buffer = bytearray()
        while self.running and self.ser and self.ser.is_open:
            try:
                data = self.ser.read(self.ser.in_waiting or 1)
                if not data:
                    time.sleep(0.001)
                    continue

                buffer.extend(data)

                # Search for 4-byte packet header 0xAA 0x55 0x01 0x4A (74)
                while len(buffer) >= PACKET_SIZE:
                    if buffer[0] == 0xAA and buffer[1] == 0x55 and buffer[2] == 0x01 and buffer[3] == (PACKET_SIZE - 4):
                        packet_bytes = buffer[:PACKET_SIZE]
                        
                        try:
                            unpacked = struct.unpack(PACKET_FORMAT, packet_bytes)
                            (magic1, magic2, pkt_type, payload_len, ts_ms,
                             ia, ib, ic, id_c, iq_c, iq_tgt,
                             da, db, dc, phase, mech, joint,
                             speed, speed_tgt, vbus, temp,
                             mode, state, fault, enc_dir,
                             vd, vq, zero_elec, id_tgt,
                             chk_val) = unpacked

                            # Verify 16-bit checksum (firmware: sum of bytes[4:-2])
                            computed_chk = sum(packet_bytes[4:-2]) & 0xFFFF
                            if computed_chk != chk_val:
                                self.error_count += 1
                                buffer.pop(0)
                                continue

                            self.device_ready.set()

                            pkt_dict = {
                                "timestamp_ms": ts_ms,
                                "i_a": round(ia, 4),
                                "i_b": round(ib, 4),
                                "i_c": round(ic, 4),
                                "i_d": round(id_c, 4),
                                "i_q": round(iq_c, 4),
                                "i_q_target": round(iq_tgt, 4),
                                "duty_a": round(da, 4),
                                "duty_b": round(db, 4),
                                "duty_c": round(dc, 4),
                                "phase_elec": round(phase, 4),
                                "mech_angle": round(mech, 4),
                                "joint_angle": round(joint, 4),
                                "speed_rpm": round(speed, 2),
                                "speed_target_rpm": round(speed_tgt, 2),
                                "v_bus": round(vbus, 2),
                                "temp_fet": round(temp, 1),
                                "control_mode": mode,
                                "motor_state": state,
                                "fault_code": fault,
                                "encoder_dir": enc_dir,
                                "vd": round(vd, 3),
                                "vq": round(vq, 3),
                                "zero_elec_angle": round(zero_elec, 4),
                                "id_target": round(id_tgt, 4)
                            }

                            with self.lock:
                                self.latest_telemetry = pkt_dict
                                self.telemetry_sequence += 1
                                self.packet_count += 1
                                if self.is_recording:
                                    self.record_buffer.append(pkt_dict)

                            # Throttled console diagnostic log (1 summary line per 500ms)
                            now = time.time()
                            if now - self.last_console_log_time >= self.console_log_interval:
                                self.last_console_log_time = now
                                # mode = control_mode enum: 0=DUTY,1=POWER,2=CURRENT,3=BRAKE,4=SPEED,5=POS,6=HANDBRAKE,7=OPENLOOP
                                mode_names = ["DUTY", "POWER", "CURRENT", "BRAKE", "SPEED", "POS", "HANDBRAKE", "OPENLOOP"]
                                m_str = mode_names[mode] if mode < len(mode_names) else f"MODE_{mode}"
                                # state = mc_state enum: 0=OFF,1=DETECTING,2=RUNNING,3=FULL_BRAKE
                                state_names = ["OFF", "DETECTING", "RUNNING", "BRAKE"]
                                s_str = state_names[state] if state < len(state_names) else f"STATE_{state}"
                                log_line = f"Telemetry @ {self.fps:.0f}Hz | mode={m_str} state={s_str} | Id={id_c:+.2f}A, Iq={iq_c:+.2f}A (Tgt={iq_tgt:+.2f}A) | RPM={speed:+.1f}/{speed_tgt:+.1f} | Vbus={vbus:.1f}V | Vd={vd:+.1f}V Vq={vq:+.1f}V θe={phase:.2f} θ0={zero_elec:.2f} dir={enc_dir} | duty={da:.3f}/{db:.3f}/{dc:.3f}"
                                if fault > 0:
                                    log_line += f" | FAULT={fault}"
                                self.log_diagnostic(log_line)

                        except Exception as parse_err:
                            self.error_count += 1

                        buffer = buffer[PACKET_SIZE:]
                    else:
                        # Scan next byte
                        buffer.pop(0)

                # Calculate Telemetry Rate (Hz)
                now = time.time()
                if now - self.last_fps_time >= 1.0:
                    elapsed = now - self.last_fps_time
                    self.fps = (self.packet_count - self.last_fps_packets) / elapsed
                    # Log checksum error rate for debugging
                    if hasattr(self, '_last_error_count'):
                        errors_per_sec = (self.error_count - self._last_error_count) / elapsed
                        if errors_per_sec > 0:
                            self.log_diagnostic(f"⚠️ Checksum errors: {errors_per_sec:.0f}/s (total={self.error_count})")
                    self._last_error_count = self.error_count
            except Exception as e:
                if self.running:
                    self.log_diagnostic(f"Serial read error: {str(e)}")
                    self.is_connected = False
                    try:
                        if self.ser:
                            self.ser.close()
                    except Exception:
                        pass
                    break
                    time.sleep(0.01)

telemetry_mgr = TelemetryManager()

class TelemetryHTTPHandler(SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        # Serve static files from current directory
        directory = os.path.dirname(os.path.abspath(__file__))
        super().__init__(*args, directory=directory, **kwargs)

    def end_headers(self):
        self.send_header('Cache-Control', 'no-store, no-cache, must-revalidate, max-age=0')
        self.send_header('Pragma', 'no-cache')
        self.send_header('Expires', '0')
        super().end_headers()

    def do_GET(self):
        parsed = urlparse(self.path)
        path = parsed.path

        if path == "/api/ports":
            ports = telemetry_mgr.get_available_ports()
            self._send_json({"ports": ports, "connected": telemetry_mgr.is_connected, "current_port": telemetry_mgr.port})
        
        elif path == "/api/status":
            self._send_json({
                "connected": telemetry_mgr.is_connected,
                "port": telemetry_mgr.port,
                "baudrate": telemetry_mgr.baudrate,
                "packet_count": telemetry_mgr.packet_count,
                "fps": round(telemetry_mgr.fps, 1),
                "is_recording": telemetry_mgr.is_recording,
                "recorded_samples": len(telemetry_mgr.record_buffer),
                "logs": telemetry_mgr.diagnostic_logs[-20:],
                "latest": telemetry_mgr.latest_telemetry
            })

        elif path == "/api/stream":
            self.send_response(200)
            self.send_header('Content-Type', 'text/event-stream')
            self.send_header('Cache-Control', 'no-cache')
            self.send_header('Connection', 'keep-alive')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()

            # Each HTTP worker publishes the latest sample independently.
            # A slow browser can block only its own worker, never the serial reader.
            last_sequence = -1
            last_keepalive = time.monotonic()
            try:
                while True:
                    with telemetry_mgr.lock:
                        sequence = telemetry_mgr.telemetry_sequence
                        pkt = (dict(telemetry_mgr.latest_telemetry)
                               if telemetry_mgr.latest_telemetry else None)

                    if pkt is not None and sequence != last_sequence:
                        msg = f"data: {json.dumps(pkt, separators=(',', ':'))}\n\n"
                        self.wfile.write(msg.encode('utf-8'))
                        self.wfile.flush()
                        last_sequence = sequence
                        last_keepalive = time.monotonic()
                    elif time.monotonic() - last_keepalive >= 5.0:
                        self.wfile.write(b": keepalive\n\n")
                        self.wfile.flush()
                        last_keepalive = time.monotonic()

                    time.sleep(0.01)
            except (BrokenPipeError, ConnectionResetError, OSError):
                return

        elif path == "/api/record/start":
            telemetry_mgr.start_recording()
            self._send_json({"status": "recording_started"})

        elif path == "/api/record/stop":
            count = telemetry_mgr.stop_recording()
            self._send_json({"status": "recording_stopped", "samples": count})

        elif path == "/api/record/download":
            csv_content = telemetry_mgr.get_csv_data()
            self.send_response(200)
            self.send_header('Content-Type', 'text/csv')
            self.send_header('Content-Disposition', f'attachment; filename="foc_telemetry_{int(time.time())}.csv"')
            self.end_headers()
            self.wfile.write(csv_content.encode('utf-8'))

        else:
            super().do_GET()

    def do_POST(self):
        parsed = urlparse(self.path)
        path = parsed.path
        content_len = int(self.headers.get('Content-Length', 0))
        post_data = self.rfile.read(content_len).decode('utf-8') if content_len > 0 else "{}"
        
        try:
            body = json.loads(post_data)
        except Exception:
            body = {}

        if path == "/api/connect":
            port = body.get("port", "")
            baud = body.get("baudrate", 115200)
            success, msg = telemetry_mgr.connect(port, baud)
            self._send_json({"success": success, "message": msg})

        elif path == "/api/disconnect":
            telemetry_mgr.disconnect()
            self._send_json({"success": True, "message": "Disconnected"})

        elif path == "/api/command":
            cmd = body.get("command", "")
            success, msg = telemetry_mgr.send_command(cmd)
            self._send_json({"success": success, "message": msg})

        elif path == "/api/auto_tune":
            import subprocess
            telemetry_mgr.auto_reconnect_enabled = False
            was_connected = telemetry_mgr.is_connected
            saved_port = telemetry_mgr.port
            saved_baud = telemetry_mgr.baudrate
            if was_connected:
                telemetry_mgr.disconnect()
                time.sleep(0.5)

            script_path = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "foc_studio_oss", "auto_identify_motor.py"))
            profile_path = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "foc_studio_oss", "motor_calib_profile.json"))
            if os.path.exists(profile_path):
                try:
                    os.remove(profile_path)
                except Exception:
                    pass
            try:
                cmd_args = [sys.executable, script_path]
                if saved_port:
                    cmd_args.append(saved_port)
                res = subprocess.run(cmd_args, capture_output=True, text=True, timeout=90)
                output = res.stdout
                profile = {}
                if os.path.exists(profile_path):
                    with open(profile_path, "r") as f:
                        profile = json.load(f)
                success = (res.returncode == 0 and bool(profile))
                msg = output if success else (res.stderr or output or "Calibration failed")
            except Exception as e:
                success = False
                msg = str(e)
                profile = {}
            finally:
                if was_connected:
                    time.sleep(0.3)
                    telemetry_mgr.connect(saved_port, saved_baud)
                telemetry_mgr.auto_reconnect_enabled = True

            self._send_json({"success": success, "message": msg, "profile": profile})

        else:
            self.send_error(404, "Endpoint not found")

    def _send_json(self, data):
        content = json.dumps(data).encode('utf-8')
        self.send_response(200)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Content-Length', len(content))
        self.end_headers()
        self.wfile.write(content)

    def log_message(self, format, *args):
        # Suppress normal HTTP access logs to keep terminal clean
        return

def run_server(port=8080):
    server_address = ('0.0.0.0', port)
    httpd = ThreadedHTTPServer(server_address, TelemetryHTTPHandler)
    print("=" * 70)
    print(f"🚀 FOC Telemetry & Oscilloscope Visualizer is running at:")
    print(f"👉 Local Web App: http://localhost:{port}")
    print("=" * 70)
    
    # Auto-scan ports & auto-connect to first USB port
    ports = telemetry_mgr.get_available_ports()
    if ports:
        print(f"🔍 Discovered USB Serial Ports: {[p['port'] for p in ports]}")
        first_port = ports[0]['port']
        print(f"⚡ Auto-connecting to {first_port}...")
        telemetry_mgr.connect(first_port, 115200)
    else:
        print("⚠️ No USB serial ports detected. Connect your STM32 USB Type-C cable.")

    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("\nStopping server...")
        telemetry_mgr.disconnect()
        httpd.server_close()

if __name__ == '__main__':
    port_arg = 5050
    if len(sys.argv) > 1:
        try:
            port_arg = int(sys.argv[1])
        except ValueError:
            pass
    run_server(port_arg)
