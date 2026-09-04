#!/usr/bin/env python3
"""
Full End-to-End FOC Closed-Loop Verification Suite with True Standstill Waiting
"""
import sys, os, time, math
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'src'))
from telemetry_parser import TelemetryParser, PACKET_SIZE_94, MAGIC1, MAGIC2
import serial
import numpy as np

PORT = '/dev/ttyACM0'
BAUD = 115200
REPORT_PATH = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', 'foc_verification_report.txt'))

class Logger:
    def __init__(self, filepath):
        self.f = open(filepath, 'w')
    def log(self, msg=""):
        print(msg)
        self.f.write(msg + "\n")
    def close(self):
        self.f.close()

def read_packets(ser, duration_sec):
    buf = bytearray()
    samples = []
    t_end = time.time() + duration_sec
    while time.time() < t_end:
        raw = ser.read(ser.in_waiting or 1)
        if raw:
            buf.extend(raw)
        while len(buf) >= PACKET_SIZE_94:
            if buf[0] == MAGIC1 and buf[1] == MAGIC2:
                p = TelemetryParser.parse_packet(bytes(buf[:PACKET_SIZE_94]))
                if p:
                    samples.append(p)
                    del buf[:PACKET_SIZE_94]
                    continue
            del buf[0]
        time.sleep(0.002)
    return samples

def wait_for_standstill(ser, max_wait_sec=3.0):
    # Active electric brake to 0 RPM
    send_cmd(ser, "SPEED 0")
    t_end = time.time() + max_wait_sec
    buf = bytearray()
    while time.time() < t_end:
        raw = ser.read(ser.in_waiting or 1)
        if raw:
            buf.extend(raw)
        while len(buf) >= PACKET_SIZE_94:
            if buf[0] == MAGIC1 and buf[1] == MAGIC2:
                p = TelemetryParser.parse_packet(bytes(buf[:PACKET_SIZE_94]))
                if p and abs(p.get('speed_rpm', 10.0)) < 3.0:
                    time.sleep(0.2)
                    send_cmd(ser, "STOP")
                    return True
                del buf[:PACKET_SIZE_94]
                continue
            del buf[0]
        time.sleep(0.01)
    send_cmd(ser, "STOP")
    return False

def send_cmd(ser, cmd):
    ser.write(f"{cmd}\r\n".encode())
    time.sleep(0.05)

L = Logger(REPORT_PATH)

ser = serial.Serial(PORT, BAUD, timeout=0.1)
time.sleep(0.2)
ser.reset_input_buffer()

L.log("=" * 70)
L.log("FOC CLOSED-LOOP VERIFICATION REPORT")
L.log("=" * 70)

# Step 0: Stop
send_cmd(ser, "STOP")
time.sleep(0.5)

# Step 1: Align
L.log("\n[1] Running ALIGN (7.5s)...")
send_cmd(ser, "ALIGN")
align_samples = read_packets(ser, 7.5)
if align_samples:
    last = align_samples[-1]
    L.log(f"  Alignment Done: EncDir={last.get('encoder_dir')}, ZeroAngle={last.get('zero_elec_angle',0):.4f} rad ({math.degrees(last.get('zero_elec_angle',0)):.1f} deg)")

# Step 2: VQ 4.0V
L.log("\n[2] Testing VQ 4.0V (3.0s)...")
ser.reset_input_buffer()
send_cmd(ser, "VQ 4.0")
vq_samples = read_packets(ser, 3.0)
wait_for_standstill(ser, 4.0)
if vq_samples:
    ss = vq_samples[len(vq_samples)*2//3:]
    speeds = [s['speed_rpm'] for s in ss]
    L.log(f"  VQ 4.0V Result: Mean = {np.mean(speeds):+.1f} RPM, Max = {max(abs(s) for s in speeds):.1f} RPM")

# Step 3: Speed 100 RPM
L.log("\n[3] Testing SPEED 100 RPM Closed-Loop (4.0s)...")
ser.reset_input_buffer()
send_cmd(ser, "SPEED 100")
s100_samples = read_packets(ser, 4.0)
wait_for_standstill(ser, 4.0)
if s100_samples:
    ss = s100_samples[len(s100_samples)*2//3:]
    speeds = [s['speed_rpm'] for s in ss]
    vqs = [s.get('vq',0) for s in ss]
    iqs = [s['i_q'] for s in ss]
    L.log(f"  SPEED 100 Result: Actual Mean = {np.mean(speeds):+.1f} RPM (Target=100 RPM) | Vq = {np.mean(vqs):.2f}V | Iq = {np.mean(iqs):.3f}A")

# Step 4: Speed 200 RPM
L.log("\n[4] Testing SPEED 200 RPM Closed-Loop (4.0s)...")
ser.reset_input_buffer()
send_cmd(ser, "SPEED 200")
s200_samples = read_packets(ser, 4.0)
wait_for_standstill(ser, 5.0) # Wait until motor drops completely to 0 RPM
if s200_samples:
    ss = s200_samples[len(s200_samples)*2//3:]
    speeds = [s['speed_rpm'] for s in ss]
    vqs = [s.get('vq',0) for s in ss]
    iqs = [s['i_q'] for s in ss]
    L.log(f"  SPEED 200 Result: Actual Mean = {np.mean(speeds):+.1f} RPM (Target=200 RPM) | Vq = {np.mean(vqs):.2f}V | Iq = {np.mean(iqs):.3f}A")

# Step 5: Speed -100 RPM (Reverse from dead standstill)
L.log("\n[5] Testing SPEED -100 RPM Reverse Closed-Loop (4.0s)...")
ser.reset_input_buffer()
send_cmd(ser, "SPEED -100")
s_rev_samples = read_packets(ser, 4.0)
send_cmd(ser, "STOP")
if s_rev_samples:
    ss = s_rev_samples[len(s_rev_samples)*2//3:]
    speeds = [s['speed_rpm'] for s in ss]
    vqs = [s.get('vq',0) for s in ss]
    iqs = [s['i_q'] for s in ss]
    L.log(f"  SPEED -100 Result: Actual Mean = {np.mean(speeds):+.1f} RPM (Target=-100 RPM) | Vq = {np.mean(vqs):.2f}V | Iq = {np.mean(iqs):.3f}A")

send_cmd(ser, "STOP")
ser.close()

L.log("\n" + "=" * 70)
L.log(f"Verification complete. Log saved to: {REPORT_PATH}")
L.log("=" * 70)
L.close()
