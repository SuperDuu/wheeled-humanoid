#!/usr/bin/env python3
"""
Test Pure Feedforward Closed-Loop (Zero Feedback Noise/Oscillation)
"""
import sys, os, time, math
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'src'))
from telemetry_parser import TelemetryParser, PACKET_SIZE_94, MAGIC1, MAGIC2
import serial
import numpy as np

PORT = '/dev/ttyACM0'
BAUD = 115200

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

def send_cmd(ser, cmd):
    ser.write(f"{cmd}\r\n".encode())
    time.sleep(0.05)

ser = serial.Serial(PORT, BAUD, timeout=0.1)
time.sleep(0.2)
ser.reset_input_buffer()

print("=" * 70)
print("🚀 TESTING PURE FEEDFORWARD CLOSED-LOOP AT 200 RPM (20 SECONDS)")
print("=" * 70)

# Set pure feedforward: Kp=0, Ki=0, Kd=0, Flux=0.0116
send_cmd(ser, "FLUX 0.0116")
send_cmd(ser, "KP_S 0.00000")
send_cmd(ser, "KI_S 0.00000")
send_cmd(ser, "KD_S 0.00000")

# Step 1: Align
print("\n[1] Running ALIGN (7.5s)...")
send_cmd(ser, "ALIGN")
align_samples = read_packets(ser, 7.5)
if align_samples:
    last = align_samples[-1]
    print(f"  Alignment: EncDir={last.get('encoder_dir')}, ZeroAngle={last.get('zero_elec_angle',0):.4f} rad")

# Step 2: Continuous 200 RPM test for 20.0 seconds
print("\n[2] Running SPEED 200 RPM for 20.0 SECONDS...")
send_cmd(ser, "SPEED 200")
samples = read_packets(ser, 20.0)

send_cmd(ser, "STOP")
ser.close()

if not samples:
    print("❌ No data collected!")
    sys.exit(1)

print(f"  Collected {len(samples)} samples over 20.0 seconds.")

# Analyze performance in 4-second segments
seg_size = len(samples) // 5
for i in range(5):
    seg = samples[i*seg_size : (i+1)*seg_size]
    speeds = [s['speed_rpm'] for s in seg]
    vqs = [s.get('vq', 0) for s in seg]
    ids = [s.get('i_d', 0) for s in seg]
    iqs = [s.get('i_q', 0) for s in seg]
    t_start = i * 4.0
    t_end = (i + 1) * 4.0
    print(f"  Segment [{t_start:4.1f}s - {t_end:4.1f}s]: RPM = {np.mean(speeds):+6.1f} ± {np.std(speeds):4.2f} | Vq = {np.mean(vqs):4.2f}V | Id = {np.mean(ids):+5.2f}A, Iq = {np.mean(iqs):+5.2f}A")

print("\n" + "=" * 70)
print("TEST COMPLETE")
print("=" * 70)
