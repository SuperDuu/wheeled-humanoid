#!/usr/bin/env python3
"""
Test single working configuration:
1. ALIGN
2. Set OFFSET = (aligned_offset + pi/2)
3. Run VQ 6.0V
4. Run SPEED 100 RPM
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
print("TESTING WORKING CONFIG (DIR=1, OFFSET = ALIGN + PI/2)")
print("=" * 70)

# Step 0: Stop
send_cmd(ser, "STOP")
time.sleep(0.2)

# Step 1: Align
print("\n[1] Running ALIGN (7.5s)...")
send_cmd(ser, "ALIGN")
align_samples = read_packets(ser, 7.5)
if not align_samples:
    print("ERROR: No response from driver!")
    ser.close()
    sys.exit(1)

base_zero = align_samples[-1].get('zero_elec_angle', 0.0)
enc_rad = align_samples[-1].get('mech_angle', 0.0)
print(f"  Alignment base: raw_enc={enc_rad:.4f} rad, zero_elec_angle={base_zero:.4f} rad")

# Test 1: VQ 6.0V with current offset
print("\n[2] Testing VQ 6.0V with ALIGN offset as-is (2s)...")
send_cmd(ser, "DIR 1")
send_cmd(ser, "VQ 6.0")
s_asis = read_packets(ser, 2.0)
send_cmd(ser, "STOP")
if s_asis:
    speeds = [s['speed_rpm'] for s in s_asis[len(s_asis)//3:]]
    print(f"  Result as-is: MeanSpeed = {np.mean(speeds):+.1f} RPM, MaxSpeed = {max(abs(s) for s in speeds):.1f} RPM")
time.sleep(0.3)

# Test 2: VQ 6.0V with offset + PI/2
off_plus_90 = base_zero + math.pi/2
while off_plus_90 > math.pi: off_plus_90 -= 2*math.pi
while off_plus_90 < -math.pi: off_plus_90 += 2*math.pi

print(f"\n[3] Testing VQ 6.0V with OFFSET = {off_plus_90:.4f} (+90 deg) (3s)...")
send_cmd(ser, f"OFFSET {off_plus_90:.4f}")
send_cmd(ser, "VQ 6.0")
s_plus90 = read_packets(ser, 3.0)
send_cmd(ser, "STOP")
if s_plus90:
    speeds = [s['speed_rpm'] for s in s_plus90[len(s_plus90)//3:]]
    mechs = [s['mech_angle'] for s in s_plus90]
    print(f"  Result +90 deg: MeanSpeed = {np.mean(speeds):+.1f} RPM, MaxSpeed = {max(abs(s) for s in speeds):.1f} RPM, DeltaMech = {mechs[-1]-mechs[0]:+.3f} rad")
time.sleep(0.3)

# Test 3: VQ 6.0V with offset - PI/2
off_minus_90 = base_zero - math.pi/2
while off_minus_90 > math.pi: off_minus_90 -= 2*math.pi
while off_minus_90 < -math.pi: off_minus_90 += 2*math.pi

print(f"\n[4] Testing VQ 6.0V with OFFSET = {off_minus_90:.4f} (-90 deg) (3s)...")
send_cmd(ser, f"OFFSET {off_minus_90:.4f}")
send_cmd(ser, "VQ 6.0")
s_minus90 = read_packets(ser, 3.0)
send_cmd(ser, "STOP")
if s_minus90:
    speeds = [s['speed_rpm'] for s in s_minus90[len(s_minus90)//3:]]
    mechs = [s['mech_angle'] for s in s_minus90]
    print(f"  Result -90 deg: MeanSpeed = {np.mean(speeds):+.1f} RPM, MaxSpeed = {max(abs(s) for s in speeds):.1f} RPM, DeltaMech = {mechs[-1]-mechs[0]:+.3f} rad")
time.sleep(0.3)

send_cmd(ser, "STOP")
ser.close()
