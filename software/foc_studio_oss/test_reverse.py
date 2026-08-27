#!/usr/bin/env python3
"""
Dedicated Reverse Closed-Loop Test:
1. ALIGN
2. SPEED -50 RPM (2s)
3. SPEED -100 RPM (3s)
4. SPEED -150 RPM (3s)
5. STOP
"""
import sys, os, time
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
print("REVERSE CLOSED-LOOP SPEED TEST")
print("=" * 70)

# Step 0: Stop
send_cmd(ser, "STOP")
time.sleep(0.3)

# Step 1: Align
print("\n[1] Running ALIGN (7.5s)...")
send_cmd(ser, "ALIGN")
read_packets(ser, 7.5)

# Step 2: Reverse Speed -50 RPM
print("\n[2] Testing SPEED -50 RPM (3.0s)...")
send_cmd(ser, "SPEED -50")
s50 = read_packets(ser, 3.0)
send_cmd(ser, "STOP")
if s50:
    speeds = [s['speed_rpm'] for s in s50[len(s50)//2:]]
    vqs = [s.get('vq',0) for s in s50[len(s50)//2:]]
    iqs = [s['i_q'] for s in s50[len(s50)//2:]]
    mechs = [s['mech_angle'] for s in s50]
    print(f"  Result -50 RPM: MeanSpeed = {np.mean(speeds):+.1f} RPM | Vq = {np.mean(vqs):.2f}V | Iq = {np.mean(iqs):.3f}A | DeltaMech = {mechs[-1]-mechs[0]:+.3f} rad")
time.sleep(0.4)

# Step 3: Reverse Speed -100 RPM
print("\n[3] Testing SPEED -100 RPM (3.5s)...")
send_cmd(ser, "SPEED -100")
s100 = read_packets(ser, 3.5)
send_cmd(ser, "STOP")
if s100:
    speeds = [s['speed_rpm'] for s in s100[len(s100)//2:]]
    vqs = [s.get('vq',0) for s in s100[len(s100)//2:]]
    iqs = [s['i_q'] for s in s100[len(s100)//2:]]
    mechs = [s['mech_angle'] for s in s100]
    print(f"  Result -100 RPM: MeanSpeed = {np.mean(speeds):+.1f} RPM | Vq = {np.mean(vqs):.2f}V | Iq = {np.mean(iqs):.3f}A | DeltaMech = {mechs[-1]-mechs[0]:+.3f} rad")
time.sleep(0.4)

# Step 4: Reverse Speed -150 RPM
print("\n[4] Testing SPEED -150 RPM (3.5s)...")
send_cmd(ser, "SPEED -150")
s150 = read_packets(ser, 3.5)
send_cmd(ser, "STOP")
if s150:
    speeds = [s['speed_rpm'] for s in s150[len(s150)//2:]]
    vqs = [s.get('vq',0) for s in s150[len(s150)//2:]]
    iqs = [s['i_q'] for s in s150[len(s150)//2:]]
    mechs = [s['mech_angle'] for s in s150]
    print(f"  Result -150 RPM: MeanSpeed = {np.mean(speeds):+.1f} RPM | Vq = {np.mean(vqs):.2f}V | Iq = {np.mean(iqs):.3f}A | DeltaMech = {mechs[-1]-mechs[0]:+.3f} rad")

send_cmd(ser, "STOP")
ser.close()
print("=" * 70)
