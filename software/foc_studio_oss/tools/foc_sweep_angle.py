#!/usr/bin/env python3
"""
Angle Fine Sweep (every 15 degrees from -180 to +180 deg) at VQ 8.0V
Directly finds the exact angle offset that produces maximum continuous rotation speed.
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
print("FOC 360-DEGREE ANGLE SWEEP (VQ 8.0V)")
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
enc_rad_raw = align_samples[-1].get('mech_angle', 0.0)
print(f"  Alignment base: raw_enc={enc_rad_raw:.4f} rad, zero_elec_angle={base_zero:.4f} rad ({math.degrees(base_zero):.1f} deg)")

# Sweep offsets for DIR=+1 and DIR=-1
for direction in [1, -1]:
    print(f"\n==================== TESTING DIR = {direction:+d} ====================")
    send_cmd(ser, f"DIR {direction}")
    
    # Calculate base offset for this direction
    raw_elec = (float(direction * 21) * enc_rad_raw)
    
    best_speed = 0.0
    best_deg = 0
    
    for offset_deg in range(-180, 181, 15):
        offset_rad = raw_elec + math.radians(offset_deg)
        while offset_rad > math.pi: offset_rad -= 2*math.pi
        while offset_rad < -math.pi: offset_rad += 2*math.pi
        
        send_cmd(ser, "STOP")
        send_cmd(ser, f"OFFSET {offset_rad:.4f}")
        time.sleep(0.03)
        
        # Apply VQ 8.0V for 1.2s
        send_cmd(ser, "VQ 8.0")
        samples = read_packets(ser, 1.2)
        send_cmd(ser, "STOP")
        
        if samples:
            speeds = [s['speed_rpm'] for s in samples[len(samples)//3:]]
            max_spd = max(abs(s) for s in speeds) if speeds else 0
            mean_spd = np.mean(speeds) if speeds else 0
            mechs = [s['mech_angle'] for s in samples]
            delta_mech = mechs[-1] - mechs[0]
            marker = " *** SPINS! ***" if max_spd > 50 else ""
            print(f"  offset={offset_deg:+4d} deg | MaxSpeed={max_spd:6.1f} RPM | MeanSpeed={mean_spd:+6.1f} RPM | DeltaMech={delta_mech:+6.2f} rad{marker}")
            if max_spd > best_speed:
                best_speed = max_spd
                best_deg = offset_deg
        time.sleep(0.05)
    
    print(f"\n>>> Best for DIR={direction:+d}: offset={best_deg:+d} deg with MaxSpeed={best_speed:.1f} RPM")

send_cmd(ser, "STOP")
ser.close()
