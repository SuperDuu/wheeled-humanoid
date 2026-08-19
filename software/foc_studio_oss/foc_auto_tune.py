#!/usr/bin/env python3
"""
FOC Auto-Angle & Polarity Discriminator
Automatically sweeps encoder direction (+1 / -1) and angle offsets (0, +90, -90, 180)
to find which configuration makes the motor spin freely on its own.
"""
import sys, os, time, math
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'src'))
from telemetry_parser import TelemetryParser, PACKET_SIZE_94, MAGIC1, MAGIC2
import serial
import numpy as np

PORT = '/dev/ttyACM0'
BAUD = 115200
REPORT_PATH = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', 'foc_autotune_results.txt'))

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

def send_cmd(ser, cmd):
    ser.write(f"{cmd}\r\n".encode())
    time.sleep(0.05)

L = Logger(REPORT_PATH)

ser = serial.Serial(PORT, BAUD, timeout=0.1)
time.sleep(0.2)
ser.reset_input_buffer()

L.log("=" * 70)
L.log("FOC POLARITY & ANGLE AUTO-DISCRIMINATOR")
L.log("DO NOT TOUCH OR PUSH MOTOR WITH HAND DURING THIS TEST!")
L.log("=" * 70)

# Step 0: Stop
send_cmd(ser, "STOP")
time.sleep(0.3)

# Step 1: Run ALIGN
L.log("\n[1] Running ALIGN to establish zero reference (7.5s)...")
send_cmd(ser, "ALIGN")
align_samples = read_packets(ser, 7.5)
if not align_samples:
    L.log("ERROR: No response from driver during ALIGN!")
    ser.close()
    L.close()
    sys.exit(1)

last = align_samples[-1]
base_offset = last.get('zero_elec_angle', 0.0)
enc_rad_raw = last.get('mech_angle', 0.0)
L.log(f"  Alignment base: raw_enc={enc_rad_raw:.4f} rad, zero_elec_angle={base_offset:.4f} rad ({math.degrees(base_offset):.1f} deg)")

# Configurations to test:
# For GB8115, 21 pole pairs:
# elec_angle = dir * 21 * raw_enc - zero_offset
# When rotor is at lock position:
# With dir = +1: zero_offset = +1 * 21 * raw_enc
# With dir = -1: zero_offset = -1 * 21 * raw_enc = -base_offset
offset_dir_plus = base_offset
offset_dir_minus = -base_offset

test_configs = [
    {"name": "CONFIG A: DIR=+1, Offset=Aligned (0 deg)",  "dir": 1,  "offset": offset_dir_plus},
    {"name": "CONFIG B: DIR=-1, Offset=Aligned (0 deg)",  "dir": -1, "offset": offset_dir_minus},
    {"name": "CONFIG C: DIR=+1, Offset=+90 deg (Q-lead)", "dir": 1,  "offset": offset_dir_plus + math.pi/2},
    {"name": "CONFIG D: DIR=+1, Offset=-90 deg (Q-lag)",  "dir": 1,  "offset": offset_dir_plus - math.pi/2},
    {"name": "CONFIG E: DIR=+1, Offset=180 deg (Inverted)","dir": 1,  "offset": offset_dir_plus + math.pi},
    {"name": "CONFIG F: DIR=-1, Offset=+90 deg",          "dir": -1, "offset": offset_dir_minus + math.pi/2},
    {"name": "CONFIG G: DIR=-1, Offset=-90 deg",          "dir": -1, "offset": offset_dir_minus - math.pi/2},
]

winner = None

for idx, cfg in enumerate(test_configs):
    L.log(f"\n--- [{idx+1}/{len(test_configs)}] Testing {cfg['name']} ---")
    send_cmd(ser, "STOP")
    time.sleep(0.1)
    
    # Configure direction and offset
    send_cmd(ser, f"DIR {cfg['dir']}")
    # Normalize offset to [-pi, +pi]
    off = cfg['offset']
    while off > math.pi: off -= 2*math.pi
    while off < -math.pi: off += 2*math.pi
    send_cmd(ser, f"OFFSET {off:.4f}")
    time.sleep(0.05)
    
    # Test VQ 4.0V for 2.0s
    send_cmd(ser, "VQ 4.0")
    samples = read_packets(ser, 2.0)
    send_cmd(ser, "STOP")
    
    if samples:
        speeds = [s['speed_rpm'] for s in samples[len(samples)//3:]]
        max_speed = max(abs(s) for s in speeds) if speeds else 0
        mean_speed = np.mean(speeds) if speeds else 0
        mechs = [s['mech_angle'] for s in samples]
        delta_mech = mechs[-1] - mechs[0]
        iqs = [s['i_q'] for s in samples[len(samples)//3:]]
        mean_iq = np.mean(iqs) if iqs else 0
        
        L.log(f"  Result: MaxSpeed={max_speed:.1f} RPM | MeanSpeed={mean_speed:+.1f} RPM | DeltaMech={delta_mech:+.3f} rad ({math.degrees(delta_mech):+.1f} deg) | MeanIq={mean_iq:+.3f}A")
        
        if max_speed > 30.0 and abs(delta_mech) > 3.0: # Spun continuously (> 1/2 turn)
            L.log(f"  >>> WINNER FOUND! Motor spun freely on its own: {mean_speed:+.1f} RPM!")
            winner = cfg
            winner['actual_speed'] = mean_speed
            break
        elif max_speed > 10.0:
            L.log(f"  Motor showed partial movement ({max_speed:.1f} RPM)")
        else:
            L.log(f"  Motor held/locked position (0 RPM)")
    time.sleep(0.2)

send_cmd(ser, "STOP")
ser.close()

L.log("\n" + "=" * 70)
if winner:
    L.log(f"SUCCESS: Working configuration is: {winner['name']}")
    L.log(f"  -> DIR: {winner['dir']}")
    L.log(f"  -> OFFSET: {winner['offset']:.4f} rad")
    L.log(f"  -> SPEED ACHIEVED: {winner['actual_speed']:+.1f} RPM")
else:
    L.log("No configuration spun continuously without external help.")
L.log("=" * 70)
L.close()
