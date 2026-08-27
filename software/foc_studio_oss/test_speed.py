#!/usr/bin/env python3
"""
Test Closed-Loop Speed & Position Control (Voltage-FOC mode)
"""
import sys, os, time, math
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'src'))
from telemetry_parser import TelemetryParser, PACKET_SIZE_94, MAGIC1, MAGIC2
import serial
import numpy as np

PORT = '/dev/ttyACM0'
BAUD = 115200
REPORT_PATH = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', 'foc_speed_test.txt'))

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
L.log("FOC CLOSED-LOOP SPEED TEST")
L.log("=" * 70)

# Step 0: Stop
send_cmd(ser, "STOP")
time.sleep(0.3)

# Step 1: Align
L.log("\n[1] Running ALIGN (7.5s)...")
send_cmd(ser, "ALIGN")
align_samples = read_packets(ser, 7.5)
if align_samples:
    last = align_samples[-1]
    L.log(f"  Alignment: EncDir={last.get('encoder_dir')}, ZeroAngle={last.get('zero_elec_angle',0):.4f} rad")

# Step 2: Test Closed-Loop Speed 50 RPM
L.log("\n[2] Testing SPEED 50 RPM (4s)...")
send_cmd(ser, "SPEED 50")
speed50_samples = read_packets(ser, 4.0)
L.log(f"  Collected {len(speed50_samples)} samples")
if speed50_samples:
    L.log("  --- First 5 samples ---")
    for s in speed50_samples[:5]:
        L.log(f"    RPM={s['speed_rpm']:+7.2f} (Target={s.get('speed_target_rpm',0):+7.2f}) | "
              f"Vq={s.get('vq',0):+6.2f} Vd={s.get('vd',0):+6.2f} | "
              f"Iq={s['i_q']:+6.3f} Id={s['i_d']:+6.3f} | Mech={s['mech_angle']:+6.3f}")
    L.log("  --- Last 5 samples ---")
    for s in speed50_samples[-5:]:
        L.log(f"    RPM={s['speed_rpm']:+7.2f} (Target={s.get('speed_target_rpm',0):+7.2f}) | "
              f"Vq={s.get('vq',0):+6.2f} Vd={s.get('vd',0):+6.2f} | "
              f"Iq={s['i_q']:+6.3f} Id={s['i_d']:+6.3f} | Mech={s['mech_angle']:+6.3f}")
    
    ss = speed50_samples[len(speed50_samples)//2:]
    speeds = [s['speed_rpm'] for s in ss]
    vqs = [s.get('vq',0) for s in ss]
    mechs = [s['mech_angle'] for s in ss]
    delta_mech = mechs[-1] - mechs[0]
    L.log(f"\n  SPEED 50 STEADY-STATE:")
    L.log(f"    Actual Speed: mean={np.mean(speeds):+.2f} RPM, std={np.std(speeds):.2f}")
    L.log(f"    Vq Applied:   mean={np.mean(vqs):+.2f} V")
    L.log(f"    Delta Mech:   {delta_mech:+.3f} rad ({math.degrees(delta_mech):+.1f} deg)")

# Step 3: Test Closed-Loop Speed 100 RPM
L.log("\n[3] Testing SPEED 100 RPM (4s)...")
send_cmd(ser, "SPEED 100")
speed100_samples = read_packets(ser, 4.0)
L.log(f"  Collected {len(speed100_samples)} samples")
if speed100_samples:
    ss = speed100_samples[len(speed100_samples)//2:]
    speeds = [s['speed_rpm'] for s in ss]
    vqs = [s.get('vq',0) for s in ss]
    mechs = [s['mech_angle'] for s in ss]
    delta_mech = mechs[-1] - mechs[0]
    L.log(f"\n  SPEED 100 STEADY-STATE:")
    L.log(f"    Actual Speed: mean={np.mean(speeds):+.2f} RPM, std={np.std(speeds):.2f}")
    L.log(f"    Vq Applied:   mean={np.mean(vqs):+.2f} V")
    L.log(f"    Delta Mech:   {delta_mech:+.3f} rad ({math.degrees(delta_mech):+.1f} deg)")

# Step 4: Test Reverse Speed -50 RPM
L.log("\n[4] Testing SPEED -50 RPM (4s)...")
send_cmd(ser, "SPEED -50")
speed_rev_samples = read_packets(ser, 4.0)
L.log(f"  Collected {len(speed_rev_samples)} samples")
if speed_rev_samples:
    ss = speed_rev_samples[len(speed_rev_samples)//2:]
    speeds = [s['speed_rpm'] for s in ss]
    vqs = [s.get('vq',0) for s in ss]
    mechs = [s['mech_angle'] for s in ss]
    delta_mech = mechs[-1] - mechs[0]
    L.log(f"\n  SPEED -50 STEADY-STATE:")
    L.log(f"    Actual Speed: mean={np.mean(speeds):+.2f} RPM, std={np.std(speeds):.2f}")
    L.log(f"    Vq Applied:   mean={np.mean(vqs):+.2f} V")
    L.log(f"    Delta Mech:   {delta_mech:+.3f} rad ({math.degrees(delta_mech):+.1f} deg)")

send_cmd(ser, "STOP")
ser.close()

L.log("\n" + "=" * 70)
L.log(f"Report saved to: {REPORT_PATH}")
L.log("=" * 70)
L.close()
