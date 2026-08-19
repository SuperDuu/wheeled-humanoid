#!/usr/bin/env python3
"""
Deep FOC Diagnostic - saves all output to file for agent to read.
"""
import sys, os, time, math, json
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'src'))
from telemetry_parser import TelemetryParser, PACKET_SIZE_94, MAGIC1, MAGIC2
import serial
import numpy as np

PORT = '/dev/ttyACM0'
BAUD = 115200
REPORT_PATH = os.path.join(os.path.dirname(os.path.dirname(__file__)), '..', 'foc_deep_diag.txt')
REPORT_PATH = os.path.abspath(REPORT_PATH)

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
L.log("DEEP FOC DIAGNOSTIC")
L.log("=" * 70)

# Step 0: Baseline
L.log("\n[0] Reading baseline (1s)...")
send_cmd(ser, "STOP")
time.sleep(0.2)
baseline = read_packets(ser, 1.0)
if baseline:
    b = baseline[-1]
    L.log(f"  Baseline: Mode={b['control_mode']}, State={b['motor_state']}, "
          f"Fault={b['fault_code']}, Vbus={b['v_bus']:.1f}V, "
          f"EncDir={b.get('encoder_dir','?')}, ZeroAngle={b.get('zero_elec_angle','?')}")

# Step 1: Alignment
L.log("\n[1] Running ALIGN (7.5s)...")
send_cmd(ser, "ALIGN")
align_samples = read_packets(ser, 7.5)
L.log(f"  Collected {len(align_samples)} alignment samples")
if align_samples:
    last = align_samples[-1]
    enc_dir = last.get('encoder_dir', '?')
    zero_ea = last.get('zero_elec_angle', 0)
    L.log(f"  encoder_dir={enc_dir}, zero_elec_angle={zero_ea:.4f} rad ({math.degrees(zero_ea):.2f} deg)")

# Step 2: VQ +6.0V
L.log("\n[2] VQ +6.0V test (3s)...")
send_cmd(ser, "VQ 6.0")
vq_samples = read_packets(ser, 3.0)
L.log(f"  Collected {len(vq_samples)} VQ samples")
if vq_samples:
    L.log("  --- First 5 VQ samples ---")
    for s in vq_samples[:5]:
        L.log(f"    RPM={s['speed_rpm']:+7.2f} Phase_e={s['phase_elec']:+6.3f} "
              f"Mech={s['mech_angle']:+6.3f} Iq={s['i_q']:+6.3f} Id={s['i_d']:+6.3f} "
              f"Vq={s.get('vq',0):+6.2f} Vd={s.get('vd',0):+6.2f} "
              f"Da={s['duty_a']:.3f} Db={s['duty_b']:.3f} Dc={s['duty_c']:.3f} "
              f"Ia={s['i_a']:+6.3f} Ib={s['i_b']:+6.3f} Ic={s['i_c']:+6.3f}")
    L.log("  --- Last 5 VQ samples ---")
    for s in vq_samples[-5:]:
        L.log(f"    RPM={s['speed_rpm']:+7.2f} Phase_e={s['phase_elec']:+6.3f} "
              f"Mech={s['mech_angle']:+6.3f} Iq={s['i_q']:+6.3f} Id={s['i_d']:+6.3f} "
              f"Vq={s.get('vq',0):+6.2f} Vd={s.get('vd',0):+6.2f} "
              f"Da={s['duty_a']:.3f} Db={s['duty_b']:.3f} Dc={s['duty_c']:.3f} "
              f"Ia={s['i_a']:+6.3f} Ib={s['i_b']:+6.3f} Ic={s['i_c']:+6.3f}")
    ss = vq_samples[len(vq_samples)//5:]
    iqs = [s['i_q'] for s in ss]
    ids = [s['i_d'] for s in ss]
    speeds = [s['speed_rpm'] for s in ss]
    L.log(f"\n  VQ STEADY-STATE ({len(ss)} samples):")
    L.log(f"    Speed: mean={np.mean(speeds):+.2f} RPM")
    L.log(f"    Iq:    mean={np.mean(iqs):+.4f} A (SHOULD BE POSITIVE)")
    L.log(f"    Id:    mean={np.mean(ids):+.4f} A")

# Step 3: IQ 1.0A
L.log("\n[3] IQ 1.0A test (3s)...")
send_cmd(ser, "IQ 1.0")
iq_samples = read_packets(ser, 3.0)
L.log(f"  Collected {len(iq_samples)} IQ samples")
if iq_samples:
    L.log("  --- ALL IQ samples (every 5th) ---")
    for i, s in enumerate(iq_samples):
        if i % 5 == 0 or i < 10:
            L.log(f"    [{i:3d}] RPM={s['speed_rpm']:+7.2f} Iq={s['i_q']:+7.3f} "
                  f"IqTgt={s['i_q_target']:+6.3f} Vq={s.get('vq',0):+7.2f} "
                  f"Vd={s.get('vd',0):+7.2f} Phase_e={s['phase_elec']:+6.3f} "
                  f"Ia={s['i_a']:+6.3f} Ib={s['i_b']:+6.3f} Ic={s['i_c']:+6.3f} "
                  f"Fault={s['fault_code']}")

send_cmd(ser, "STOP")
ser.close()

L.log("\n" + "=" * 70)
L.log(f"Report saved to: {REPORT_PATH}")
L.log("=" * 70)
L.close()

print(f"\n>>> Report saved to: {REPORT_PATH}")
