#!/usr/bin/env python3
"""
High-Speed Reversal Transient Diagnostic Logger
Logs Vbus, Vq, raw encoder count, speed, and electrical angle during a rapid speed reversal (+150 RPM -> -100 RPM).
Diagnoses:
1. Speed PI Integrator Windup
2. Bench Power Supply Regenerative Overvoltage (Vbus surge)
3. Angle / Quadrant Discontinuities
"""
import sys, os, time, math
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'src'))
from telemetry_parser import TelemetryParser, PACKET_SIZE_94, MAGIC1, MAGIC2
import serial
import numpy as np

PORT = '/dev/ttyACM0'
BAUD = 115200

def send_cmd(ser, cmd):
    ser.write(f"{cmd}\r\n".encode())
    time.sleep(0.02)

def read_all_packets(ser, duration_sec):
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
        time.sleep(0.001)
    return samples

def main():
    print("=" * 75)
    print("REVERSAL TRANSIENT & REGEN VOLTAGE DIAGNOSTIC LOGGER")
    print("=" * 75)

    try:
        ser = serial.Serial(PORT, BAUD, timeout=0.05)
    except Exception as e:
        print(f"Error opening {PORT}: {e}")
        return

    time.sleep(0.2)
    ser.reset_input_buffer()

    send_cmd(ser, "STOP")
    time.sleep(0.3)

    # Step 1: Align
    print("\n[1] Running ALIGN (7.5s)...")
    send_cmd(ser, "ALIGN")
    read_all_packets(ser, 7.6)

    # Step 2: Spin up forward to +150 RPM (2.5s)
    print("\n[2] Spinning up to +150 RPM (2.5s)...")
    send_cmd(ser, "SPEED 150")
    s_fwd = read_all_packets(ser, 2.5)

    # Step 3: Rapid Reversal to -100 RPM (2.5s) with high-density logging
    print("\n[3] COMMANDING RAPID REVERSAL TO -100 RPM...")
    t_cmd = time.time()
    send_cmd(ser, "SPEED -100")
    s_rev = read_all_packets(ser, 2.5)

    send_cmd(ser, "STOP")
    ser.close()

    print(f"\n[4] Analyzing Reversal Transient ({len(s_rev)} samples collected):")
    print(f"{'Time(ms)':>8} | {'Vbus(V)':>7} | {'Vq(V)':>7} | {'Iq(A)':>7} | {'Speed(RPM)':>10} | {'Phase_e(rad)':>12} | {'Mech(rad)':>9}")
    print("-" * 75)

    vbus_vals = [s['v_bus'] for s in s_rev]
    max_vbus = max(vbus_vals)
    min_vbus = min(vbus_vals)
    vqs = [s.get('vq', 0.0) for s in s_rev]
    speeds = [s['speed_rpm'] for s in s_rev]

    # Print first 25 samples around reversal
    for i, s in enumerate(s_rev[:25]):
        print(f"{s['timestamp_ms']:8d} | {s['v_bus']:7.2f} | {s.get('vq',0):7.2f} | {s['i_q']:7.3f} | {s['speed_rpm']:10.1f} | {s['phase_elec']:12.4f} | {s['mech_angle']:9.4f}")

    print("-" * 75)
    print("\n" + "=" * 75)
    print("DIAGNOSTIC SUMMARY (EXPERT CRITERIA EVALUATION):")
    print(f"  Vbus Range during Reversal: {min_vbus:.2f}V -> {max_vbus:.2f}V (Delta = {max_vbus - min_vbus:+.2f}V)")
    if max_vbus > 26.5:
        print("  -> REGEN DETECTED: Vbus surged during braking! Bench power supply absorbed energy.")
    else:
        print("  -> Vbus is stable (No severe regen overvoltage trip).")

    # Check Vq sign response
    if vqs[0] < 0.0 or vqs[1] < 0.0:
        print("  -> Vq responded NEGATIVE immediately (Anti-windup verified, no windup delay!).")
    else:
        print(f"  -> Vq delay before flipping negative: {vqs[:5]}")

    print(f"  Final Steady-State Speed: {np.mean(speeds[len(speeds)*2//3:]):+.1f} RPM (Target = -100 RPM)")
    print("=" * 75)

if __name__ == '__main__':
    main()
