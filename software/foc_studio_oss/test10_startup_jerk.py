#!/usr/bin/env python3
"""
Test 10: Startup Transient & Integral Carryover Test
Dumps 500 high-speed 10 kHz samples at the exact instant of SPEED command enable.
Compares:
  Scenario (b): Cold Start (fresh reset + ALIGN)
  Scenario (a): Hot Start (immediately after STOP from high-load run)
"""
import sys, os, time
import serial
import pandas as pd
import numpy as np

PORT = '/dev/ttyACM0'
BAUD = 115200

def send_cmd(ser, cmd):
    ser.write(f"{cmd}\r\n".encode())
    ser.flush()
    time.sleep(0.05)

def dump_startup_samples(ser):
    send_cmd(ser, "DUMP_STARTUP")
    lines = []
    t0 = time.time()
    recording = False
    
    while time.time() - t0 < 3.0:
        line = ser.readline().decode('utf-8', errors='ignore').strip()
        if not line:
            continue
        if "START_DIAG_DUMP" in line:
            recording = True
            continue
        if "END_DIAG_DUMP" in line:
            break
        if recording:
            parts = line.split(',')
            if len(parts) == 8:
                try:
                    lines.append([int(parts[0])] + [float(p) for p in parts[1:]])
                except Exception:
                    pass
    
    if lines:
        cols = ['sample_idx', 'speed_i_term', 'vd_int', 'vq_int', 'iq_target', 'iq_meas', 'vd', 'vq']
        df = pd.DataFrame(lines, columns=cols)
        return df
    return None

def main():
    # Disconnect API
    import urllib.request
    try:
        urllib.request.urlopen(urllib.request.Request('http://127.0.0.1:1111/api/disconnect', data=b'', headers={'Content-Type': 'application/json'}))
    except Exception:
        pass
    time.sleep(0.5)

    ser = serial.Serial(PORT, BAUD, timeout=0.1)
    time.sleep(0.2)
    ser.reset_input_buffer()

    print("=" * 80)
    print("🔬 TEST 10: STARTUP TRANSIENT & INTEGRAL CARRYOVER AUDIT (10 kHz / 100µs ISR)")
    print("=" * 80)

    # ----------------------------------------------------
    # SCENARIO B: Cold Start after ALIGN
    # ----------------------------------------------------
    print("\n--- SCENARIO (b): COLD START AFTER FRESH ALIGN ---")
    send_cmd(ser, "STOP")
    time.sleep(0.5)
    ser.reset_input_buffer()

    print("Running ALIGN (18s)...", flush=True)
    send_cmd(ser, "ALIGN")
    t0 = time.time()
    while time.time() - t0 < 18.0:
        if ser.in_waiting: ser.read(ser.in_waiting)
        time.sleep(0.1)
    print("ALIGN complete!")
    time.sleep(0.5)
    ser.reset_input_buffer()

    print("Sending SPEED 50 and capturing first 500 PWM ISR cycles (50ms @ 10kHz)...", flush=True)
    send_cmd(ser, "SPEED 50")
    time.sleep(0.1) # Let first 500 samples capture (50ms)
    send_cmd(ser, "STOP")
    time.sleep(0.2)
    ser.reset_input_buffer()

    df_b = dump_startup_samples(ser)
    if df_b is not None and not df_b.empty:
        print(f"Captured {len(df_b)} samples for Scenario (b)!")
        df_b.to_csv("test10_startup_scenario_b_cold.csv", index=False)
        print("First 10 ISR Cycles at t=0 (Scenario b):")
        print(df_b.head(10)[['sample_idx', 'speed_i_term', 'vd_int', 'vq_int', 'iq_target', 'iq_meas', 'vq']].to_string(index=False))
        max_iq_b = df_b['iq_meas'].abs().max()
        vq_int_0_b = df_b['vq_int'].iloc[0]
        speed_i_0_b = df_b['speed_i_term'].iloc[0]
        print(f"\n--> Scenario (b) Summary: vq_int(t=0) = {vq_int_0_b:.3f}V, speed_i_term(t=0) = {speed_i_0_b:.3f}A, Peak |Iq| = {max_iq_b:.3f}A")
    else:
        print("❌ Failed to capture startup dump for Scenario (b).")

    # ----------------------------------------------------
    # SCENARIO A: Hot Start (Enable after high-torque stall)
    # ----------------------------------------------------
    print("\n--- SCENARIO (a): HOT START (RE-ENABLE IMMEDIATELY AFTER STALL) ---")
    send_cmd(ser, "SPEED 50")
    print("Running motor for 30 seconds to build load / approach stall...", flush=True)
    time.sleep(30.0)
    print("Sending STOP...")
    send_cmd(ser, "STOP")
    time.sleep(0.2) # Short pause
    ser.reset_input_buffer()

    print("Re-enabling SPEED 50 immediately (HOT START)...", flush=True)
    send_cmd(ser, "SPEED 50")
    time.sleep(0.1)
    send_cmd(ser, "STOP")
    time.sleep(0.2)
    ser.reset_input_buffer()

    df_a = dump_startup_samples(ser)
    if df_a is not None and not df_a.empty:
        print(f"Captured {len(df_a)} samples for Scenario (a)!")
        df_a.to_csv("test10_startup_scenario_a_hot.csv", index=False)
        print("First 10 ISR Cycles at t=0 (Scenario a):")
        print(df_a.head(10)[['sample_idx', 'speed_i_term', 'vd_int', 'vq_int', 'iq_target', 'iq_meas', 'vq']].to_string(index=False))
        max_iq_a = df_a['iq_meas'].abs().max()
        vq_int_0_a = df_a['vq_int'].iloc[0]
        speed_i_0_a = df_a['speed_i_term'].iloc[0]
        print(f"\n--> Scenario (a) Summary: vq_int(t=0) = {vq_int_0_a:.3f}V, speed_i_term(t=0) = {speed_i_0_a:.3f}A, Peak |Iq| = {max_iq_a:.3f}A")
    else:
        print("❌ Failed to capture startup dump for Scenario (a).")

    ser.close()

if __name__ == '__main__':
    main()
