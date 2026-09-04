#!/usr/bin/env python3
"""
High-Density Diagnostic Comparison Experiment:
1. Pure Closed-Loop SPEED 20 for 17 revolutions (51s)
2. Open-Loop OPENLOOP 100 12.0 for 17 revolutions (10.2s)
High-rate live logging (every 0.5s / 2Hz) with continuous cumulative revolution tracking
and back-and-forth oscillation detection.
"""
import sys, os, time, math
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'src'))
from telemetry_parser import TelemetryParser, PACKET_SIZES, MAGIC1, MAGIC2
import serial
import numpy as np
import pandas as pd

PORT = '/dev/ttyACM0'
BAUD = 115200

def send_cmd(ser, cmd):
    ser.write(f"{cmd}\r\n".encode())
    ser.flush()
    time.sleep(0.05)

def read_packets_until(ser, stop_condition_fn, max_duration=65.0, label=""):
    buf = bytearray()
    records = []
    t_start = time.time()
    last_print_t = t_start
    last_mech = None
    unwrapped_mech = 0.0
    start_unwrapped = None
    osc_check_t = t_start
    osc_joint_anchor = None

    while time.time() - t_start < max_duration:
        try:
            waiting = ser.in_waiting
            if waiting > 0:
                raw = ser.read(waiting)
                if raw:
                    buf.extend(raw)
        except Exception:
            pass

        while len(buf) >= 78:
            if buf[0] != MAGIC1 or buf[1] != MAGIC2:
                del buf[0]
                continue

            packet_size = int(buf[3]) + 4
            if packet_size not in PACKET_SIZES:
                del buf[0]
                continue
            if len(buf) < packet_size:
                break

            candidate = bytes(buf[:packet_size])
            p = TelemetryParser.parse_packet(candidate)
            if p:
                del buf[:packet_size]
                p['t_elapsed'] = time.time() - t_start

                # Unwrapped mechanical angle tracking
                m = p['mech_angle']
                if last_mech is not None:
                    d_m = m - last_mech
                    if d_m > math.pi:
                        d_m -= 2.0 * math.pi
                    elif d_m < -math.pi:
                        d_m += 2.0 * math.pi
                    unwrapped_mech += d_m
                else:
                    unwrapped_mech = m
                last_mech = m
                p['mech_unwrapped'] = unwrapped_mech
                p['motor_revs'] = unwrapped_mech / (2.0 * math.pi)

                if start_unwrapped is None:
                    start_unwrapped = unwrapped_mech

                p['delta_revs'] = (unwrapped_mech - start_unwrapped) / (2.0 * math.pi)
                records.append(p)
                continue
            del buf[0]

        now = time.time()
        # High-density logging every 0.5 seconds (2 times per second)
        if now - last_print_t >= 0.5:
            elapsed = now - t_start
            last_print_t = now
            if records:
                r = records[-1]
                rpm = r['speed_rpm']
                iq = r['i_q']
                iq_t = r.get('i_q_target', 0.0)
                id_c = r['i_d']
                joint = r['joint_angle']
                delta_rev = r['delta_revs']
                th_e = r['phase_elec']
                vq = r.get('vq', 0.0)
                vd = r.get('vd', 0.0)

                print(f"[{label}] t={elapsed:4.1f}s | RPM={rpm:5.1f} | Iq={iq:5.2f}A(cmd:{iq_t:4.2f}) | Id={id_c:5.2f}A | Vq={vq:4.1f}V | Vd={vd:4.1f}V | Revs={delta_rev:5.2f}/17.0 | Joint={joint:6.2f} rad", flush=True)

        # Oscillation detector: check if motor is trapped shaking in place
        if now - osc_check_t >= 3.0:
            osc_check_t = now
            if records:
                cur_joint = records[-1]['joint_angle']
                if osc_joint_anchor is not None:
                    joint_diff = abs(cur_joint - osc_joint_anchor)
                    if joint_diff < 0.08 and time.time() - t_start > 5.0:
                        recent_iq = [abs(x['i_q']) for x in records[-20:]]
                        if np.mean(recent_iq) > 1.5:
                            print(f"   ⚠️ WARNING: Trapped at Joint={cur_joint:.2f} rad (delta={joint_diff:.3f} rad in 3s) while Iq={np.mean(recent_iq):.2f}A! Back-and-forth oscillation!", flush=True)
                osc_joint_anchor = cur_joint

        if stop_condition_fn(records, time.time() - t_start):
            break

        time.sleep(0.005)

    return records

print("=" * 80)
print("🔬 HIGH-DENSITY COMPARISON TEST: CLOSED-LOOP 20 RPM vs OPEN-LOOP 100 RPM")
print("=" * 80, flush=True)

# Disconnect any open API handles
import urllib.request
try:
    urllib.request.urlopen(urllib.request.Request('http://127.0.0.1:1111/api/disconnect', data=b'', headers={'Content-Type': 'application/json'}))
except Exception:
    pass
time.sleep(0.5)

ser = serial.Serial(PORT, BAUD, timeout=0.05)
time.sleep(0.2)
ser.reset_input_buffer()

# 1. Stop
print("\n[Step 0] Sending STOP...", flush=True)
send_cmd(ser, "STOP")
time.sleep(0.5)

# 2. Run ALIGN
print("\n[Step 1] Sending ALIGN (18 seconds)...", flush=True)
send_cmd(ser, "ALIGN")
t_align_start = time.time()
while time.time() - t_align_start < 18.0:
    try:
        if ser.in_waiting > 0:
            ser.read(ser.in_waiting)
    except Exception:
        pass
    time.sleep(0.1)
print("   ALIGN complete!", flush=True)

# -------------------------------------------------------------
# PHASE 1: Closed-Loop SPEED 20 for 17 motor revolutions
# -------------------------------------------------------------
print("\n[Step 2] >>> STARTING PHASE 1: CLOSED-LOOP SPEED 20 RPM (Target: 17 Motor Revs) <<<", flush=True)
send_cmd(ser, "SPEED 20")
time.sleep(0.5)

def stop_closed(records, elapsed):
    if len(records) < 50:
        return False
    delta_rev = records[-1]['delta_revs']
    # 17 motor revolutions completed
    if delta_rev >= 17.0:
        print(f"\n   ✅ Phase 1 complete: Reached {delta_rev:.2f} motor revolutions in {elapsed:.1f}s!", flush=True)
        return True
    return False

records_closed = read_packets_until(ser, stop_closed, max_duration=65.0, label="CLOSED-20")

print("\n[Step 3] Stopping motor after Phase 1...", flush=True)
send_cmd(ser, "STOP")
time.sleep(2.0)

# -------------------------------------------------------------
# PHASE 2: Open-Loop OPENLOOP 100 12.0 for 17 motor revolutions
# -------------------------------------------------------------
print("\n[Step 4] >>> STARTING PHASE 2: OPEN-LOOP 100 RPM 12.0V (Target: 17 Motor Revs) <<<", flush=True)
send_cmd(ser, "OPENLOOP 100 12.0")
time.sleep(0.5)

def stop_open(records, elapsed):
    if len(records) < 50:
        return False
    delta_rev = records[-1]['delta_revs']
    if delta_rev >= 17.0:
        print(f"\n   ✅ Phase 2 complete: Reached {delta_rev:.2f} motor revolutions in {elapsed:.1f}s!", flush=True)
        return True
    return False

records_open = read_packets_until(ser, stop_open, max_duration=20.0, label="OPEN-100")

print("\n[Step 5] Stopping motor after Phase 2...", flush=True)
send_cmd(ser, "STOP")
ser.close()

# -------------------------------------------------------------
# DETAILED ANALYSIS & COMPARISON
# -------------------------------------------------------------
print("\n" + "=" * 80)
print("📊 DETAILED DIAGNOSTIC COMPARISON REPORT")
print("=" * 80, flush=True)

df_cl = pd.DataFrame(records_closed) if records_closed else pd.DataFrame()
df_ol = pd.DataFrame(records_open) if records_open else pd.DataFrame()

if not df_cl.empty:
    df_cl.to_csv("diagnostic_closed_loop_20rpm.csv", index=False)
if not df_ol.empty:
    df_ol.to_csv("diagnostic_open_loop_100rpm.csv", index=False)

print("\n--- PHASE 1: CLOSED-LOOP 20 RPM ---")
if not df_cl.empty:
    cl_run = df_cl[df_cl['t_elapsed'] > 2.0]
    total_revs_cl = df_cl['delta_revs'].iloc[-1]
    print(f"Total Samples:          {len(df_cl)}")
    print(f"Duration:               {df_cl['t_elapsed'].iloc[-1]:.2f} s")
    print(f"Total Motor Revs:       {total_revs_cl:.2f} revs (Target: 17.00 revs)")
    print(f"Mean Speed:             {cl_run['speed_rpm'].mean():.2f} RPM (Std: {cl_run['speed_rpm'].std():.2f} RPM)")
    print(f"Min / Max Speed:        {cl_run['speed_rpm'].min():.2f} RPM / {cl_run['speed_rpm'].max():.2f} RPM")
    print(f"Mean Iq:                {cl_run['i_q'].mean():.2f} A (Std: {cl_run['i_q'].std():.2f} A)")
    print(f"Mean Id:                {cl_run['i_d'].mean():.2f} A (Std: {cl_run['i_d'].std():.2f} A)")
    print(f"Iq Min / Max:           {cl_run['i_q'].min():.2f} A / {cl_run['i_q'].max():.2f} A")
    print(f"I_RMS:                  {np.sqrt((cl_run['i_q']**2 + cl_run['i_d']**2).mean()):.2f} A")
    print(f"Joint Angle:            {df_cl['joint_angle'].iloc[0]:.2f} -> {df_cl['joint_angle'].iloc[-1]:.2f} rad (Delta = {df_cl['joint_angle'].iloc[-1] - df_cl['joint_angle'].iloc[0]:.2f} rad)")

print("\n--- PHASE 2: OPEN-LOOP 100 RPM ---")
if not df_ol.empty:
    ol_run = df_ol[df_ol['t_elapsed'] > 1.0]
    total_revs_ol = df_ol['delta_revs'].iloc[-1]
    print(f"Total Samples:          {len(df_ol)}")
    print(f"Duration:               {df_ol['t_elapsed'].iloc[-1]:.2f} s")
    print(f"Total Motor Revs:       {total_revs_ol:.2f} revs (Target: 17.00 revs)")
    print(f"Mean Speed:             {ol_run['speed_rpm'].mean():.2f} RPM (Std: {ol_run['speed_rpm'].std():.2f} RPM)")
    print(f"Min / Max Speed:        {ol_run['speed_rpm'].min():.2f} RPM / {ol_run['speed_rpm'].max():.2f} RPM")
    print(f"Mean Iq:                {ol_run['i_q'].mean():.2f} A (Std: {ol_run['i_q'].std():.2f} A)")
    print(f"Mean Id:                {ol_run['i_d'].mean():.2f} A (Std: {ol_run['i_d'].std():.2f} A)")
    print(f"Iq Min / Max:           {ol_run['i_q'].min():.2f} A / {ol_run['i_q'].max():.2f} A")
    print(f"I_RMS:                  {np.sqrt((ol_run['i_q']**2 + ol_run['i_d']**2).mean()):.2f} A")
    print(f"Joint Angle:            {df_ol['joint_angle'].iloc[0]:.2f} -> {df_ol['joint_angle'].iloc[-1]:.2f} rad (Delta = {df_ol['joint_angle'].iloc[-1] - df_ol['joint_angle'].iloc[0]:.2f} rad)")
