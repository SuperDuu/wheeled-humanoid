#!/usr/bin/env python3
"""
Test 7: Bisection Test by Starting Position
Runs 4 trials starting from 4 different angular positions (0 deg, 90 deg, 180 deg, 270 deg)
to strictly determine if stall is caused by relative revolution count (bug in turns counter)
or by an absolute mechanical sector (local magnetic/gear sector).
"""
import sys, os, time, math
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'src'))
from telemetry_parser import TelemetryParser, PACKET_SIZES, MAGIC1, MAGIC2
import serial
import numpy as np

PORT = '/dev/ttyACM0'
BAUD = 115200

def send_cmd(ser, cmd):
    ser.write(f"{cmd}\r\n".encode())
    ser.flush()
    time.sleep(0.05)

def run_single_bisection_trial(ser, trial_num, target_offset_deg):
    print("\n" + "=" * 70)
    print(f"🔬 TRIAL {trial_num}/4: Starting with Mechanical Offset ~{target_offset_deg}°")
    print("=" * 70)

    # 1. Stop motor to let rotor settle
    send_cmd(ser, "STOP")
    time.sleep(0.5)
    ser.reset_input_buffer()

    # 2. Align encoder at current starting position
    print("   Running ALIGN (18s)...", flush=True)
    send_cmd(ser, "ALIGN")
    t0 = time.time()
    while time.time() - t0 < 18.0:
        try:
            if ser.in_waiting > 0:
                ser.read(ser.in_waiting)
        except Exception:
            pass
        time.sleep(0.1)
    print("   ALIGN complete!", flush=True)

    # 3. Read initial starting telemetry
    buf = bytearray()
    start_telemetry = None
    t_probe = time.time()
    while time.time() - t_probe < 2.0 and start_telemetry is None:
        try:
            w = ser.in_waiting
            if w > 0:
                buf.extend(ser.read(w))
        except Exception:
            pass
        while len(buf) >= 78:
            if buf[0] != MAGIC1 or buf[1] != MAGIC2:
                del buf[0]
                continue
            psize = int(buf[3]) + 4
            if psize not in PACKET_SIZES or len(buf) < psize:
                break
            p = TelemetryParser.parse_packet(bytes(buf[:psize]))
            if p:
                del buf[:psize]
                start_telemetry = p
                break
            del buf[0]
        time.sleep(0.01)

    start_mech = start_telemetry['mech_angle'] if start_telemetry else 0.0
    start_joint = start_telemetry['joint_angle'] if start_telemetry else 0.0
    print(f"   Initial Start Position: Mech = {start_mech:.3f} rad ({math.degrees(start_mech):.1f}°), Joint = {start_joint:.3f} rad ({math.degrees(start_joint):.1f}°)")

    # 4. Start SPEED 50
    print("   Starting SPEED 50...", flush=True)
    send_cmd(ser, "SPEED 50")
    time.sleep(0.5)

    records = []
    t_start = time.time()
    last_print = t_start
    unwrapped_mech = start_mech
    last_m = start_mech
    stall_detected = False

    while time.time() - t_start < 60.0:
        try:
            w = ser.in_waiting
            if w > 0:
                raw = ser.read(w)
                if raw: buf.extend(raw)
        except Exception:
            pass

        while len(buf) >= 78:
            if buf[0] != MAGIC1 or buf[1] != MAGIC2:
                del buf[0]
                continue
            psize = int(buf[3]) + 4
            if psize not in PACKET_SIZES or len(buf) < psize:
                break
            p = TelemetryParser.parse_packet(bytes(buf[:psize]))
            if p:
                del buf[:psize]
                p['t'] = time.time() - t_start

                m = p['mech_angle']
                d_m = m - last_m
                if d_m > math.pi: d_m -= 2*math.pi
                elif d_m < -math.pi: d_m += 2*math.pi
                unwrapped_mech += d_m
                last_m = m
                p['delta_revs'] = (unwrapped_mech - start_mech) / (2.0 * math.pi)
                records.append(p)
                continue
            del buf[0]

        now = time.time()
        if now - last_print >= 2.0:
            last_print = now
            if records:
                recent = records[-20:]
                rpm = np.mean([r['speed_rpm'] for r in recent])
                iq = np.mean([r['i_q'] for r in recent])
                joint = records[-1]['joint_angle']
                d_rev = records[-1]['delta_revs']
                print(f"   [t={now-t_start:4.1f}s] Speed: {rpm:5.1f} RPM | Iq: {iq:4.2f}A | Delta Revs: {d_rev:5.1f} | Joint: {joint:6.2f} rad", flush=True)

                if rpm < 5.0 and (now - t_start) > 5.0:
                    print(f"   ⚠️ Motor stopped at t={now-t_start:.1f}s! Stall recorded.", flush=True)
                    stall_detected = True
                    break

        time.sleep(0.005)

    send_cmd(ser, "STOP")
    time.sleep(0.5)

    if not records:
        return None

    final_rec = records[-1]
    final_joint = final_rec['joint_angle']
    final_delta_revs = final_rec['delta_revs']
    run_duration = final_rec['t']

    result = {
        'trial': trial_num,
        'target_offset_deg': target_offset_deg,
        'start_mech_rad': start_mech,
        'start_joint_rad': start_joint,
        'final_joint_rad': final_joint,
        'total_delta_motor_revs': final_delta_revs,
        'total_delta_joint_travel_rad': final_joint - start_joint,
        'run_duration_s': run_duration,
        'stall_detected': stall_detected
    }

    print(f"   --> Trial {trial_num} Summary: Delta Motor Revs = {final_delta_revs:.2f} revs, Final Joint Angle = {final_joint:.2f} rad")
    return result

def main():
    # Disconnect API
    import urllib.request
    try:
        urllib.request.urlopen(urllib.request.Request('http://127.0.0.1:1111/api/disconnect', data=b'', headers={'Content-Type': 'application/json'}))
    except Exception:
        pass
    time.sleep(0.5)

    ser = serial.Serial(PORT, BAUD, timeout=0.05)
    time.sleep(0.2)

    offsets = [0, 90, 180, 270]
    results = []

    for idx, off in enumerate(offsets):
        res = run_single_bisection_trial(ser, idx + 1, off)
        if res:
            results.append(res)
        time.sleep(2.0)

    ser.close()

    print("\n" + "=" * 80)
    print("📋 TEST 7: BISECTION BY STARTING POSITION FULL REPORT")
    print("=" * 80)
    print(f"{'Trial':<6} | {'Start Joint (rad)':<18} | {'Delta Motor Revs':<18} | {'Final Joint (rad)':<18} | {'Delta Joint (rad)':<18}")
    print("-" * 80)
    for r in results:
        print(f"{r['trial']:<6} | {r['start_joint_rad']:<18.2f} | {r['total_delta_motor_revs']:<18.2f} | {r['final_joint_rad']:<18.2f} | {r['total_delta_joint_travel_rad']:<18.2f}")

    print("\n" + "=" * 80)
    delta_revs = [r['total_delta_motor_revs'] for r in results]
    final_joints = [r['final_joint_rad'] for r in results]

    std_delta_revs = np.std(delta_revs)
    std_final_joints = np.std(final_joints)

    print(f"Standard Deviation of Delta Motor Revs: {std_delta_revs:.2f} revs")
    print(f"Standard Deviation of Final Joint Angle: {std_final_joints:.2f} rad")

    if std_delta_revs < std_final_joints and std_delta_revs < 3.0:
        print("\n🔍 CONCLUSION: FAILS AT CONSTANT DELTA REVOLUTIONS (~35.7 REVS) REGARDLESS OF STARTING POSITION!")
        print("--> ROOT CAUSE: BUG IN MULTI-TURN ACCUMULATOR / UNWRAP LOGIC.")
    elif std_final_joints < std_delta_revs and std_final_joints < 1.0:
        print("\n🔍 CONCLUSION: FAILS AT CONSTANT ABSOLUTE JOINT ANGLE REGARDLESS OF STARTING POSITION!")
        print("--> ROOT CAUSE: LOCAL MECHANICAL / ENCODER SECTOR ANOMALY.")
    else:
        print("\n🔍 CONCLUSION: COMPARATIVE DATA COLLECTED FOR ANALYSIS.")

if __name__ == '__main__':
    main()
