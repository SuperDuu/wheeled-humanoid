#!/usr/bin/env python3
"""
Automated 120-Second (2 Full Minutes) Closed-Loop 50 RPM Endurance Test
Logs every 2 seconds with unwrapped motor revolution tracking and strict 10s safety checks.
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

print("=" * 80)
print("⚡ FULL 120-SECOND (2-MINUTE) CLOSED-LOOP 50 RPM ENDURANCE TEST")
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
print("\n1. Sending STOP...", flush=True)
send_cmd(ser, "STOP")
time.sleep(0.5)

# 2. Run ALIGN
print("2. Sending ALIGN (18 seconds)...", flush=True)
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

# 3. Start SPEED 50
print("3. Starting SPEED 50 (Target: 120.0 seconds continuous run)...", flush=True)
send_cmd(ser, "SPEED 50")
time.sleep(0.5)

# 4. Record 120 seconds
print("4. Monitoring 2-Minute Endurance Run (High-Density Logging every 2.0s)...", flush=True)
buf = bytearray()
records = []
t_start = time.time()
last_print_t = t_start
target_duration = 120.0 # Exactly 2 full minutes
stall_counter = 0
last_mech = None
unwrapped_mech = 0.0
start_unwrapped = None

try:
    while time.time() - t_start < target_duration:
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

                # Multi-turn tracking
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
        # High-density log every 2.0 seconds
        if now - last_print_t >= 2.0:
            elapsed = now - t_start
            last_print_t = now
            if records:
                recent = records[-40:]
                rpm = np.mean([r['speed_rpm'] for r in recent])
                iq = np.mean([r['i_q'] for r in recent])
                id_c = np.mean([r['i_d'] for r in recent])
                joint = records[-1]['joint_angle']
                delta_rev = records[-1]['delta_revs']
                vq = records[-1].get('vq', 0.0)
                vd = records[-1].get('vd', 0.0)

                print(f"[{elapsed:5.1f}s / {target_duration:3.0f}s] Speed: {rpm:5.1f} RPM | Iq: {iq:5.2f}A | Id: {id_c:5.2f}A | Vq: {vq:4.1f}V | Vd: {vd:4.1f}V | Motor Revs: {delta_rev:6.1f} | Joint: {joint:6.2f} rad", flush=True)

                # Safety check: if speed < 5 RPM for 4 consecutive seconds, stop!
                if rpm < 5.0 and elapsed > 5.0:
                    stall_counter += 1
                    if stall_counter >= 2:
                        print(f"⚠️ SAFETY TRIP: Motor stalled at t={elapsed:.1f}s! Stopping immediately.", flush=True)
                        send_cmd(ser, "STOP")
                        break
                else:
                    stall_counter = 0

        time.sleep(0.005)

finally:
    print("\n🛑 Stopping motor...", flush=True)
    send_cmd(ser, "STOP")
    ser.close()

# 5. Final Report
print("\n" + "=" * 80)
print("📊 120-SECOND (2-MINUTE) ENDURANCE TEST FULL REPORT")
print("=" * 80, flush=True)

if not records:
    print("❌ No telemetry records collected.")
    sys.exit(1)

df = pd.DataFrame(records)
df.to_csv("endurance_run_2min_telemetry.csv", index=False)
total_time = df['t_elapsed'].iloc[-1]
total_motor_revs = df['delta_revs'].iloc[-1]
total_joint_travel = df['joint_angle'].iloc[-1] - df['joint_angle'].iloc[0]

running_df = df[(df['t_elapsed'] > 2.0) & (df['t_elapsed'] < total_time - 1.0)]

print(f"Total Samples Collected: {len(df)}")
print(f"Total Run Time:          {total_time:.2f} seconds (Target: 120.00 s)")
print(f"Total Motor Revolutions: {total_motor_revs:.2f} revs (Target: ~100 revs at 50 RPM)")
print(f"Total Joint Travel:      {total_joint_travel:.2f} rad ({math.degrees(total_joint_travel):.1f} deg / {total_joint_travel/(2*math.pi):.2f} joint turns)")

if not running_df.empty:
    speed = running_df['speed_rpm']
    iq = running_df['i_q']
    id_cur = running_df['i_d']

    print(f"Mean Speed:              {speed.mean():.2f} RPM (Target: 50.00 RPM)")
    print(f"Speed Std Dev:           {speed.std():.2f} RPM")
    print(f"Min / Max Speed:         {speed.min():.2f} RPM / {speed.max():.2f} RPM")
    print(f"Mean Iq (Torque):        {iq.mean():.2f} A")
    print(f"RMS Current:             {np.sqrt((iq**2 + id_cur**2).mean()):.2f} A")
    print(f"Mean Id (Field):         {id_cur.mean():.2f} A")

    if total_time >= 118.0 and speed.mean() >= 45.0:
        print("\n🏆 RESULT: PASSED! 2-MINUTE STABLE 50 RPM CLOSED-LOOP FOC ENDURANCE RUN FULLY ACHIEVED!")
    else:
        print(f"\n⚠️ RESULT: Completed {total_time:.1f}s, Mean Speed: {speed.mean():.1f} RPM.")
