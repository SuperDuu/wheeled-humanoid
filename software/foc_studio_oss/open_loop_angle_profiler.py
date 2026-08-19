#!/usr/bin/env python3
"""
Open-Loop Angle Profiler
Runs Open-Loop 60 RPM (which is known to work smoothly) and logs:
- open_loop_angle (stator angle)
- raw_enc_rad (sensor angle)
- Computed electrical angle
This reveals the EXACT true relationship between encoder angle and stator angle!
"""
import sys, os, time, math
sys.path.insert(0, os.path.join('software/foc_studio_oss/src'))
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
print("OPEN-LOOP SYNCHRONOUS ANGLE PROFILER")
print("=" * 70)

# Step 0: Stop
send_cmd(ser, "STOP")
time.sleep(0.2)

# Step 1: Run OPENLOOP at 60 RPM (Voltage = 8.0V)
print("\n[1] Running OPENLOOP 60 RPM (4.0s)...")
send_cmd(ser, "OPENLOOP 60 8.0")
samples = read_packets(ser, 4.0)
send_cmd(ser, "STOP")

print(f"Collected {len(samples)} samples during Open-Loop.")

if samples:
    # Take steady-state samples (last 60%)
    ss = samples[len(samples)*2//5:]
    
    print("\nSample Data (every 5th steady-state sample):")
    print(f"{'Time(ms)':>8} | {'Speed(RPM)':>10} | {'Mech(rad)':>9} | {'Phase(rad)':>10} | {'Iq(A)':>7} | {'Id(A)':>7} | {'Da':>5} {'Db':>5} {'Dc':>5}")
    print("-" * 75)
    
    for i, s in enumerate(ss):
        if i % 5 == 0:
            print(f"{s['timestamp_ms']:8d} | {s['speed_rpm']:10.2f} | {s['mech_angle']:9.4f} | {s['phase_elec']:10.4f} | {s['i_q']:7.3f} | {s['i_d']:7.3f} | {s['duty_a']:.3f} {s['duty_b']:.3f} {s['duty_c']:.3f}")
    
    # Analyze relationship between mech_angle and time
    mechs = [s['mech_angle'] for s in ss]
    times = [s['timestamp_ms'] * 1e-3 for s in ss]
    
    # Unwrap mechanical angle
    unwrapped_mech = np.unwrap(mechs)
    mech_speed_rad_s = (unwrapped_mech[-1] - unwrapped_mech[0]) / (times[-1] - times[0])
    mech_speed_rpm = mech_speed_rad_s * 60.0 / (2.0 * math.pi)
    
    print("\n" + "=" * 70)
    print("ANALYSIS RESULTS:")
    print(f"  Target Open-Loop Speed: +60.0 RPM")
    print(f"  Measured Actual Mech Speed: {mech_speed_rpm:+.2f} RPM")
    
    # Check direction of rotation
    if mech_speed_rpm > 10.0:
        print("  -> Physical rotor rotation is POSITIVE (+)")
        print("  -> encoder_direction = +1 is VERIFIED PHYSICALLY!")
    elif mech_speed_rpm < -10.0:
        print("  -> Physical rotor rotation is NEGATIVE (-)")
        print("  -> encoder_direction MUST BE -1!")
    else:
        print("  -> Motor did not rotate during open loop.")
    
    # Now analyze the PWM duty cycles vs encoder angle
    # From Da, Db, Dc we can reconstruct the applied voltage angle in stator:
    stator_angles = []
    rotor_elec_angles_p1 = []
    rotor_elec_angles_m1 = []
    
    for s in ss:
        da = s['duty_a'] - 0.5
        db = s['duty_b'] - 0.5
        dc = s['duty_c'] - 0.5
        # Inverse Clarke: alpha = da, beta = (da + 2*db)/sqrt(3)
        alpha = da
        beta = (da + 2.0*db) / math.sqrt(3.0)
        stator_ang = math.atan2(beta, alpha)
        stator_angles.append(stator_ang)
        
        # Rotor electrical angle if dir = +1:
        th_p1 = math.fmod(1 * 21 * s['mech_angle'], 2.0 * math.pi)
        while th_p1 > math.pi: th_p1 -= 2*math.pi
        while th_p1 < -math.pi: th_p1 += 2*math.pi
        rotor_elec_angles_p1.append(th_p1)
        
        # Rotor electrical angle if dir = -1:
        th_m1 = math.fmod(-1 * 21 * s['mech_angle'], 2.0 * math.pi)
        while th_m1 > math.pi: th_m1 -= 2*math.pi
        while th_m1 < -math.pi: th_m1 += 2*math.pi
        rotor_elec_angles_m1.append(th_m1)
    
    # Calculate phase difference between stator voltage and rotor angle for dir=+1
    diffs_p1 = []
    for st, rt in zip(stator_angles, rotor_elec_angles_p1):
        d = st - rt
        while d > math.pi: d -= 2*math.pi
        while d < -math.pi: d += 2*math.pi
        diffs_p1.append(d)
    
    # Calculate phase difference for dir=-1
    diffs_m1 = []
    for st, rt in zip(stator_angles, rotor_elec_angles_m1):
        d = st - rt
        while d > math.pi: d -= 2*math.pi
        while d < -math.pi: d += 2*math.pi
        diffs_m1.append(d)
    
    std_p1 = np.std(diffs_p1)
    std_m1 = np.std(diffs_m1)
    
    print(f"\n  Synchronous Tracking Quality:")
    print(f"    For DIR = +1: Phase diff std = {std_p1:.4f} rad, Mean diff = {np.mean(diffs_p1):+.4f} rad ({math.degrees(np.mean(diffs_p1)):+.1f} deg)")
    print(f"    For DIR = -1: Phase diff std = {std_m1:.4f} rad, Mean diff = {np.mean(diffs_m1):+.4f} rad ({math.degrees(np.mean(diffs_m1)):+.1f} deg)")
    
    if std_p1 < std_m1:
        true_dir = 1
        true_offset = np.mean(diffs_p1)
        print(f"\n  ===> TRUE CONFIGURATION DETERMINED FROM SYNCHRONOUS RUN:")
        print(f"       encoder_direction = +1")
        print(f"       Stator-Rotor Angle Offset = {true_offset:+.4f} rad ({math.degrees(true_offset):+.1f} deg)")
    else:
        true_dir = -1
        true_offset = np.mean(diffs_m1)
        print(f"\n  ===> TRUE CONFIGURATION DETERMINED FROM SYNCHRONOUS RUN:")
        print(f"       encoder_direction = -1")
        print(f"       Stator-Rotor Angle Offset = {true_offset:+.4f} rad ({math.degrees(true_offset):+.1f} deg)")

print("=" * 70)
ser.close()
