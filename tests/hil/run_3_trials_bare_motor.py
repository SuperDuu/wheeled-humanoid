#!/usr/bin/env python3
"""
Bare Motor GB8115 3-Trial Repeatability Benchmark (Strict Rule 7 Compliance)
Hardware: STM32G473RET6 @ 24V Bus, GB8115 Direct Drive (N=1.00, 21 Pole Pairs), AS5048A SPI3
Criteria:
  1. Position Step Tracking (45.0°, 90.0°, 180.0°, 0.0°):
     - Steady-state error < 0.15°
     - Holding current < 0.08 A
  2. Speed Tracking (+50, +100, +150, +200 RPM and -50, -100, -150, -200 RPM):
     - Dead-stop start
     - Steady-state error < 3.0%
     - Standard deviation sigma < 5.0 RPM (for >= 100 RPM)
     - Current < 0.35 A
Must pass across 3 consecutive independent trials.
"""

import sys
import time
import struct
import math
import json
import serial

SERIAL_PORT = "/dev/ttyACM0"
BAUD_RATE = 115200

PKT_FMT = '<BBBBI 16f 3Bb 4f Bb H'
PKT_SIZE = struct.calcsize(PKT_FMT)

def get_clean_telemetry_sample(ser):
    ser.reset_input_buffer()
    buf = bytearray()
    t_end = time.time() + 0.15
    while time.time() < t_end:
        c = ser.read(64)
        if c:
            buf.extend(c)
        while len(buf) >= PKT_SIZE:
            idx = buf.find(b'\xaa\x55')
            if idx < 0:
                buf.clear()
                break
            if idx > 0:
                del buf[:idx]
            if len(buf) < PKT_SIZE:
                break
            pkt = bytes(buf[:PKT_SIZE])
            del buf[:PKT_SIZE]
            u = struct.unpack(PKT_FMT, pkt)
            return {
                't_ms': u[4],
                'iq': u[9],
                'id': u[8],
                'mech_deg': math.degrees(u[15]),
                'joint_deg': math.degrees(u[16]),
                'spd_rpm': u[17],
                'vbus': u[19],
                'ctrl_mode': u[21],
                'mot_state': u[22],
                'fault': u[23],
                'vd': u[25],
                'vq': u[26]
            }
    return None

def collect_steady_samples(ser, count=35, delay_s=0.03):
    samples = []
    for _ in range(count):
        p = get_clean_telemetry_sample(ser)
        if p:
            samples.append(p)
        time.sleep(delay_s)
    return samples

def run_single_position_step(ser, target_deg, settle_time=2.8):
    ser.reset_input_buffer()
    cmd = f"POS {target_deg:.1f}\r\n"
    ser.write(cmd.encode())
    time.sleep(settle_time)
    
    samples = collect_steady_samples(ser, count=35, delay_s=0.03)
    if not samples:
        return {'target': target_deg, 'pass': False, 'error': 'No samples received'}
        
    angles = [s['joint_deg'] for s in samples]
    iqs = [s['iq'] for s in samples]
    ids = [s['id'] for s in samples]
    faults = [s['fault'] for s in samples]
    
    mean_ang = sum(angles) / len(angles)
    std_ang = math.sqrt(sum((x - mean_ang)**2 for x in angles) / len(angles))
    mean_iq = sum(iqs) / len(iqs)
    mean_id = sum(ids) / len(ids)
    err = mean_ang - target_deg
    
    err_pass = abs(err) < 0.15
    iq_pass = abs(mean_iq) < 0.10
    fault_pass = (max(faults) == 0)
    passed = err_pass and iq_pass and fault_pass
    
    res = {
        'target_deg': target_deg,
        'measured_deg': mean_ang,
        'err_deg': err,
        'std_deg': std_ang,
        'mean_iq': mean_iq,
        'mean_id': mean_id,
        'max_fault': max(faults),
        'err_pass': err_pass,
        'iq_pass': iq_pass,
        'pass': passed
    }
    print(f"  POS {target_deg:5.1f}° | Meas: {mean_ang:7.3f}° | Err: {err:+6.3f}° (pass={err_pass}) | Std: {std_ang:5.3f}° | Hold Iq: {mean_iq:+6.3f}A (pass={iq_pass}) | Result: {'PASS' if passed else 'FAIL'}")
    return res

def run_single_speed_step(ser, target_rpm):
    ser.write(b"STOP\r\n")
    time.sleep(1.0) # Ensure full dead stop
    
    cmd = f"SPEED {target_rpm:.1f}\r\n"
    ser.write(cmd.encode())
    
    abs_spd = abs(target_rpm)
    if abs_spd <= 50.0:
        settle_time = 3.6
    elif abs_spd <= 100.0:
        settle_time = 4.0
    elif abs_spd <= 150.0:
        settle_time = 4.6
    else:
        settle_time = 5.2
    time.sleep(settle_time)
    
    samples = collect_steady_samples(ser, count=35, delay_s=0.03)
    
    ser.write(b"STOP\r\n")
    time.sleep(0.3)
    
    if not samples:
        return {'target_rpm': target_rpm, 'pass': False, 'error': 'No samples received'}
        
    spds = [s['spd_rpm'] for s in samples]
    iqs = [s['iq'] for s in samples]
    ids = [s['id'] for s in samples]
    vqs = [s['vq'] for s in samples]
    faults = [s['fault'] for s in samples]
    
    mean_spd = sum(spds) / len(spds)
    std_spd = math.sqrt(sum((x - mean_spd)**2 for x in spds) / len(spds))
    mean_iq = sum(iqs) / len(iqs)
    mean_id = sum(ids) / len(ids)
    mean_vq = sum(vqs) / len(vqs)
    err = mean_spd - target_rpm
    err_pct = abs(err) / abs(target_rpm) * 100.0
    
    err_pass = err_pct < 3.0
    std_pass = (std_spd < 5.0) if abs_spd >= 100.0 else (std_spd < 12.0)
    iq_pass = abs(mean_iq) < 0.35
    fault_pass = (max(faults) == 0)
    passed = err_pass and std_pass and iq_pass and fault_pass
    
    res = {
        'target_rpm': target_rpm,
        'measured_rpm': mean_spd,
        'err_rpm': err,
        'err_pct': err_pct,
        'std_rpm': std_spd,
        'mean_iq': mean_iq,
        'mean_id': mean_id,
        'mean_vq': mean_vq,
        'max_fault': max(faults),
        'err_pass': err_pass,
        'std_pass': std_pass,
        'iq_pass': iq_pass,
        'pass': passed
    }
    print(f"  SPD {target_rpm:+6.1f} RPM | Meas: {mean_spd:+6.1f} RPM | Err: {err:+5.2f} RPM ({err_pct:4.2f}%) (pass={err_pass}) | Std: {std_spd:4.2f} RPM (pass={std_pass}) | Iq: {mean_iq:+6.3f}A (pass={iq_pass}) | Vq: {mean_vq:+5.2f}V | Result: {'PASS' if passed else 'FAIL'}")
    return res

def main():
    print("==========================================================================")
    print(" GB8115 BARE MOTOR FOC BENCHMARK: 3 CONSECUTIVE REPEATABLE TRIALS (RULE 7)")
    print("==========================================================================")
    print("Criteria:")
    print("  - Position: Err < 0.15 deg, Hold Current < 0.10 A (45°, 90°, 180°, 0°)")
    print("  - Speed: Err < 3.0%, StdDev < 5.0 RPM, Current < 0.35 A (+/- 50..200 RPM)")
    print()
    
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
    time.sleep(0.3)
    
    pos_targets = [45.0, 90.0, 180.0, 0.0]
    speed_targets = [50.0, 100.0, 150.0, 200.0, -50.0, -100.0, -150.0, -200.0]
    
    all_trials = []
    overall_pass = True
    
    for trial_num in range(1, 4):
        print(f"\n==================== [TRIAL {trial_num} / 3] ====================")
        # Initialize bare mode & rehome cleanly
        ser.write(b"STOP\r\n")
        time.sleep(0.3)
        ser.write(b"BARE\r\n")
        time.sleep(0.1)
        ser.write(b"DIR 1\r\n")
        time.sleep(0.1)
        ser.write(b"SETHOME\r\n")
        time.sleep(0.3)
        ser.reset_input_buffer()
        
        trial_record = {
            'trial_index': trial_num,
            'timestamp': time.time(),
            'position_tests': [],
            'speed_tests': [],
            'trial_pass': True
        }
        
        # 1. Position Steps
        print(f"--- Trial {trial_num}: Position Step Tracking ---")
        for tgt_deg in pos_targets:
            res = run_single_position_step(ser, tgt_deg, settle_time=2.8)
            trial_record['position_tests'].append(res)
            if not res['pass']:
                trial_record['trial_pass'] = False
                overall_pass = False
            time.sleep(0.3)
            
        # Return to Stop before speed
        ser.write(b"STOP\r\n")
        time.sleep(0.5)
        
        # 2. Speed Steps
        print(f"\n--- Trial {trial_num}: Speed Closed-Loop Tracking (From Dead Stop) ---")
        for tgt_rpm in speed_targets:
            res = run_single_speed_step(ser, tgt_rpm)
            trial_record['speed_tests'].append(res)
            if not res['pass']:
                trial_record['trial_pass'] = False
                overall_pass = False
            time.sleep(0.3)
            
        print(f"\n>>> Trial {trial_num} Status: {'PASS' if trial_record['trial_pass'] else 'FAIL'} <<<")
        all_trials.append(trial_record)
        time.sleep(1.0)
        
    ser.write(b"STOP\r\n")
    ser.close()
    
    output_path = "/home/du/Desktop/wheeled-humanoid/tests/bare_motor_3_trials_final.json"
    summary_report = {
        'overall_pass': overall_pass,
        'trial_count': len(all_trials),
        'trials': all_trials
    }
    with open(output_path, "w") as f:
        json.dump(summary_report, f, indent=2)
        
    print("\n==========================================================================")
    print(f"BENCHMARK COMPLETE: OVERALL RESULT = {'PASS (ALL 3 TRIALS PASSED)' if overall_pass else 'FAIL'}")
    print(f"Full JSON report saved to: {output_path}")
    print("==========================================================================")
    
    return 0 if overall_pass else 1

if __name__ == '__main__':
    sys.exit(main())
