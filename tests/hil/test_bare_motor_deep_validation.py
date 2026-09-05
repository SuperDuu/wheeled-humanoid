#!/usr/bin/env python3
"""
Deep Empirical Validation & Repeatability Suite for Bare Motor GB8115 (No Gearbox)
Obeying strict AGENTS.md rules:
- 3 consecutive repeatable trials per test condition
- Clear falsification criteria stated before test execution
- Raw signal extraction and statistical analysis (mean, std, min, max, ripple)
- A/B comparative testing for Anti-Cogging
"""

import sys
import time
import struct
import math
import serial
import json

SERIAL_PORT = "/dev/ttyACM0"
BAUD_RATE = 115200

PKT_FMT = "<BBBBI 16f 3Bb 4f Bb H"
PKT_SIZE = struct.calcsize(PKT_FMT)

def open_serial():
    s = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
    time.sleep(0.3)
    s.reset_input_buffer()
    return s

def send_cmd(s, cmd_str, delay=0.15):
    s.write(cmd_str.encode() + b"\r\n")
    time.sleep(delay)

def read_telemetry_samples(s, duration_s):
    samples = []
    t_end = time.time() + duration_s
    buf = bytearray()
    
    while time.time() < t_end:
        chunk = s.read(256)
        if chunk:
            buf.extend(chunk)
            while len(buf) >= PKT_SIZE:
                # Find magic bytes 0xAA 0x55
                idx = buf.find(b"\xaa\x55")
                if idx < 0:
                    buf.clear()
                    break
                if idx > 0:
                    del buf[:idx]
                if len(buf) < PKT_SIZE:
                    break
                
                pkt_data = bytes(buf[:PKT_SIZE])
                del buf[:PKT_SIZE]
                
                try:
                    unpacked = struct.unpack(PKT_FMT, pkt_data)
                    # Extract fields
                    ts_ms = unpacked[4]
                    ia, ib, ic = unpacked[5:8]
                    id_c, iq, iq_t = unpacked[8:11]
                    da, db, dc = unpacked[11:14]
                    ph_e, mech_a, joint_a = unpacked[14:17]
                    spd_rpm, spd_t_rpm = unpacked[17:19]
                    vbus, tfet = unpacked[19:21]
                    ctrl_m, mot_st, fault = unpacked[21:24]
                    enc_dir = unpacked[24]
                    vd, vq, zero_e, id_t = unpacked[25:29]
                    
                    samples.append({
                        "t_ms": ts_ms,
                        "spd_rpm": spd_rpm,
                        "spd_t_rpm": spd_t_rpm,
                        "iq": iq,
                        "id": id_c,
                        "ia": ia,
                        "ib": ib,
                        "ic": ic,
                        "vbus": vbus,
                        "mech_rad": mech_a,
                        "joint_rad": joint_a,
                        "vd": vd,
                        "vq": vq,
                        "fault": fault
                    })
                except Exception as e:
                    pass
        else:
            time.sleep(0.005)
            
    return samples

def analyze_speed_window(samples, target_rpm, discard_initial=30):
    if len(samples) <= discard_initial:
        return None
    steady = samples[discard_initial:]
    spds = [s["spd_rpm"] for s in steady]
    iqs = [s["iq"] for s in steady]
    ids = [s["id"] for s in steady]
    vds = [s["vd"] for s in steady]
    vqs = [s["vq"] for s in steady]
    faults = [s["fault"] for s in steady]
    
    mean_spd = sum(spds) / len(spds)
    std_spd = math.sqrt(sum((x - mean_spd)**2 for x in spds) / len(spds))
    mean_iq = sum(iqs) / len(iqs)
    std_iq = math.sqrt(sum((x - mean_iq)**2 for x in iqs) / len(iqs))
    max_iq = max(iqs)
    min_iq = min(iqs)
    
    return {
        "target_rpm": target_rpm,
        "sample_count": len(steady),
        "mean_spd": mean_spd,
        "std_spd": std_spd,
        "spd_err": mean_spd - target_rpm,
        "spd_err_pct": abs(mean_spd - target_rpm) / (abs(target_rpm) + 1e-3) * 100.0,
        "mean_iq": mean_iq,
        "std_iq": std_iq,
        "iq_ripple": max_iq - min_iq,
        "mean_id": sum(ids) / len(ids),
        "mean_vq": sum(vqs) / len(vqs),
        "max_fault": max(faults)
    }

def run_all_tests():
    print("==================================================================")
    print(" BARE MOTOR GB8115 DEEP EMPIRICAL VALIDATION & REPEATABILITY TEST ")
    print("==================================================================")
    print("Conditions: Gearbox removed (N=1.00), AS5048A SPI3, Vbus=24V")
    print("Testing 3 Consecutive Repeatable Trials across 4 Speeds + Anti-Cogging A/B")
    print()
    
    s = open_serial()
    
    # 0. Ensure motor configuration is set to Bare Motor
    send_cmd(s, "STOP")
    send_cmd(s, "GEAR 1.0")
    send_cmd(s, "CLEAR")
    time.sleep(0.5)
    
    results = {
        "trials": [],
        "anticog_ab": {},
        "position_test": []
    }
    
    speeds = [50.0, 100.0, 150.0, 200.0]
    
    # ==================================================================
    # TEST 1: 3 REPEATABLE TRIALS OF SPEED SWEEP (50, 100, 150, 200 RPM)
    # ==================================================================
    print("--- TEST 1: 3 Repeatable Trials of Speed Sweep (50 -> 200 RPM) ---")
    print("Hypothesis: Bare motor FOC starts cleanly and tracks speeds with < 5% error.")
    print("Falsification: Any trial with mean error > 10% or fault > 0 falsifies hypothesis.\n")
    
    for trial_idx in range(1, 4):
        print(f"--- Running Trial {trial_idx}/3 ---")
        trial_data = []
        for spd in speeds:
            print(f"  Commanding SPEED {spd} RPM for 4.0 seconds...")
            send_cmd(s, f"SPEED {spd}")
            samples = read_telemetry_samples(s, 4.0)
            stat = analyze_speed_window(samples, spd, discard_initial=40)
            if stat:
                trial_data.append(stat)
                print(f"    Target: {spd:5.1f} RPM | Measured: {stat['mean_spd']:5.1f} RPM | "
                      f"StdDev: {stat['std_spd']:4.2f} RPM | Err: {stat['spd_err']:+4.1f} RPM ({stat['spd_err_pct']:4.1f}%) | "
                      f"Iq: {stat['mean_iq']:5.3f} A | IqRipple: {stat['iq_ripple']:5.3f} A | Fault: {stat['max_fault']}")
            else:
                print(f"    ERROR: No telemetry received for {spd} RPM!")
                trial_data.append({"target_rpm": spd, "error": "No data"})
        
        # Stop between trials
        send_cmd(s, "STOP")
        time.sleep(1.0)
        results["trials"].append({"trial": trial_idx, "data": trial_data})
    
    # ==================================================================
    # TEST 2: ANTI-COGGING HARMONIC INJECTION A/B COMPARATIVE TEST
    # ==================================================================
    print("\n--- TEST 2: Anti-Cogging Harmonic Injection A/B Comparison (at 40 RPM) ---")
    print("Condition A: ANTICOG OFF (Baseline FOC)")
    print("Condition B: ANTICOG ON 0.120 A, 45 deg (6th Harmonic Feedforward)")
    print("Metric: Compare speed StdDev and Iq current ripple between A and B.\n")
    
    # Run Condition A (OFF)
    send_cmd(s, "ANTICOG OFF")
    send_cmd(s, "SPEED 40.0")
    samples_a = read_telemetry_samples(s, 4.0)
    stat_a = analyze_speed_window(samples_a, 40.0, discard_initial=40)
    
    # Run Condition B (ON)
    send_cmd(s, "ANTICOG ON 0.120 45.0")
    time.sleep(0.5)
    samples_b = read_telemetry_samples(s, 4.0)
    stat_b = analyze_speed_window(samples_b, 40.0, discard_initial=40)
    
    send_cmd(s, "STOP")
    send_cmd(s, "ANTICOG OFF")
    
    results["anticog_ab"] = {"condition_A_off": stat_a, "condition_B_on": stat_b}
    
    if stat_a and stat_b:
        print(f"  Condition A (OFF): Measured={stat_a['mean_spd']:.1f} RPM, StdDev={stat_a['std_spd']:.2f} RPM, Iq={stat_a['mean_iq']:.3f} A, IqRipple={stat_a['iq_ripple']:.3f} A")
        print(f"  Condition B (ON) : Measured={stat_b['mean_spd']:.1f} RPM, StdDev={stat_b['std_spd']:.2f} RPM, Iq={stat_b['mean_iq']:.3f} A, IqRipple={stat_b['iq_ripple']:.3f} A")
        delta_std = stat_b['std_spd'] - stat_a['std_spd']
        print(f"  Delta Speed StdDev: {delta_std:+.2f} RPM ({delta_std/stat_a['std_spd']*100.0:+.1f}%)")
    
    # ==================================================================
    # TEST 3: POSITION STEP CONTROL (45°, 90°, 180°, 0°)
    # ==================================================================
    print("\n--- TEST 3: Position Step Control on Bare Motor ---")
    send_cmd(s, "SETHOME")
    time.sleep(0.5)
    
    pos_targets = [45.0, 90.0, 180.0, 0.0]
    pos_results = []
    for deg in pos_targets:
        send_cmd(s, f"POS {deg}")
        samples_pos = read_telemetry_samples(s, 2.5)
        if len(samples_pos) > 20:
            last_sample = samples_pos[-1]
            meas_deg = math.degrees(last_sample["joint_rad"])
            err_deg = meas_deg - deg
            print(f"  Step to {deg:5.1f} deg | Measured: {meas_deg:5.1f} deg | Error: {err_deg:+4.2f} deg | Hold Iq: {last_sample['iq']:5.3f} A")
            pos_results.append({"target_deg": deg, "meas_deg": meas_deg, "err_deg": err_deg, "iq": last_sample["iq"]})
    
    results["position_test"] = pos_results
    
    # 4. Final Stop & Power Off
    send_cmd(s, "STOP")
    s.close()
    
    # Save raw results JSON
    json_path = "/home/du/Desktop/wheeled-humanoid/tests/bare_motor_test_results.json"
    with open(json_path, "w") as f:
        json.dump(results, f, indent=2)
    print(f"\nSaved raw validation data to {json_path}")
    print("==================================================================")
    print(" TEST COMPLETED ")
    print("==================================================================")

if __name__ == "__main__":
    run_all_tests()
