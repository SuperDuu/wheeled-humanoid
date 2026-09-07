import serial
import time
import struct
import math
import json

PKT_FMT = '<BBBBI 16f 3Bb 4f Bb H'
PKT_SIZE = struct.calcsize(PKT_FMT)

s = serial.Serial('/dev/ttyACM0', 115200, timeout=0.02)
time.sleep(0.1)

def get_latest():
    latest = None
    buf = bytearray()
    t_end = time.time() + 0.05
    while time.time() < t_end or s.in_waiting > 0:
        c = s.read(s.in_waiting or 64)
        if c: buf.extend(c)
        while len(buf) >= PKT_SIZE:
            idx = buf.find(b'\xaa\x55')
            if idx < 0: buf.clear(); break
            if idx > 0: del buf[:idx]
            if len(buf) < PKT_SIZE: break
            pkt = bytes(buf[:PKT_SIZE])
            del buf[:PKT_SIZE]
            try:
                unp = struct.unpack(PKT_FMT, pkt)
                latest = {
                    'spd': unp[17],
                    'spd_tgt': unp[18],
                    'iq': unp[9],
                    'id': unp[8],
                    'joint_deg': math.degrees(unp[16]),
                    'mech_deg': math.degrees(unp[15]),
                    'vd': unp[25],
                    'vq': unp[26],
                    'vbus': unp[19],
                    'fault': unp[23]
                }
            except: pass
    return latest

def run_position_trial(trial_num):
    print(f"\n--- [TRIAL {trial_num}] POSITION STEP TRACKING ---")
    s.write(b"STOP\r\n")
    time.sleep(0.2)
    s.write(b"GEAR 1.0\r\n")
    time.sleep(0.05)
    s.write(b"SETHOME\r\n")
    time.sleep(0.1)
    
    pos_targets = [45.0, 90.0, 180.0, 0.0]
    trial_results = []
    
    for tgt in pos_targets:
        s.write(f"POS {tgt:.1f}\r\n".encode())
        settle_time = 1.8 if abs(tgt) <= 90.0 else 2.2
        time.sleep(settle_time)
        
        samples = []
        for _ in range(30):
            time.sleep(0.03)
            p = get_latest()
            if p: samples.append(p)
            
        if samples:
            angles = [x['joint_deg'] for x in samples]
            iqs = [x['iq'] for x in samples]
            ids = [x['id'] for x in samples]
            mean_ang = sum(angles) / len(angles)
            std_ang = math.sqrt(sum((x - mean_ang)**2 for x in angles) / len(angles))
            mean_iq = sum(iqs) / len(iqs)
            mean_id = sum(ids) / len(ids)
            err = mean_ang - tgt
            res = {
                'target': tgt,
                'measured': mean_ang,
                'err': err,
                'std': std_ang,
                'iq': mean_iq,
                'id': mean_id
            }
            trial_results.append(res)
            print(f"Target: {tgt:5.1f}° | Measured: {mean_ang:6.2f}° | Err: {err:+5.2f}° | Std: {std_ang:5.3f}° | Hold Iq: {mean_iq:6.3f}A | Id: {mean_id:6.3f}A")
        else:
            print(f"Target: {tgt:5.1f}° | FAILED TO RECEIVE TELEMETRY")
            
    s.write(b"STOP\r\n")
    time.sleep(0.2)
    return trial_results

def run_speed_trial(trial_num):
    print(f"\n--- [TRIAL {trial_num}] SPEED CLOSED-LOOP TRACKING (FROM DEAD STOP) ---")
    s.write(b"STOP\r\n")
    time.sleep(0.3)
    s.write(b"GEAR 1.0\r\n")
    time.sleep(0.05)
    
    speed_targets = [50.0, 100.0, 150.0, 200.0]
    trial_results = []
    
    for spd in speed_targets:
        s.write(b"STOP\r\n")
        time.sleep(0.8) # Ensure full dead stop
        
        s.write(f"SPEED {spd:.1f}\r\n".encode())
        if spd <= 50.0:
            settle_t = 2.0
        elif spd <= 100.0:
            settle_t = 2.6
        elif spd <= 150.0:
            settle_t = 3.5
        else:
            settle_t = 4.5
        time.sleep(settle_t) # Allow ramp up and settle
        
        samples = []
        for _ in range(30):
            time.sleep(0.03)
            p = get_latest()
            if p: samples.append(p)
            
        s.write(b"STOP\r\n")
        time.sleep(0.3)
        
        if samples:
            spds = [x['spd'] for x in samples]
            iqs = [x['iq'] for x in samples]
            ids = [x['id'] for x in samples]
            vqs = [x['vq'] for x in samples]
            mean_spd = sum(spds) / len(spds)
            std_spd = math.sqrt(sum((x - mean_spd)**2 for x in spds) / len(spds))
            mean_iq = sum(iqs) / len(iqs)
            mean_id = sum(ids) / len(ids)
            mean_vq = sum(vqs) / len(vqs)
            err = mean_spd - spd
            err_pct = abs(err) / spd * 100.0
            res = {
                'target': spd,
                'measured': mean_spd,
                'err': err,
                'err_pct': err_pct,
                'std': std_spd,
                'iq': mean_iq,
                'id': mean_id,
                'vq': mean_vq
            }
            trial_results.append(res)
            print(f"Target: {spd:5.1f} RPM | Measured: {mean_spd:5.1f} RPM | Err: {err:+5.1f} RPM ({err_pct:4.1f}%) | Std: {std_spd:4.2f} RPM | Iq: {mean_iq:5.3f}A | Id: {mean_id:5.3f}A | Vq: {mean_vq:5.2f}V")
        else:
            print(f"Target: {spd:5.1f} RPM | FAILED TO RECEIVE TELEMETRY")
            
    return trial_results

print("===============================================================")
print("=== GB8115 DIRECT-DRIVE BARE MOTOR 3-TRIAL DEEP VALIDATION ===")
print("===============================================================")

all_pos = []
all_spd = []

for trial in [1, 2, 3]:
    pos_res = run_position_trial(trial)
    all_pos.append(pos_res)
    time.sleep(1.0)
    
    spd_res = run_speed_trial(trial)
    all_spd.append(spd_res)
    time.sleep(1.0)

s.close()

with open("/home/du/Desktop/wheeled-humanoid/tests/bare_motor_3_trials_report.json", "w") as f:
    json.dump({'position_trials': all_pos, 'speed_trials': all_spd}, f, indent=2)

print("\nValidation complete! Results saved to tests/bare_motor_3_trials_report.json")
