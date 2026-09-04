import time
import requests
import json
import pandas as pd
import numpy as np
import io

BASE_URL = "http://127.0.0.1:1111"

def cmd(c):
    r = requests.post(f"{BASE_URL}/api/command", json={"command": c}, timeout=5)
    r.raise_for_status()
    return r.json()

def start_record():
    requests.post(f"{BASE_URL}/api/record/start", timeout=5)

def stop_and_get_df():
    requests.post(f"{BASE_URL}/api/record/stop", timeout=5)
    r = requests.get(f"{BASE_URL}/api/record/export", timeout=10)
    if r.status_code == 200 and len(r.content) > 0:
        return pd.read_csv(io.StringIO(r.text), comment='#')
    return pd.DataFrame()

def measure_speed_steady(target_rpm, run_duration=6.5, steady_t_start=2.0, steady_t_end=6.0):
    start_record()
    cmd(f"SPEED {target_rpm}")
    time.sleep(run_duration)
    # Stop recording BEFORE issuing STOP command so deceleration doesn't pollute the steady state!
    requests.post(f"{BASE_URL}/api/record/stop", timeout=5)
    cmd("STOP")
    time.sleep(0.5)
    
    r = requests.get(f"{BASE_URL}/api/record/export", timeout=10)
    df = pd.read_csv(io.StringIO(r.text), comment='#')
    if df.empty:
        return None
    
    t = (df['timestamp_ms'] - df['timestamp_ms'].iloc[0]) / 1000.0
    steady = df[(t >= steady_t_start) & (t <= steady_t_end)]
    
    s = steady['speed_rpm'].values
    iq = steady['i_q'].values
    
    mean_speed = float(np.mean(s))
    std_speed = float(np.std(s))
    err = float(mean_speed - target_rpm)
    mean_iq = float(np.mean(iq))
    max_iq = float(np.max(np.abs(iq)))
    fault = int(df['fault_code'].max())
    
    return {
        "target": target_rpm,
        "mean_speed": round(mean_speed, 2),
        "std_speed": round(std_speed, 2),
        "error": round(err, 2),
        "mean_iq": round(mean_iq, 3),
        "max_iq": round(max_iq, 3),
        "fault": fault,
        "samples": len(steady)
    }

all_trials = []
print("Starting 3 consecutive independent repeatability trials...")

try:
    cmd("STOP")
    time.sleep(1.0)
    
    for trial_idx in range(1, 4):
        print(f"\n================ TRIAL {trial_idx} / 3 ================")
        trial_data = {"trial": trial_idx, "runs": {}}
        for target in [50, 100, -100]:
            print(f"Trial {trial_idx}: Testing {target} RPM...")
            res = measure_speed_steady(target)
            trial_data["runs"][str(target)] = res
            print(f"  Target {target:4d} RPM -> Mean: {res['mean_speed']:6.2f} RPM, Err: {res['error']:+5.2f} RPM, Std: {res['std_speed']:5.2f} RPM, Iq: {res['mean_iq']:+5.3f} A, Fault: {res['fault']}")
            time.sleep(1.0)
        all_trials.append(trial_data)

    with open("/tmp/trials_3_repeatability_speed.json", "w") as f:
        json.dump(all_trials, f, indent=2)
    print("\nAll 3 trials completed! Summary saved to /tmp/trials_3_repeatability_speed.json")

finally:
    try:
        cmd("STOP")
    except Exception:
        pass
