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

def run_move(target_deg, duration_s, wait_tail_s=2.0):
    start_record()
    cmd(f"MOVE {target_deg} {duration_s}")
    time.sleep(duration_s + wait_tail_s)
    df = stop_and_get_df()
    if df.empty:
        return None
    
    t = (df['timestamp_ms'] - df['timestamp_ms'].iloc[0]) / 1000.0
    actual_deg = np.rad2deg(df['joint_angle'].values)
    err = np.abs(actual_deg - target_deg)
    
    # Rigorous settling time
    bad = np.flatnonzero(err > 0.5)
    if len(bad) == 0:
        ts = 0.0
    elif bad[-1] + 1 < len(t):
        ts = float(t.iloc[bad[-1] + 1])
    else:
        ts = float('inf')
        
    tail = actual_deg[t >= (t.max() - 1.0)]
    final_deg = float(np.mean(tail))
    final_err = float(final_deg - target_deg)
    peak_iq = float(np.max(np.abs(df['i_q'].values)))
    fault = int(df['fault_code'].max())
    
    return {
        "target": target_deg,
        "final": round(final_deg, 2),
        "error": round(final_err, 2),
        "abs_err": round(abs(final_err), 2),
        "ts": round(ts, 2) if ts != float('inf') else "inf",
        "peak_iq": round(peak_iq, 2),
        "fault": fault
    }

moves = [
    (45.0, 1.2),
    (90.0, 1.5),
    (180.0, 2.0),
    (-90.0, 4.0),
    (0.0, 2.0)
]

all_trials = []
print("Starting 3 consecutive independent position repeatability trials...")

try:
    cmd("STOP")
    time.sleep(0.5)
    cmd("ZERO")
    time.sleep(0.5)
    
    for trial_idx in range(1, 4):
        print(f"\n================ TRIAL {trial_idx} / 3 ================")
        trial_data = {"trial": trial_idx, "moves": {}}
        for deg, dur in moves:
            res = run_move(deg, dur)
            trial_data["moves"][str(deg)] = res
            print(f"  Target {deg:6.1f}° -> Final: {res['final']:6.2f}°, Err: {res['error']:+5.2f}°, Ts: {res['ts']}s, Peak Iq: {res['peak_iq']:4.2f} A, Fault: {res['fault']}")
            time.sleep(0.5)
        all_trials.append(trial_data)

    with open("/tmp/trials_3_repeatability_position.json", "w") as f:
        json.dump(all_trials, f, indent=2)
    print("\nAll 3 trials completed! Summary saved to /tmp/trials_3_repeatability_position.json")

finally:
    try:
        cmd("STOP")
    except Exception:
        pass
