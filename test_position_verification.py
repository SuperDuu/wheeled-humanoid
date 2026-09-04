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

def run_pos_move(target_deg, duration_s, wait_tail_s=2.5):
    print(f"\nCommanding MOVE {target_deg} deg (traj time: {duration_s}s)...")
    start_record()
    cmd(f"MOVE {target_deg} {duration_s}")
    time.sleep(duration_s + wait_tail_s)
    df = stop_and_get_df()
    
    if df.empty:
        print(f"ERROR: No data for MOVE {target_deg}")
        return None
    
    df.to_csv(f"/tmp/pos_move_{target_deg}.csv", index=False)
    
    t = (df['timestamp_ms'] - df['timestamp_ms'].iloc[0]) / 1000.0
    actual_deg = np.rad2deg(df['joint_angle'].values)
    err = np.abs(actual_deg - target_deg)
    
    # Rigorous settling time: earliest sample after which ALL remaining samples meet tolerance <= 0.5 deg
    bad = np.flatnonzero(err > 0.5)
    if len(bad) == 0:
        ts = 0.0
    elif bad[-1] + 1 < len(t):
        ts = float(t.iloc[bad[-1] + 1])
    else:
        ts = float('inf')
        
    # Tail steady state: last 1.0 second
    tail_mask = t >= (t.max() - 1.0)
    final_angle = float(np.mean(actual_deg[tail_mask]))
    final_error = float(final_angle - target_deg)
    final_abs_err = float(abs(final_error))
    peak_iq = float(np.max(np.abs(df['i_q'].values)))
    fault = int(df['fault_code'].max())
    
    res = {
        "target_deg": target_deg,
        "duration_s": duration_s,
        "final_deg": round(final_angle, 3),
        "error_deg": round(final_error, 3),
        "abs_error_deg": round(final_abs_err, 3),
        "settling_time_s": round(ts, 2) if ts != float('inf') else "inf",
        "peak_iq_a": round(peak_iq, 3),
        "fault": fault
    }
    print(f"  Target: {target_deg:6.1f}° -> Final: {final_angle:6.2f}°, Error: {final_error:+5.2f}° (abs: {final_abs_err:4.2f}°), Ts: {res['settling_time_s']}s, Peak Iq: {peak_iq:5.2f} A, Fault: {fault}")
    return res

results = []
try:
    print("Zeroing joint reference...")
    cmd("STOP")
    time.sleep(0.5)
    cmd("ZERO")
    time.sleep(0.5)
    
    moves = [
        (45.0, 1.2),
        (90.0, 1.5),
        (180.0, 2.0),
        (-90.0, 2.5),
        (0.0, 1.5)
    ]
    
    for deg, dur in moves:
        r = run_pos_move(deg, dur, wait_tail_s=2.0)
        if r:
            results.append(r)
        time.sleep(0.5)
        
    with open("/tmp/position_verification_results.json", "w") as f:
        json.dump(results, f, indent=2)
    print("\nPosition verification completed! Summary Table:")
    print(pd.DataFrame(results))

finally:
    try:
        cmd("STOP")
    except Exception:
        pass
