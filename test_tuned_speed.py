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

def run_test(target_rpm, duration=8.0, kp=0.0005, sfilt=0.25, ki=0.0025):
    print(f"\nTesting target: {target_rpm} RPM with KP={kp}, SFILT={sfilt}, KI={ki} (duration: {duration}s)")
    cmd("STOP")
    time.sleep(0.5)
    cmd(f"KP_S {kp}")
    cmd(f"SFILT {sfilt}")
    cmd(f"KI_S {ki}")
    time.sleep(0.2)
    
    start_record()
    cmd(f"SPEED {target_rpm}")
    time.sleep(duration)
    cmd("STOP")
    time.sleep(0.5)
    
    df = stop_and_get_df()
    if df.empty:
        print("ERROR: empty recording!")
        return None
    
    df.to_csv(f"/tmp/tuned_speed_{target_rpm}.csv", index=False)
    
    t = (df['timestamp_ms'] - df['timestamp_ms'].iloc[0]) / 1000.0
    # True steady-state window: last 4 seconds
    steady = df[t >= (t.max() - 4.0)]
    
    s = steady['speed_rpm'].values
    iq = steady['i_q'].values
    mean_speed = float(np.mean(s))
    std_speed = float(np.std(s))
    err = float(mean_speed - target_rpm)
    mean_iq = float(np.mean(iq))
    std_iq = float(np.std(iq))
    fault = int(df['fault_code'].max())
    
    print(f"Result for {target_rpm} RPM:")
    print(f"  Steady Mean Speed: {mean_speed:.2f} RPM (Error: {err:+.2f} RPM)")
    print(f"  Speed Ripple Std:  {std_speed:.2f} RPM")
    print(f"  Steady Mean Iq:    {mean_iq:.3f} A (Std Iq: {std_iq:.3f} A)")
    print(f"  Max Fault Code:    {fault}")
    return {
        "target": target_rpm,
        "mean_speed": mean_speed,
        "std_speed": std_speed,
        "error": err,
        "mean_iq": mean_iq,
        "std_iq": std_iq,
        "fault": fault
    }

results = []
try:
    for target in [50, 100, -100]:
        res = run_test(target, duration=8.0, kp=0.0005, sfilt=0.25, ki=0.0025)
        if res:
            results.append(res)
        time.sleep(1.0)
        
    with open("/tmp/tuned_speed_summary.json", "w") as f:
        json.dump(results, f, indent=2)
    print("\nSummary Table:")
    print(pd.DataFrame(results))
finally:
    try:
        cmd("STOP")
    except Exception:
        pass
