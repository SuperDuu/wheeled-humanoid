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

def run_speed_test(target_rpm, duration_s=6.0, discard_s=2.0):
    print(f"\n--- Testing SPEED {target_rpm} RPM (duration: {duration_s}s) ---")
    start_record()
    cmd(f"SPEED {target_rpm}")
    time.sleep(duration_s)
    cmd("STOP")
    time.sleep(0.5)
    df = stop_and_get_df()
    
    if df.empty:
        print(f"ERROR: No data recorded for SPEED {target_rpm}")
        return None
    
    # Save raw CSV
    df.to_csv(f"/tmp/speed_run_{target_rpm}.csv", index=False)
    
    t = (df['timestamp_ms'] - df['timestamp_ms'].iloc[0]) / 1000.0
    # Keep steady-state window (after discard_s)
    steady_mask = t >= discard_s
    if not steady_mask.any():
        steady_mask = t >= (t.max() * 0.5)
    
    df_steady = df[steady_mask]
    
    mean_speed = float(df_steady['speed_rpm'].mean())
    std_speed = float(df_steady['speed_rpm'].std())
    speed_error = float(mean_speed - target_rpm)
    abs_speed_error = float(abs(speed_error))
    mean_iq = float(df_steady['i_q'].mean())
    max_abs_iq = float(df['i_q'].abs().max())
    faults = int(df['fault_code'].max())
    
    # Also calculate derived speed from unwrapped mech_angle to verify encoder differentiation
    unwrapped_angle = np.unwrap(df_steady['mech_angle'])
    dt_arr = np.diff(t[steady_mask])
    if len(dt_arr) > 0 and np.mean(dt_arr) > 0:
        d_angle = np.diff(unwrapped_angle)
        derived_rpm = (d_angle / dt_arr) * (60.0 / (2.0 * np.pi))
        derived_mean = float(np.mean(derived_rpm))
        derived_std = float(np.std(derived_rpm))
    else:
        derived_mean = mean_speed
        derived_std = std_speed

    res = {
        "target_rpm": target_rpm,
        "mean_speed_telem": mean_speed,
        "std_speed_telem": std_speed,
        "mean_speed_derived": derived_mean,
        "std_speed_derived": derived_std,
        "error_rpm": speed_error,
        "abs_error_rpm": abs_speed_error,
        "mean_iq_a": mean_iq,
        "max_abs_iq_a": max_abs_iq,
        "max_fault_code": faults,
        "sample_count": len(df),
        "steady_sample_count": len(df_steady)
    }
    print(f"Result for {target_rpm} RPM:")
    print(f"  Mean speed: {mean_speed:.2f} RPM (derived: {derived_mean:.2f} RPM)")
    print(f"  Speed error: {speed_error:+.2f} RPM (abs: {abs_speed_error:.2f} RPM)")
    print(f"  Speed ripple std: {std_speed:.2f} RPM")
    print(f"  Steady Iq: {mean_iq:.3f} A, Peak |Iq|: {max_abs_iq:.3f} A")
    print(f"  Fault code: {faults}")
    return res

results = {}
try:
    cmd("STOP")
    time.sleep(1.0)
    
    # Run speed targets
    for target in [50, 100, -100]:
        res = run_speed_test(target, duration_s=7.0, discard_s=2.5)
        results[f"speed_{target}"] = res
        time.sleep(1.0)

    with open("/tmp/speed_suite_results.json", "w") as f:
        json.dump(results, f, indent=2)
    print("\nAll speed tests finished! Results saved to /tmp/speed_suite_results.json")

finally:
    try:
        cmd("STOP")
    except Exception as e:
        print("Final STOP error:", e)
