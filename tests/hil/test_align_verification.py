import time
import requests
import json
import pandas as pd
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

try:
    print("Stopping motor first...")
    cmd("STOP")
    time.sleep(0.5)

    print("Starting recording and sending ALIGN...")
    start_record()
    res = cmd("ALIGN")
    print("ALIGN command sent:", res)

    # ALIGN duration is ~14s
    print("Waiting for ALIGN to complete (16 seconds)...")
    time.sleep(16)

    df = stop_and_get_df()
    print(f"Recorded {len(df)} samples during ALIGN.")
    if not df.empty:
        print("Columns:", list(df.columns))
        last = df.iloc[-1]
        print(f"  timestamp_ms: {last.get('timestamp_ms')}")
        print(f"  fault_code: {last.get('fault_code')}")
        print(f"  zero_elec_angle: {last.get('zero_elec_angle')}")
        print(f"  mech_angle: {last.get('mech_angle')}")
        print(f"  motor_state: {last.get('motor_state')}")
        print(f"  i_q: {last.get('i_q')}")
        print(f"  v_bus: {last.get('v_bus')}")
        
        if 'mech_angle' in df.columns:
            print(f"  mech_angle range: min={df['mech_angle'].min():.4f}, max={df['mech_angle'].max():.4f}")
        
        df.to_csv("/tmp/align_run_raw.csv", index=False)
        print("Saved to /tmp/align_run_raw.csv")

finally:
    try:
        cmd("STOP")
        print("Motor STOP issued.")
    except Exception as e:
        print("STOP error:", e)
