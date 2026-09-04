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
    print("Stopping motor...")
    cmd("STOP")
    time.sleep(0.5)

    print("Starting recording and sending ALIGN...")
    start_record()
    res = cmd("ALIGN")
    print("ALIGN response:", res)

    t0 = time.time()
    last_state = 1
    last_zero = 0.0
    while time.time() - t0 < 25.0:
        time.sleep(0.5)
        # Check current status by getting latest sample or checking status
        elapsed = time.time() - t0
        print(f"  elapsed: {elapsed:.1f}s...")
        # Check if completed after at least 15s
        if elapsed > 16.0:
            # Let it run a bit more to be completely sure
            if elapsed > 20.0:
                break

    print("Waiting finished. Stopping recording and fetching DF...")
    df = stop_and_get_df()
    print(f"Recorded {len(df)} samples during ALIGN.")
    if not df.empty:
        last = df.iloc[-1]
        print("Last row:")
        print(f"  timestamp_ms: {last.get('timestamp_ms')}")
        print(f"  fault_code: {last.get('fault_code')}")
        print(f"  zero_elec_angle: {last.get('zero_elec_angle')}")
        print(f"  mech_angle: {last.get('mech_angle')}")
        print(f"  motor_state: {last.get('motor_state')}")
        print(f"  i_q: {last.get('i_q')}")
        print(f"  zero_elec_angle range in DF: {df['zero_elec_angle'].unique()}")
        
        # Check when zero_elec_angle changed
        non_zero = df[df['zero_elec_angle'] != 0.0]
        if not non_zero.empty:
            print(f"ALIGN SUCCESS! First non-zero offset recorded at t={non_zero['time_sec'].iloc[0]}s: {non_zero['zero_elec_angle'].iloc[0]:.4f} rad")
            print(f"Final calibrated offset: {non_zero['zero_elec_angle'].iloc[-1]:.4f} rad")
        else:
            print("WARNING: zero_elec_angle remained 0.0 in all samples!")
        
        df.to_csv("/tmp/align_run_full.csv", index=False)
        print("Saved to /tmp/align_run_full.csv")

finally:
    try:
        cmd("STOP")
        print("Motor STOP confirmed.")
    except Exception as e:
        print("STOP error:", e)
