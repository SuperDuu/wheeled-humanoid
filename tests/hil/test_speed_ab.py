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

def run_trial(name, kp, sfilt, ki, target_rpm=50.0, duration=6.0):
    print(f"\n==========================================")
    print(f"Running Configuration {name}: KP={kp}, SFILT={sfilt}, KI={ki}")
    print(f"==========================================")
    
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
    
    df.to_csv(f"/tmp/ab_{name}.csv", index=False)
    
    t = (df['timestamp_ms'] - df['timestamp_ms'].iloc[0]) / 1000.0
    steady = df[t >= 2.0]
    
    mean_speed = float(steady['speed_rpm'].mean())
    std_speed = float(steady['speed_rpm'].std())
    error = float(mean_speed - target_rpm)
    mean_iq = float(steady['i_q'].mean())
    peak_iq = float(df['i_q'].abs().max())
    
    # FFT peak
    s = steady['speed_rpm'].values
    s_detrend = s - np.mean(s)
    fft_s = np.fft.rfft(s_detrend * np.hanning(len(s_detrend)))
    freqs = np.fft.rfftfreq(len(s_detrend), 0.01)
    amps = np.abs(fft_s)
    peak_idx = np.argmax(amps[freqs > 0.5])
    peak_freq = float(freqs[freqs > 0.5][peak_idx])
    peak_amp = float(amps[freqs > 0.5][peak_idx])
    
    res = {
        "name": name,
        "kp": kp,
        "sfilt": sfilt,
        "ki": ki,
        "mean_speed": mean_speed,
        "std_speed": std_speed,
        "error": error,
        "mean_iq": mean_iq,
        "peak_iq": peak_iq,
        "peak_freq_hz": peak_freq,
        "peak_amp": peak_amp
    }
    print(f"Result for {name}:")
    print(f"  Mean Speed: {mean_speed:.2f} RPM (error: {error:+.2f} RPM)")
    print(f"  Speed Ripple Std: {std_speed:.2f} RPM")
    print(f"  FFT Peak: {peak_freq:.2f} Hz (amp: {peak_amp:.1f})")
    print(f"  Steady Iq: {mean_iq:.3f} A, Peak |Iq|: {peak_iq:.3f} A")
    return res

configs = [
    ("A_Baseline", 0.0015, 0.08, 0.002),
    ("B1_MidGain", 0.0006, 0.16, 0.001),
    ("B2_LowGain_HighBW", 0.0004, 0.25, 0.001),
    ("B3_Soft_HighBW", 0.0003, 0.35, 0.001),
]

all_results = []
try:
    for cfg in configs:
        r = run_trial(*cfg)
        if r:
            all_results.append(r)
        time.sleep(1.0)
        
    with open("/tmp/ab_comparison_results.json", "w") as f:
        json.dump(all_results, f, indent=2)
    print("\n\nAll A/B runs completed. Summary:")
    print(pd.DataFrame(all_results)[['name', 'kp', 'sfilt', 'ki', 'mean_speed', 'std_speed', 'error', 'peak_freq_hz', 'peak_amp']])
finally:
    try:
        cmd("STOP")
    except Exception:
        pass
