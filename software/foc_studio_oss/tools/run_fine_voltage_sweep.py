#!/usr/bin/env python3
import time
import requests
import json

BASE_URL = "http://localhost:5050"

def send_cmd(cmd):
    print(f"\n=======================================================")
    print(f"---> GỬI LỆNH: {cmd}")
    print(f"=======================================================")
    try:
        r = requests.post(f"{BASE_URL}/api/command", json={"command": cmd}, timeout=5)
        print(f"     Kết quả server: {r.json()}")
    except Exception as e:
        print(f"     Lỗi gửi lệnh: {e}")

def get_telemetry(duration_s=3.5, interval_s=0.15):
    t_end = time.time() + duration_s
    while time.time() < t_end:
        try:
            r = requests.get(f"{BASE_URL}/api/status", timeout=2)
            data = r.json()
            latest = data.get("latest", {})
            rpm = latest.get("speed_rpm", 0.0)
            id_val = latest.get("i_d", 0.0)
            iq_val = latest.get("i_q", 0.0)
            vd = latest.get("vd", 0.0)
            vq = latest.get("vq", 0.0)
            vbus = latest.get("vbus", 24.0)
            theta_e = latest.get("phase_elec", 0.0)
            
            print(f"[{time.strftime('%H:%M:%S')}] Telemetry | RPM={rpm:+.1f} | Vbus={vbus:.1f}V | Vd={vd:+.2f}V Vq={vq:+.2f}V | Id={id_val:+.2f}A, Iq={iq_val:+.2f}A | θe={theta_e:+.2f}")
        except Exception as e:
            print(f"     Read Error: {e}")
        time.sleep(interval_s)

def main():
    print("=== BẮT ĐẦU QUY TRÌNH QUÉT TINH ĐIỆN ÁP 6.20V -> 6.50V ===")
    
    # 1. ALIGN
    send_cmd("ALIGN")
    time.sleep(2.0)
    
    # 2. Quét tinh các mức điện áp
    voltages = [6.20, 6.30, 6.35, 6.40, 6.45, 6.50]
    for v in voltages:
        send_cmd(f"VOLT {v:.2f}")
        get_telemetry(duration_s=3.5)
        
    send_cmd("STOP")
    get_telemetry(duration_s=1.0)
    print("\n=== HOÀN TẤT QUY TRÌNH QUÉT TINH ĐIỆN ÁP ===")

if __name__ == "__main__":
    main()
