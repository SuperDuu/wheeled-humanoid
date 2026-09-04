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
        print(f"     Kết quả: {r.json()}")
    except Exception as e:
        print(f"     Lỗi gửi: {e}")

def main():
    print("=== TEST TỰ ĐỘNG HIỆU CHUẨN BEN KATZ CALIB (1 VÒNG) ===")
    
    # 1. Gửi lệnh CALIB
    send_cmd("STOP")
    time.sleep(1.0)
    
    send_cmd("CALIB")
    print("     Đang thực hiện quét hiệu chuẩn 128 điểm (1 vòng tới, 1 vòng lui, ~7 giây)...")
    for i in range(8):
        time.sleep(1.0)
        try:
            r = requests.get(f"{BASE_URL}/api/status", timeout=2)
            data = r.json()
            latest = data.get("latest", {})
            print(f"     [{i+1}s/8s] State={latest.get('state')} | θe={latest.get('angle_rad', 0):.2f} | θ0={latest.get('encoder_zero_angle', 0):.2f} | RPM={latest.get('speed_rpm', 0):.1f}")
        except:
            pass

    print("\n---> HIỆU CHUẨN 128 ĐIỂM HOÀN TẤT! BẮT ĐẦU TEST SPEED 150 <---")
    send_cmd("SPEED 150")
    
    t_end = time.time() + 25.0
    t_start = time.time()
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
            elapsed = time.time() - t_start
            print(f"[{elapsed:4.1f}s] Telemetry | RPM={rpm:+5.1f} | Vq={vq:4.2f}V, Vd={vd:+5.2f}V | Id={id_val:+4.2f}A, Iq={iq_val:+4.2f}A")
        except Exception as e:
            print(f"     Read Error: {e}")
        time.sleep(0.5)
        
    send_cmd("STOP")
    print("\n=== HOÀN TẤT TEST ===")

if __name__ == "__main__":
    main()
