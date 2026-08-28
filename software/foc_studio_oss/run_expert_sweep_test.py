#!/usr/bin/env python3
import time
import requests
import json
import sys

BASE_URL = "http://localhost:5050"

def check_and_connect():
    try:
        r = requests.get(f"{BASE_URL}/api/ports", timeout=2)
        ports = r.json().get("ports", [])
        print(f"[PORT SCAN] Detected ports: {ports}")
        
        status_r = requests.get(f"{BASE_URL}/api/status", timeout=2)
        connected = status_r.json().get("connected", False)
        if not connected and ports:
            target_port = ports[0]
            print(f"[CONNECT] Connecting to {target_port} @ 115200...")
            conn_r = requests.post(f"{BASE_URL}/api/connect", json={"port": target_port, "baudrate": 115200}, timeout=3)
            print(f"[CONNECT RES] {conn_r.json()}")
            time.sleep(1.0)
    except Exception as e:
        print(f"[ERR] Server check error: {e}")

def send_cmd(cmd):
    print(f"\n=======================================================")
    print(f"---> GỬI LỆNH: {cmd}")
    print(f"=======================================================")
    try:
        r = requests.post(f"{BASE_URL}/api/command", json={"command": cmd}, timeout=5)
        print(f"     Kết quả server: {r.json()}")
    except Exception as e:
        print(f"     Lỗi gửi lệnh: {e}")

def get_telemetry(duration_s=3.0, interval_s=0.15):
    t_end = time.time() + duration_s
    while time.time() < t_end:
        try:
            r = requests.get(f"{BASE_URL}/api/status", timeout=2)
            data = r.json()
            latest = data.get("latest", {})
            rpm = latest.get("speed_rpm", 0.0)
            tgt_rpm = latest.get("speed_target_rpm", 0.0)
            id_val = latest.get("i_d", 0.0)
            iq_val = latest.get("i_q", 0.0)
            tgt_iq = latest.get("i_q_target", 0.0)
            vd = latest.get("vd", 0.0)
            vq = latest.get("vq", 0.0)
            vbus = latest.get("vbus", 24.0)
            theta_e = latest.get("phase_elec", 0.0)
            theta_0 = latest.get("zero_elec_angle", 0.0)
            dir_val = latest.get("encoder_dir", 1)
            mode = latest.get("control_mode", 0)
            state = latest.get("motor_state", 0)
            duty_a = latest.get("duty_a", 0.0)
            duty_b = latest.get("duty_b", 0.0)
            duty_c = latest.get("duty_c", 0.0)
            
            print(f"[{time.strftime('%H:%M:%S')}] Telemetry | Mode={mode} State={state} | Id={id_val:+.2f}A, Iq={iq_val:+.2f}A (Tgt={tgt_iq:+.2f}A) | RPM={rpm:+.1f}/{tgt_rpm:+.1f} | Vbus={vbus:.1f}V | Vd={vd:+.2f}V Vq={vq:+.2f}V θe={theta_e:+.2f} θ0={theta_0:+.2f} dir={dir_val} | duty={duty_a:.3f}/{duty_b:.3f}/{duty_c:.3f}")
        except Exception as e:
            print(f"     Read Error: {e}")
        time.sleep(interval_s)

def main():
    print("=== BẮT ĐẦU CHẠY THỬ NGHIỆM ĐỘNG HỌC FOC TỰ ĐỘNG ===")
    check_and_connect()
    
    # 1. STOP & Settling
    send_cmd("STOP")
    get_telemetry(duration_s=1.0)
    
    # 2. ALIGN
    send_cmd("ALIGN")
    print("     Đang chờ ALIGN hoàn tất (2.0s)...")
    time.sleep(2.0)
    get_telemetry(duration_s=1.0)
    
    # 3. BƯỚC 1: Quét điện áp cố định Fixed-Voltage (VOLT 5.0 -> 6.0 -> 6.3 -> 6.5)
    send_cmd("VOLT 5.0")
    get_telemetry(duration_s=3.0)
    
    send_cmd("VOLT 6.0")
    get_telemetry(duration_s=3.0)
    
    send_cmd("VOLT 6.3")
    get_telemetry(duration_s=3.0)
    
    send_cmd("VOLT 6.5")
    get_telemetry(duration_s=3.5)
    
    # 4. BƯỚC 2: Kiểm chứng vòng kín SPEED 200
    send_cmd("SPEED 200")
    get_telemetry(duration_s=6.0)
    
    # 5. Dừng an toàn
    send_cmd("STOP")
    get_telemetry(duration_s=1.0)
    print("\n=== HOÀN TẤT BÀI TEST CHẨN ĐOÁN TỰ ĐỘNG ===")

if __name__ == "__main__":
    main()
