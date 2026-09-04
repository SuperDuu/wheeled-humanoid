#!/usr/bin/env python3
import time
import requests
import json

BASE_URL = "http://localhost:5050"

def send_cmd(cmd):
    print(f"\n---> Sending Command: {cmd}")
    try:
        r = requests.post(f"{BASE_URL}/api/command", json={"command": cmd}, timeout=5)
        print(f"     Response: {r.json()}")
    except Exception as e:
        print(f"     Error: {e}")

def get_telemetry(duration_s=2.0, interval_s=0.2):
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
            theta_e = latest.get("phase_elec", 0.0)
            theta_0 = latest.get("zero_elec_angle", 0.0)
            dir_val = latest.get("encoder_dir", 1)
            mode = latest.get("control_mode", 0)
            state = latest.get("motor_state", 0)
            print(f"[{time.strftime('%H:%M:%S')}] Mode={mode} State={state} | Id={id_val:+.2f}A, Iq={iq_val:+.2f}A (Tgt={tgt_iq:+.2f}A) | RPM={rpm:+.1f}/{tgt_rpm:+.1f} | Vd={vd:+.1f}V Vq={vq:+.1f}V θe={theta_e:+.2f} θ0={theta_0:+.2f} dir={dir_val}")
        except Exception as e:
            print(f"     Read Error: {e}")
        time.sleep(interval_s)

def main():
    print("=== BẮT ĐẦU QUY TRÌNH AUTO DEBUG FOC ===")
    
    # 1. Dừng an toàn
    send_cmd("STOP")
    get_telemetry(duration_s=1.0)
    
    # 2. ALIGN
    send_cmd("ALIGN")
    print("     Đang đợi ALIGN 2 chiều hoàn tất (3 giây)...")
    time.sleep(3.0)
    get_telemetry(duration_s=1.5)
    
    # 3. Test IQ +0.3A (Quay thuận)
    send_cmd("IQ 0.3")
    get_telemetry(duration_s=2.0)
    
    # 4. Test IQ -0.3A (Quay nghịch)
    send_cmd("IQ -0.3")
    get_telemetry(duration_s=2.0)
    
    send_cmd("STOP")
    get_telemetry(duration_s=1.0)
    
    # 5. Test SPEED 20
    send_cmd("SPEED 20")
    get_telemetry(duration_s=3.0)
    
    # 6. Test SPEED 50
    send_cmd("SPEED 50")
    get_telemetry(duration_s=3.0)
    
    # 7. Test SPEED 100
    send_cmd("SPEED 100")
    get_telemetry(duration_s=3.0)
    
    # 8. Dừng
    send_cmd("STOP")
    get_telemetry(duration_s=1.0)
    print("\n=== HOÀN TẤT QUY TRÌNH AUTO DEBUG FOC ===")

if __name__ == "__main__":
    main()
