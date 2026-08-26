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

def get_telemetry(duration_s=35.0, interval_s=0.5):
    t_end = time.time() + duration_s
    t_start = time.time()
    
    # Accumulators for window statistics (every 3 seconds)
    win_rpm = []
    win_id = []
    win_iq = []
    win_vd = []
    win_vq = []
    
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
            fet_temp = latest.get("fet_temp", 25.0)
            
            elapsed = time.time() - t_start
            win_rpm.append(rpm)
            win_id.append(id_val)
            win_iq.append(iq_val)
            win_vd.append(vd)
            win_vq.append(vq)
            
            # Print sample
            print(f"[{elapsed:4.1f}s] Telemetry | RPM={rpm:+5.1f} | Vq={vq:4.2f}V, Vd={vd:+5.2f}V | Id={id_val:+4.2f}A, Iq={iq_val:+4.2f}A | T_fet={fet_temp:.1f}C")
            
        except Exception as e:
            print(f"     Read Error: {e}")
        time.sleep(interval_s)
        
    if win_rpm:
        avg_rpm = sum(win_rpm) / len(win_rpm)
        avg_id = sum(win_id) / len(win_id)
        avg_iq = sum(win_iq) / len(win_iq)
        print(f"\n---> [THỐNG KÊ TOÀN GIAI ĐOẠN {duration_s:.0f}s]: RPM_avg={avg_rpm:+.1f} (min={min(win_rpm):+.1f}, max={max(win_rpm):+.1f}), Id_avg={avg_id:+.3f}A, Iq_avg={avg_iq:+.3f}A")

def main():
    print("=== BẮT ĐẦU BÀI TEST GIỮ ÁP DÀI HẠN (LONG-DURATION FIXED-VOLTAGE TEST) ===")
    
    # 1. ALIGN
    send_cmd("STOP")
    time.sleep(1.0)
    send_cmd("ALIGN")
    print("     Đang đợi ALIGN...")
    time.sleep(2.0)
    
    # 2. Test VOLT 5.0 giữ 35 giây liên tục
    send_cmd("VOLT 5.0")
    get_telemetry(duration_s=35.0, interval_s=0.5)
    
    send_cmd("STOP")
    time.sleep(2.0)
    
    # 3. Test VOLT 6.0 giữ 35 giây liên tục
    send_cmd("VOLT 6.0")
    get_telemetry(duration_s=35.0, interval_s=0.5)
    
    send_cmd("STOP")
    time.sleep(2.0)
    
    # 4. Test VOLT 6.25 giữ 35 giây liên tục
    send_cmd("VOLT 6.25")
    get_telemetry(duration_s=35.0, interval_s=0.5)
    
    send_cmd("STOP")
    time.sleep(1.0)
    print("\n=== HOÀN TẤT BÀI TEST GIỮ ÁP DÀI HẠN ===")

if __name__ == "__main__":
    main()
