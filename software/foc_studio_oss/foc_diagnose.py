#!/usr/bin/env python3
"""
⚡ STM32 FOC Closed-Loop Automated Diagnostics & Diagnostic Reporter
Wheeled Humanoid Robotics Project - Joint Driver 8115

Usage:
  1. Passive Monitor Mode (Capture whatever runs on STM32 / CubeIDE):
     python3 foc_diagnose.py

  2. Automated 4-Step Test Routine (ALIGN -> VQ -> IQ -> STOP):
     python3 foc_diagnose.py --auto-test

Outputs:
  - Concise Markdown Diagnostic Report: foc_diagnostic_report.md (Workspace Root)
  - Concise JSON Diagnostic Summary:    foc_diagnostic_report.json
"""

import sys
import os
import glob
import time
import math
import struct
import argparse
from typing import Optional, Dict, Any, List
import numpy as np

try:
    import serial
    import serial.tools.list_ports
    HAS_SERIAL = True
except ImportError:
    HAS_SERIAL = False

# Import Telemetry Parser
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(SCRIPT_DIR, "src"))
from telemetry_parser import TelemetryParser, PACKET_SIZE_94, PACKET_SIZE_78, MAGIC1, MAGIC2

WORKSPACE_ROOT = "/home/du/Desktop/wheeled-humanoid"
REPORT_MD_PATH = os.path.join(WORKSPACE_ROOT, "foc_diagnostic_report.md")
REPORT_JSON_PATH = os.path.join(WORKSPACE_ROOT, "foc_diagnostic_report.json")


def find_serial_port(preferred_port: Optional[str] = None) -> Optional[str]:
    """Find STM32 USB CDC Virtual COM port."""
    if preferred_port and os.path.exists(preferred_port):
        return preferred_port

    if HAS_SERIAL:
        for p in serial.tools.list_ports.comports():
            if "STM32" in (p.description or "") or "Virtual COM" in (p.description or "") or "ttyACM" in p.device:
                return p.device

    candidates = glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*")
    if candidates:
        return sorted(candidates)[0]
    return None


class FOCDiagnosticEngine:
    def __init__(self, port: str, baudrate: int = 115200):
        self.port_name = port
        self.baudrate = baudrate
        self.ser: Optional[serial.Serial] = None
        self.samples: List[Dict[str, Any]] = []
        self.max_samples = 2000

    def connect(self) -> bool:
        if not HAS_SERIAL:
            print("❌ PySerial is not installed.")
            return False
        try:
            self.ser = serial.Serial(self.port_name, self.baudrate, timeout=0.1)
            print(f" Connected to {self.port_name} at {self.baudrate} baud.")
            return True
        except Exception as e:
            print(f"❌ Failed to connect to {self.port_name}: {e}")
            return False

    def close(self):
        if self.ser and self.ser.is_open:
            self.ser.close()

    def send_cmd(self, cmd_str: str):
        if not self.ser or not self.ser.is_open:
            return
        msg = (cmd_str.strip() + "\r\n").encode("utf-8")
        self.ser.write(msg)
        self.ser.flush()

    def read_packets(self, duration_sec: float = 1.0) -> List[Dict[str, Any]]:
        """Collect telemetry packets over duration."""
        if not self.ser or not self.ser.is_open:
            return []

        buffer = bytearray()
        collected = []
        t_end = time.time() + duration_sec

        while time.time() < t_end:
            waiting = self.ser.in_waiting
            if waiting > 0:
                raw = self.ser.read(waiting)
                if raw:
                    buffer.extend(raw)

            while len(buffer) >= PACKET_SIZE_78:
                if buffer[0] == MAGIC1 and buffer[1] == MAGIC2:
                    pkt_size = PACKET_SIZE_94 if len(buffer) >= PACKET_SIZE_94 else PACKET_SIZE_78
                    candidate = bytes(buffer[:pkt_size])
                    parsed = TelemetryParser.parse_packet(candidate)
                    if parsed:
                        del buffer[:pkt_size]
                        collected.append(parsed)
                        if len(self.samples) < self.max_samples:
                            self.samples.append(parsed)
                        continue
                del buffer[0]

            if len(buffer) > 4096:
                buffer.clear()
            time.sleep(0.005)

        return collected

    def run_auto_diagnostics(self) -> Dict[str, Any]:
        """Execute 4-Step Diagnostic Routine."""
        print("\n" + "="*60)
        print("⚡ BẮT ĐẦU CHẨN ĐOÁN TỰ ĐỘNG FOC CLOSED-LOOP (4 BƯỚC)")
        print("="*60)

        # 0. Flush & Read Initial State
        print("\n[Bước 0] Đọc trạng thái tĩnh (Rest State & Baseline)...")
        self.send_cmd("STOP")
        time.sleep(0.3)
        baseline = self.read_packets(duration_sec=1.0)

        # 1. Trigger Alignment
        print("\n[Bước 1] Kích hoạt Căn chỉnh Encoder (ALIGN)...")
        self.send_cmd("ALIGN")
        align_samples = self.read_packets(duration_sec=7.5)
        print(f"  -> Thu thập {len(align_samples)} mẫu telemetry trong lúc Align.")

        # 2. Test Voltage Mode Closed-Loop (VQ 6.0V - Bỏ qua PI Dòng, đủ lực kéo hộp số 1:17)
        print("\n[Bước 2] Test Voltage-Mode Closed-Loop (VQ 6.0V - Bỏ qua PI Dòng)...")
        self.send_cmd("VQ 6.0")
        vq_samples = self.read_packets(duration_sec=3.0)
        print(f"  -> Thu thập {len(vq_samples)} mẫu telemetry ở chế độ VQ.")

        # 3. Test Current Mode (IQ 1.0A - Kích hoạt PI Dòng)
        print("\n[Bước 3] Test Current-Mode Closed-Loop (IQ 1.0A - Kích hoạt PI Dòng)...")
        self.send_cmd("IQ 1.0")
        iq_samples = self.read_packets(duration_sec=3.0)
        print(f"  -> Thu thập {len(iq_samples)} mẫu telemetry ở chế độ IQ.")

        # 4. Safe Stop
        print("\n[Bước 4] Dừng an toàn (STOP)...")
        self.send_cmd("STOP")
        self.read_packets(duration_sec=0.5)

        return self.analyze_all_data(baseline, align_samples, vq_samples, iq_samples)

    def analyze_all_data(self, baseline: List[Dict], align: List[Dict], vq: List[Dict], iq: List[Dict]) -> Dict[str, Any]:
        """Analyze gathered telemetry across all test stages."""
        results = {
            "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
            "port": self.port_name,
            "vbus_volts": 24.0,
            "encoder_dir": -1,
            "zero_elec_angle_rad": 0.0,
            "zero_elec_angle_deg": 0.0,
            "current_offset_ia": 0.0,
            "current_offset_ib": 0.0,
            "kirchhoff_sum_error_amps": 0.0,
            "vq_mode_success": False,
            "vq_avg_speed_rpm": 0.0,
            "vq_avg_iq_measured": 0.0,
            "iq_mode_success": False,
            "iq_tracking_error_amps": 0.0,
            "fault_occurred": False,
            "fault_code": 0,
            "diagnosis_verdict": "UNKNOWN",
            "detected_issues": [],
            "action_items": []
        }

        all_pkts = baseline + align + vq + iq
        if not all_pkts:
            results["diagnosis_verdict"] = "FAIL_NO_DATA"
            results["detected_issues"].append("Không nhận được dữ liệu Telemetry nào qua USB CDC.")
            results["action_items"].append("Kiểm tra cáp USB kết nối chân USB_DP/USB_DM, cấp nguồn VBUS và flash firmware.")
            return results

        # 1. Bus Voltage Check
        vbus_vals = [p["v_bus"] for p in all_pkts if p["v_bus"] > 1.0]
        if vbus_vals:
            results["vbus_volts"] = round(float(np.median(vbus_vals)), 2)
            if results["vbus_volts"] < 12.0:
                results["detected_issues"].append(f"Điện áp VBUS đo được thấp ({results['vbus_volts']}V < 12.0V). Động cơ có thể bị ngắt do UVP.")
                results["action_items"].append("Kiểm tra nguồn cấp 24V hoặc kiểm tra hệ số VBUS_DIVIDER_RATIO trong main.c.")

        # 2. Alignment & Encoder Angle Check
        if align:
            last_align = align[-1]
            results["encoder_dir"] = last_align.get("encoder_dir", -1)
            results["zero_elec_angle_rad"] = last_align.get("zero_elec_angle", 0.0)
            results["zero_elec_angle_deg"] = round(math.degrees(results["zero_elec_angle_rad"]), 2)

        # 3. Voltage Mode Closed Loop Analysis (VQ)
        if vq:
            vq_speeds = [p["speed_rpm"] for p in vq]
            vq_iqs = [p["i_q"] for p in vq]
            faults = [p["fault_code"] for p in vq if p["fault_code"] != 0]

            results["vq_avg_speed_rpm"] = round(float(np.mean(vq_speeds)), 2)
            results["vq_avg_iq_measured"] = round(float(np.mean(vq_iqs)), 3)

            if len(faults) > 0:
                results["fault_occurred"] = True
                results["fault_code"] = faults[-1]
                results["detected_issues"].append(f"Gặp lỗi bảo vệ (Fault code {faults[-1]}) ngay trong chế độ Voltage-Mode VQ!")
                results["action_items"].append("Góc Encoder hoặc Thứ tự pha chưa đúng. Chạy lại ALIGN hoặc kiểm tra lại thứ tự pha U-V-W.")
            elif abs(results["vq_avg_speed_rpm"]) > 5.0:
                results["vq_mode_success"] = True
            else:
                results["detected_issues"].append(f"Ở chế độ VQ (Voltage-Mode), tốc độ động cơ gần như bằng 0 ({results['vq_avg_speed_rpm']} RPM).")
                results["action_items"].append("Kiểm tra xem động cơ có bị kẹt cơ khí hộp số 1:17 hay Zero Offset bị lệch 90 độ (ép áp vào trục D thay vì trục Q).")

        # 4. Current Mode Closed Loop Analysis (IQ)
        if iq:
            iq_measured = [p["i_q"] for p in iq]
            iq_targets = [p["i_q_target"] for p in iq]
            faults_iq = [p["fault_code"] for p in iq if p["fault_code"] != 0]

            if len(faults_iq) > 0:
                results["fault_occurred"] = True
                results["fault_code"] = faults_iq[-1]
                results["detected_issues"].append(f"Ngắt quá dòng (Fault {faults_iq[-1]}) khi bật Current-Mode IQ.")
                results["action_items"].append("Dấu cảm biến dòng (Current Sensing) bị NGƯỢC hoặc hệ số Kp/Ki quá lớn. Đổi dấu âm/dương trong main.c:L1643.")
            else:
                avg_meas = float(np.mean(iq_measured))
                avg_tgt = float(np.mean(iq_targets))
                results["iq_tracking_error_amps"] = round(abs(avg_tgt - avg_meas), 3)

                # Check current polarity
                if avg_tgt > 0.1 and avg_meas < -0.1:
                    results["detected_issues"].append("Dòng điện đo về Iq BỊ NGƯỢC DẤU so với dòng mục tiêu (Target > 0 nhưng Measured < 0).")
                    results["action_items"].append("Đổi dấu dòng trong main.c (dòng 1643-1644): Đổi `current_b = -((float)raw_ib ...)` thành `+`.")
                elif results["iq_tracking_error_amps"] < 0.5:
                    results["iq_mode_success"] = True

        # 5. Overall Diagnosis Verdict
        if results["vq_mode_success"] and results["iq_mode_success"]:
            results["diagnosis_verdict"] = "PASS_ALL_OK"
        elif results["vq_mode_success"] and not results["iq_mode_success"]:
            results["diagnosis_verdict"] = "FAIL_CURRENT_SENSE_OR_PI"
        elif not results["vq_mode_success"]:
            results["diagnosis_verdict"] = "FAIL_ENCODER_OR_PHASE"
        else:
            results["diagnosis_verdict"] = "FAIL_UNSTABLE"

        return results

    def generate_report(self, diag: Dict[str, Any]):
        """Write concise markdown & json report to workspace root."""
        # Write JSON
        import json
        with open(REPORT_JSON_PATH, "w", encoding="utf-8") as f:
            json.dump(diag, f, indent=2, ensure_ascii=False)

        # Write Markdown
        verdict_badge = {
            "PASS_ALL_OK": "🟢 **PASS (HOÀN HẢO - Closed Loop Hoạt Động Tốt)**",
            "FAIL_CURRENT_SENSE_OR_PI": "🔴 **FAIL TẠI VÒNG DÒNG (Góc Encoder Đúng, Lỗi Mạch Đo Dòng hoặc PI)**",
            "FAIL_ENCODER_OR_PHASE": "🔴 **FAIL TẠI GÓC ENCODER (Sai Chiều Quay, Offset hoặc Thứ Tự Pha)**",
            "FAIL_NO_DATA": "⚠️ **FAIL (Không Nhận Được Dữ Liệu USB Telemetry)**",
            "FAIL_UNSTABLE": "🔴 **FAIL (Hệ Thống Mất Ổn Định / Quá Dòng)**"
        }.get(diag["diagnosis_verdict"], "⚠️ **CHƯA RÕ**")

        md_content = f"""# 📊 Báo Cáo Chẩn Đoán FOC Closed-Loop (Joint Driver 8115)
*Thời gian tạo:* `{diag.get('timestamp', 'N/A')}` | *Cổng kết nối:* `{diag.get('port', 'N/A')}`

---

### 1. Kết Luận Nhanh (Executive Verdict)
{verdict_badge}

---

### 2. Bảng Thông Số Đo Đạc Thực Tế (Telemetry Snapshot)

| Thông số | Giá trị đo được | Đánh giá / Trạng thái |
| :--- | :--- | :--- |
| **Điện áp Bus (VBUS)** | `{diag.get('vbus_volts', 0)} V` | {'✅ Bình thường (≥12V)' if diag.get('vbus_volts', 0) >= 12.0 else '❌ Quá thấp (UVP)'} |
| **Chiều quay Encoder (`encoder_dir`)** | `{diag.get('encoder_dir', -1)}` | {'✅ Hợp lệ' if diag.get('encoder_dir') in [1, -1] else '❌ Lỗi'} |
| **Zero Electrical Offset ($\theta_{{offset}}$)** | `{diag.get('zero_elec_angle_rad', 0)} rad` (`{diag.get('zero_elec_angle_deg', 0)}°`) | {'✅ Đã căn chỉnh' if diag.get('zero_elec_angle_rad') != 0.0 else '⚠️ Chưa Align'} |
| **Test Voltage-Mode (`VQ 1.5V`)** | `{diag.get('vq_avg_speed_rpm', 0)} RPM` | {'✅ Quay êm (Encoder & Pha OK)' if diag.get('vq_mode_success') else '❌ Không quay / Giật khục'} |
| **Test Current-Mode (`IQ 0.4A`)** | Sai số bám `{diag.get('iq_tracking_error_amps', 0)} A` | {'✅ Bám dòng mượt mà' if diag.get('iq_mode_success') else '❌ Lỗi đo dòng / Quá dòng'} |
| **Lỗi phần cứng (Fault Code)** | `{diag.get('fault_code', 0)}` | {'✅ Không có lỗi' if diag.get('fault_code', 0) == 0 else '❌ Kích hoạt bảo vệ'} |

---

### 3. Vấn Đề Được Phát Hiện (Detected Issues)
"""
        if diag.get("detected_issues"):
            for issue in diag["detected_issues"]:
                md_content += f"- ⚠️ **{issue}**\n"
        else:
            md_content += "- ✅ Không phát hiện bất thường nào.\n"

        md_content += "\n---\n\n### 4. Hướng Dẫn Khắc Phục Chính Xác (Action Items)\n"
        if diag.get("action_items"):
            for i, action in enumerate(diag["action_items"], 1):
                md_content += f"{i}. **{action}**\n"
        else:
            md_content += "- Hệ thống đã hoạt động hoàn hảo. Có thể tăng dần tải hoặc chuyển sang điều khiển Position/Speed PID.\n"

        with open(REPORT_MD_PATH, "w", encoding="utf-8") as f:
            f.write(md_content)

        print(f"\n📄 Đã xuất báo cáo chẩn đoán tóm tắt tại:")
        print(f"   -> {REPORT_MD_PATH}")
        print(f"   -> {REPORT_JSON_PATH}")


def main():
    parser = argparse.ArgumentParser(description="FOC Telemetry Diagnostic Tool")
    parser.add_argument("--port", type=str, default=None, help="Serial port (/dev/ttyACM0)")
    parser.add_argument("--auto-test", action="store_true", help="Run automated 4-step test routine")
    parser.add_argument("--duration", type=float, default=5.0, help="Passive monitoring duration in seconds")
    args = parser.parse_args()

    port = find_serial_port(args.port)
    if not port:
        print("❌ Không tìm thấy cổng USB STM32 (/dev/ttyACM* hoặc /dev/ttyUSB*).")
        print("   Vui lòng cắm cáp USB bo mạch vào máy tính hoặc chỉ định --port.")
        # Create empty failure report for agent
        diag = {
            "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
            "port": "None",
            "diagnosis_verdict": "FAIL_NO_DATA",
            "detected_issues": ["Không tìm thấy thiết bị USB Virtual COM port kết nối vào máy tính."],
            "action_items": ["Cắm cáp USB và kiểm tra quyền truy cập cổng: `sudo chmod 666 /dev/ttyACM*`."]
        }
        engine = FOCDiagnosticEngine(port="None")
        engine.generate_report(diag)
        sys.exit(1)

    engine = FOCDiagnosticEngine(port=port)
    if not engine.connect():
        sys.exit(1)

    try:
        if args.auto_test:
            diag = engine.run_auto_diagnostics()
        else:
            print(f" Đang lắng nghe Telemetry thụ động từ STM32 trong {args.duration}s...")
            print("   (Nếu bạn vừa ấn Run / Debug trên CubeIDE, dữ liệu sẽ được ghi nhận ngay lập tức)")
            packets = engine.read_packets(duration_sec=args.duration)
            print(f" -> Thu thập được {len(packets)} mẫu.")
            diag = engine.analyze_all_data(baseline=[], align=[], vq=packets, iq=[])

        engine.generate_report(diag)
    finally:
        engine.close()


if __name__ == "__main__":
    main()
