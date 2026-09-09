#!/usr/bin/env python3
"""
HIL Verification Script for MIT Cheetah Real-Time Impedance & Torque Control
GB8115 BLDC Joint Driver (STM32G473)
"""

import sys
import time
import struct
import math
import serial

PKT_FMT = '<BBBBI 16f 3Bb 4f Bb H'
PKT_SIZE = struct.calcsize(PKT_FMT)

PORT = '/dev/ttyACM0'
BAUD = 115200

def read_telemetry_pkt(ser, timeout=0.1):
    t0 = time.time()
    buf = bytearray()
    while time.time() - t0 < timeout:
        c = ser.read(64)
        if c:
            buf.extend(c)
        while len(buf) >= PKT_SIZE:
            idx = buf.find(b'\xaa\x55')
            if idx < 0:
                buf.clear()
                break
            if idx > 0:
                del buf[:idx]
            if len(buf) < PKT_SIZE:
                break
            pkt = bytes(buf[:PKT_SIZE])
            calc_csum = sum(pkt[4:-2]) & 0xFFFF
            pkt_csum = struct.unpack('<H', pkt[-2:])[0]
            if calc_csum != pkt_csum:
                del buf[:2]
                continue
            del buf[:PKT_SIZE]
            u = struct.unpack(PKT_FMT, pkt)
            # u[8]=id, u[9]=iq, u[15]=mech_deg, u[17]=spd_rpm, u[19]=vbus, u[20]=temp, u[21]=mode
            return {
                'id': u[8],
                'iq': u[9],
                'angle_deg': u[15],
                'speed_rpm': u[17],
                'vbus': u[19],
                'temp': u[20],
                'mode': u[21],
                'fault': u[23]
            }
    return None

def test_zero_torque(ser, duration=15.0):
    print("=" * 65)
    print("TEST 1: TRANSPARENT BACKDRIVABILITY (ZERO-TORQUE MODE)")
    print("=" * 65)
    print("Commanding: MIT 0 0 0 0 0 (Zero impedance, Zero torque)")
    print("Hand action: Hãy dùng tay xoay trục motor tự do!")
    print("Mục tiêu: Động cơ quay nhẹ như không có điện, dòng Iq xấp xỉ 0.0A.")
    print("-" * 65)
    ser.write(b'BARE\r\n')
    time.sleep(0.05)
    ser.write(b'MIT 0 0 0 0 0\r\n')
    t0 = time.time()
    while time.time() - t0 < duration:
        pkt = read_telemetry_pkt(ser, timeout=0.08)
        if pkt:
            tau_est = pkt['iq'] * 0.6615
            rem = duration - (time.time() - t0)
            print(f"\r[{rem:4.1f}s] Góc: {pkt['angle_deg']:6.1f}° | Tốc độ: {pkt['speed_rpm']:6.1f} RPM | Dòng Iq: {pkt['iq']:6.3f}A | Mô-men: {tau_est:6.3f} Nm", end="", flush=True)
        time.sleep(0.05)
    print("\n\nKết thúc Test 1. Ngắt dòng an toàn...")
    ser.write(b'STOP\r\n')

def test_virtual_spring(ser, kp=5.0, kd=0.15, duration=15.0):
    print("=" * 65)
    print(f"TEST 2: VIRTUAL SPRING IMPEDANCE (Kp = {kp:.1f} Nm/rad, Kd = {kd:.2f} Nm/(rad/s))")
    print("=" * 65)
    print(f"Lệnh: MIT 0 0 {kp:.1f} {kd:.2f} 0 (Điểm cân bằng ảo tại 0.0°)")
    print("Hand action: Dùng tay bẻ lệch trục khỏi vị trí 0° để cảm nhận độ đàn hồi!")
    print("Mục tiêu: Cảm nhận lực lò xo kéo về 0°, buông tay tự về êm ru không rung.")
    print("-" * 65)
    ser.write(b'BARE\r\n')
    time.sleep(0.05)
    ser.write(b'SETHOME\r\n')
    time.sleep(0.05)
    cmd = f"MIT 0.0 0.0 {kp:.2f} {kd:.2f} 0.0\r\n"
    ser.write(cmd.encode('ascii'))
    t0 = time.time()
    while time.time() - t0 < duration:
        pkt = read_telemetry_pkt(ser, timeout=0.08)
        if pkt:
            tau_est = pkt['iq'] * 0.6615
            rem = duration - (time.time() - t0)
            print(f"\r[{rem:4.1f}s] Lệch góc: {pkt['angle_deg']:6.1f}° | Tốc độ: {pkt['speed_rpm']:6.1f} RPM | Dòng Iq: {pkt['iq']:6.3f}A | Phản lực: {tau_est:6.3f} Nm", end="", flush=True)
        time.sleep(0.05)
    print("\n\nKết thúc Test 2. Ngắt dòng an toàn...")
    ser.write(b'STOP\r\n')

def test_pure_torque(ser, tau=1.0, duration=10.0):
    print("=" * 65)
    print(f"TEST 3: PURE FEEDFORWARD TORQUE (Tau_ff = {tau:.2f} Nm)")
    print("=" * 65)
    print(f"Lệnh: TORQUE {tau:.2f} (Tương đương lực cản ~{tau/0.025:.1f} N trên mép vỏ)")
    print("Hand action: Hãy dùng tay giữ ghì trục để cảm nhận lực kéo liên tục!")
    print("Mục tiêu: Động cơ sinh lực kéo vững chắc không đổi, không trễ tích phân.")
    print("-" * 65)
    ser.write(b'BARE\r\n')
    time.sleep(0.05)
    cmd = f"TORQUE {tau:.2f}\r\n"
    ser.write(cmd.encode('ascii'))
    t0 = time.time()
    while time.time() - t0 < duration:
        pkt = read_telemetry_pkt(ser, timeout=0.08)
        if pkt:
            tau_est = pkt['iq'] * 0.6615
            rem = duration - (time.time() - t0)
            print(f"\r[{rem:4.1f}s] Tốc độ: {pkt['speed_rpm']:6.1f} RPM | Dòng Iq: {pkt['iq']:6.3f}A | Mô-men thực: {tau_est:6.3f} Nm | Vbus: {pkt['vbus']:5.2f}V", end="", flush=True)
        time.sleep(0.05)
    print("\n\nKết thúc Test 3. Ngắt dòng an toàn...")
    ser.write(b'STOP\r\n')

def main():
    if len(sys.argv) < 2:
        print("Cách dùng:")
        print("  python3 tests/hil/test_mit_impedance.py zero [duration_s]")
        print("  python3 tests/hil/test_mit_impedance.py spring [kp] [kd] [duration_s]")
        print("  python3 tests/hil/test_mit_impedance.py torque [tau_Nm] [duration_s]")
        sys.exit(1)

    mode = sys.argv[1].lower()
    ser = serial.Serial(PORT, BAUD, timeout=0.2)
    ser.reset_input_buffer()
    ser.write(b'STOP\r\n')
    time.sleep(0.1)

    try:
        if mode == 'zero':
            dur = float(sys.argv[2]) if len(sys.argv) > 2 else 15.0
            test_zero_torque(ser, dur)
        elif mode == 'spring':
            kp = float(sys.argv[2]) if len(sys.argv) > 2 else 5.0
            kd = float(sys.argv[3]) if len(sys.argv) > 3 else 0.15
            dur = float(sys.argv[4]) if len(sys.argv) > 4 else 15.0
            test_virtual_spring(ser, kp, kd, dur)
        elif mode == 'torque':
            tau = float(sys.argv[2]) if len(sys.argv) > 2 else 1.0
            dur = float(sys.argv[3]) if len(sys.argv) > 3 else 10.0
            test_pure_torque(ser, tau, dur)
        else:
            print(f"Chế độ không hợp lệ: {mode}")
    finally:
        ser.write(b'STOP\r\n')
        ser.close()

if __name__ == '__main__':
    main()
