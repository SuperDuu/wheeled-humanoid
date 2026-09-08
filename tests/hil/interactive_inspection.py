#!/usr/bin/env python3
"""
Interactive Hardware-in-the-Loop Inspection Tool for GB8115
Allows human operator (Du) to physically test:
  1. Position holding, dither check, stiffness & hand-resist
  2. Low speed smooth rotation & hand drag resistance
  3. High speed stability & sound/vibration check
"""

import sys
import time
import struct
import math
import serial

SERIAL_PORT = "/dev/ttyACM0"
BAUD_RATE = 115200

PKT_FMT = '<BBBBI 16f 3Bb 4f Bb H'
PKT_SIZE = struct.calcsize(PKT_FMT)

def open_serial():
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.05)
    time.sleep(0.1)
    ser.reset_input_buffer()
    return ser

def read_telemetry(ser):
    buf = bytearray()
    t_end = time.time() + 0.1
    while time.time() < t_end:
        c = ser.read(ser.in_waiting or 64)
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
            del buf[:PKT_SIZE]
            try:
                u = struct.unpack(PKT_FMT, pkt)
                return {
                    't_ms': u[4],
                    'iq': u[9],
                    'id': u[8],
                    'joint_deg': math.degrees(u[16]),
                    'spd_rpm': u[17],
                    'vbus': u[19],
                    'temp_fet': u[20],
                    'fault': u[23]
                }
            except Exception:
                pass
    return None

def test_hold_position(angle_deg=0.0, duration_s=15):
    ser = open_serial()
    print(f"\n>>> [KÍCH HOẠT GIỮ VỊ TRÍ {angle_deg:.1f}° TRONG {duration_s} GIÂY] <<<")
    ser.write(b"STOP\r\n")
    time.sleep(0.1)
    ser.write(b"BARE\r\n")
    time.sleep(0.05)
    ser.write(b"DIR 1\r\n")
    time.sleep(0.05)
    ser.write(f"POS {angle_deg:.1f}\r\n".encode())
    
    t_start = time.time()
    t_last_print = 0
    
    try:
        while time.time() - t_start < duration_s:
            p = read_telemetry(ser)
            now = time.time()
            if p and (now - t_last_print >= 0.25):
                t_last_print = now
                err = p['joint_deg'] - angle_deg
                rem = duration_s - (now - t_start)
                print(f"  [Còn {rem:4.1f}s] Góc: {p['joint_deg']:6.2f}° (Lệch: {err:+5.2f}°) | Phản lực Iq: {p['iq']:+6.3f}A | Tốc độ: {p['spd_rpm']:+5.1f}RPM | Temp FET: {p['temp_fet']:.1f}°C")
            time.sleep(0.02)
    finally:
        ser.write(b"STOP\r\n")
        ser.close()
        print(">>> [ĐÃ DỪNG AN TOÀN - CUỘN DÂY NGẮT DÒNG] <<<\n")

def test_run_speed(speed_rpm=50.0, duration_s=10):
    ser = open_serial()
    print(f"\n>>> [KÍCH HOẠT QUAY {speed_rpm:+.1f} RPM TRONG {duration_s} GIÂY] <<<")
    ser.write(b"STOP\r\n")
    time.sleep(0.1)
    ser.write(b"BARE\r\n")
    time.sleep(0.05)
    ser.write(b"DIR 1\r\n")
    time.sleep(0.05)
    ser.write(f"SPEED {speed_rpm:.1f}\r\n".encode())
    
    t_start = time.time()
    t_last_print = 0
    
    try:
        while time.time() - t_start < duration_s:
            p = read_telemetry(ser)
            now = time.time()
            if p and (now - t_last_print >= 0.25):
                t_last_print = now
                rem = duration_s - (now - t_start)
                err = p['spd_rpm'] - speed_rpm
                print(f"  [Còn {rem:4.1f}s] Tốc độ: {p['spd_rpm']:+6.1f} RPM (Lệch: {err:+5.1f}) | Dòng Iq: {p['iq']:+6.3f}A | Vbus: {p['vbus']:.1f}V | Temp: {p['temp_fet']:.1f}°C")
            time.sleep(0.02)
    finally:
        ser.write(b"STOP\r\n")
        ser.close()
        print(">>> [ĐÃ DỪNG AN TOÀN - CUỘN DÂY NGẮT DÒNG] <<<\n")

def test_speed_chain(speeds=[100.0, 200.0], dur_per_speed=8.0):
    ser = open_serial()
    try:
        ser.write(b"STOP\r\n")
        time.sleep(0.1)
        ser.write(b"BARE\r\n")
        time.sleep(0.05)
        ser.write(b"DIR 1\r\n")
        time.sleep(0.05)
        
        for spd in speeds:
            print(f"\n>>> [BƯỚC CHUYỂN TỐC: KÍCH HOẠT {spd:+.1f} RPM TRONG {dur_per_speed} GIÂY] <<<")
            ser.write(f"SPEED {spd:.1f}\r\n".encode())
            t_start = time.time()
            t_last_print = 0
            while time.time() - t_start < dur_per_speed:
                p = read_telemetry(ser)
                now = time.time()
                if p and (now - t_last_print >= 0.25):
                    t_last_print = now
                    rem = dur_per_speed - (now - t_start)
                    err = p['spd_rpm'] - spd
                    print(f"  [{spd:+.0f} RPM | Còn {rem:4.1f}s] Tốc độ: {p['spd_rpm']:+6.1f} RPM (Lệch: {err:+5.1f}) | Dòng Iq: {p['iq']:+6.3f}A | Vbus: {p['vbus']:.1f}V | Temp: {p['temp_fet']:.1f}°C")
                time.sleep(0.02)
            time.sleep(0.2)
    finally:
        ser.write(b"STOP\r\n")
        ser.close()
        print("\n>>> [ĐÃ DỪNG AN TOÀN - CUỘN DÂY NGẮT DÒNG] <<<\n")

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage: interactive_inspection.py hold [deg] [duration_s] | speed [rpm] [duration_s] | chain")
        sys.exit(1)
    cmd = sys.argv[1]
    if cmd == 'hold':
        deg = float(sys.argv[2]) if len(sys.argv) > 2 else 0.0
        dur = float(sys.argv[3]) if len(sys.argv) > 3 else 15.0
        test_hold_position(deg, dur)
    elif cmd == 'speed':
        rpm = float(sys.argv[2]) if len(sys.argv) > 2 else 50.0
        dur = float(sys.argv[3]) if len(sys.argv) > 3 else 10.0
        test_run_speed(rpm, dur)
    elif cmd == 'chain':
        test_speed_chain([100.0, 200.0], 8.0)

