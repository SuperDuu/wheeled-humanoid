#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
===============================================================================
Automated FOC Parameter Grid Search & Optimization Tuner
===============================================================================
Automatically sweeps candidate ranges of Flux Linkage (λ), Speed Kp, and Speed Ki
to empirically find the globally optimal, ultra-stable parameter set for GB8115.
"""

import sys
import time
import math
import glob
import struct
import serial

PACKET_FORMAT = "<BBBB I 16f BBBb 4f H"
PACKET_SIZE = struct.calcsize(PACKET_FORMAT)

def find_serial_port():
    ports = glob.glob('/dev/ttyACM*') + glob.glob('/dev/ttyUSB*')
    if not ports:
        print("❌ Không tìm thấy cổng COM kết nối STM32!")
        sys.exit(1)
    return ports[0]

def read_telemetry_packet(ser, buffer):
    while ser.in_waiting > 0:
        chunk = ser.read(ser.in_waiting)
        if chunk:
            buffer.extend(chunk)
            
    while len(buffer) >= PACKET_SIZE:
        if buffer[0] == 0xAA and buffer[1] == 0x55 and buffer[2] == 0x01 and buffer[3] == (PACKET_SIZE - 4):
            packet_bytes = buffer[:PACKET_SIZE]
            try:
                unpacked = struct.unpack(PACKET_FORMAT, packet_bytes)
                (magic1, magic2, pkt_type, payload_len, ts_ms,
                 ia, ib, ic, id_c, iq_c, iq_tgt,
                 da, db, dc, phase, mech, joint,
                 speed, speed_tgt, vbus, temp,
                 mode, state, fault, enc_dir,
                 vd, vq, zero_elec, id_tgt,
                 chk_val) = unpacked

                computed_chk = sum(packet_bytes[4:-2]) & 0xFFFF
                if computed_chk == chk_val:
                    del buffer[:PACKET_SIZE]
                    return {
                        'speed_rpm': speed,
                        'speed_target_rpm': speed_tgt,
                        'vq': vq,
                        'vd': vd,
                        'id': id_c,
                        'iq': iq_c,
                        'state': state
                    }
            except Exception:
                pass
        buffer.pop(0)
    return None

def test_parameter_candidate(ser, flux_lambda, kp, ki, test_rpm=200.0, duration_s=4.0):
    # Set parameters on the fly
    ser.write(f"FLUX {flux_lambda:.5f}\r\n".encode('utf-8'))
    ser.flush()
    time.sleep(0.05)
    ser.write(f"KP_S {kp:.5f}\r\n".encode('utf-8'))
    ser.flush()
    time.sleep(0.05)
    ser.write(f"KI_S {ki:.5f}\r\n".encode('utf-8'))
    ser.flush()
    time.sleep(0.05)
    
    # Step target speed
    ser.write(f"SPEED {test_rpm:.1f}\r\n".encode('utf-8'))
    ser.flush()
    
    buffer = bytearray()
    records = []
    t_start = time.time()
    while time.time() - t_start < duration_s:
        pkt = read_telemetry_packet(ser, buffer)
        if pkt:
            pkt['t'] = time.time() - t_start
            records.append(pkt)
        time.sleep(0.005)
        
    # Stop motor
    ser.write(b"SPEED 0\r\n")
    ser.flush()
    time.sleep(0.6)
    
    if len(records) < 15:
        return None
        
    # Evaluate performance on steady state (last 2.0s)
    steady_records = [r for r in records if r['t'] >= 1.8]
    if not steady_records:
        steady_records = records[-10:]
        
    rpms = [r['speed_rpm'] for r in steady_records]
    vqs = [r['vq'] for r in steady_records]
    
    mean_rpm = sum(rpms) / len(rpms)
    std_rpm = math.sqrt(sum((x - mean_rpm)**2 for x in rpms) / len(rpms))
    rmse = math.sqrt(sum((x - test_rpm)**2 for x in rpms) / len(rpms))
    final_rpm = rpms[-1]
    
    # Stall penalty if motor halted
    stall_penalty = 1000.0 if (final_rpm < 80.0 or mean_rpm < 100.0) else 0.0
    
    # Total Cost Function J (Lower is Better)
    cost = rmse + 2.0 * std_rpm + abs(mean_rpm - test_rpm) * 1.5 + stall_penalty
    
    return {
        'flux': flux_lambda,
        'kp': kp,
        'ki': ki,
        'mean_rpm': mean_rpm,
        'std_rpm': std_rpm,
        'rmse': rmse,
        'final_rpm': final_rpm,
        'mean_vq': sum(vqs) / len(vqs),
        'cost': cost,
        'stalled': stall_penalty > 0
    }

def main():
    port = find_serial_port()
    print(f"🔌 Kết nối tới STM32 trên cổng: {port} @ 115200 bps...")
    ser = serial.Serial(port, 115200, timeout=0.1)
    time.sleep(0.5)
    
    print("\n" + "="*75)
    print("🚀 BẮT ĐẦU TỰ ĐỘNG QUÉT DẢI THÔNG SỐ FOC (GRID SEARCH TUNER)")
    print("="*75)
    
    # 1. Alignment first
    print("\n[Bước 1/2] Đang căn chỉnh điểm 0 điện (ALIGN Encoder ~7.0s)...")
    ser.write(b"ALIGN\r\n")
    ser.flush()
    t_align = time.time()
    while time.time() - t_align < 7.2:
        if ser.in_waiting > 0:
            ser.read(ser.in_waiting)
        time.sleep(0.02)
    print("  -> Alignment hoàn tất! Bắt đầu quét thông số...\n")
    
    # Candidate parameter ranges
    flux_candidates = [0.0145, 0.0155, 0.0165]
    kp_candidates   = [0.00100, 0.00150, 0.00200]
    ki_candidates   = [0.00000, 0.00005, 0.00010]
    
    total_runs = len(flux_candidates) * len(kp_candidates) * len(ki_candidates)
    run_idx = 0
    
    results = []
    
    print(f"{'No':<4} | {'Flux (λ)':<9} | {'Kp':<8} | {'Ki':<8} | {'RPM Trung Bình':<14} | {'Std Dev':<8} | {'Vq (V)':<7} | {'Cost J':<8} | {'Status'}")
    print("-" * 88)
    
    for flux in flux_candidates:
        for kp in kp_candidates:
            for ki in ki_candidates:
                run_idx += 1
                res = test_parameter_candidate(ser, flux, kp, ki, test_rpm=200.0, duration_s=3.5)
                if res:
                    results.append(res)
                    status = "❌ STALL" if res['stalled'] else ("✨ PERFECT" if res['cost'] < 10.0 else "✅ OK")
                    print(f"{run_idx:<4} | {res['flux']:<9.5f} | {res['kp']:<8.5f} | {res['ki']:<8.5f} | {res['mean_rpm']:<14.1f} | {res['std_rpm']:<8.2f} | {res['mean_vq']:<7.1f} | {res['cost']:<8.2f} | {status}")
                else:
                    print(f"{run_idx:<4} | {flux:<9.5f} | {kp:<8.5f} | {ki:<8.5f} | {'NO DATA':<14} | {'-':<8} | {'-':<7} | {'-':<8} | ❌ ERR")
                time.sleep(0.3)
                
    ser.close()
    
    if not results:
        print("❌ Không thu được kết quả đo!")
        return
        
    # Sort by Cost J
    results.sort(key=lambda x: x['cost'])
    best = results[0]
    
    print("\n" + "="*75)
    print("🏆 BỘ THÔNG SỐ TỐI ƯU TOÀN CỤC TỐT NHẤT ĐƯỢC TÌM THẤY:")
    print("="*75)
    print(f"  ⭐ Flux Linkage (λ) : {best['flux']:.5f} Wb")
    print(f"  ⭐ Speed Kp         : {best['kp']:.5f} V/ERPM")
    print(f"  ⭐ Speed Ki         : {best['ki']:.5f} V/(ERPM*s)")
    print(f"  📊 Tốc độ xác lập   : {best['mean_rpm']:.1f} RPM (Độ lệch chuẩn: ±{best['std_rpm']:.2f} RPM)")
    print(f"  ⚡ Điện áp Vq chuẩn : {best['mean_vq']:.2f} V")
    print(f"  🎯 Chỉ số Cost J    : {best['cost']:.2f}")
    print("="*75 + "\n")

if __name__ == '__main__':
    main()
