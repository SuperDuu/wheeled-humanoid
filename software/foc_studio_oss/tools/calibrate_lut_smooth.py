#!/usr/bin/env python3
"""
Smooth Continuous Open-Loop LUT Calibration with Steady-State Tracking
Only analyzes the constant-velocity region after ramp-up is complete.
"""
import sys, os, time, math
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'src'))
from telemetry_parser import TelemetryParser, PACKET_SIZE_94, MAGIC1, MAGIC2
import serial
import numpy as np

PORT = '/dev/ttyACM0'
BAUD = 115200
LUT_SIZE = 128
POLE_PAIRS = 21

def read_packets(ser, duration_sec):
    buf = bytearray()
    samples = []
    t_end = time.time() + duration_sec
    while time.time() < t_end:
        raw = ser.read(ser.in_waiting or 1)
        if raw:
            buf.extend(raw)
        while len(buf) >= PACKET_SIZE_94:
            if buf[0] == MAGIC1 and buf[1] == MAGIC2:
                p = TelemetryParser.parse_packet(bytes(buf[:PACKET_SIZE_94]))
                if p:
                    samples.append(p)
                    del buf[:PACKET_SIZE_94]
                    continue
            del buf[0]
        time.sleep(0.002)
    return samples

def send_cmd(ser, cmd):
    ser.write(f"{cmd}\r\n".encode())
    time.sleep(0.05)

def main():
    print("=" * 75)
    print("STEADY-STATE ENCODER NON-LINEARITY CALIBRATION")
    print("=" * 75)

    ser = serial.Serial(PORT, BAUD, timeout=0.1)
    time.sleep(0.2)
    ser.reset_input_buffer()

    send_cmd(ser, "CLEAR_LUT")
    send_cmd(ser, "USE_LUT 0")
    send_cmd(ser, "STOP")
    time.sleep(0.3)

    # 1. Forward sweep: Run for 7 seconds, analyze last 3.5s (exact 1 revolution at 17.14 RPM = 3.5s)
    print("\n[1] Running FORWARD open-loop sweep (7.0s)...")
    ser.reset_input_buffer()
    send_cmd(ser, "OPENLOOP 20 6.0")
    s_fwd = read_packets(ser, 7.0)
    send_cmd(ser, "STOP")
    time.sleep(0.5)

    # 2. Reverse sweep: Run for 7 seconds, analyze last 3.5s
    print("\n[2] Running REVERSE open-loop sweep (7.0s)...")
    ser.reset_input_buffer()
    send_cmd(ser, "OPENLOOP -20 6.0")
    s_rev = read_packets(ser, 7.0)
    send_cmd(ser, "STOP")
    time.sleep(0.3)

    ser.close()

    # Slice only the last 3.5s of steady-state (when speed is exactly 20 RPM)
    fwd_ss = s_fwd[len(s_fwd)//2:]
    rev_ss = s_rev[len(s_rev)//2:]

    fwd_mech = np.unwrap([s['mech_angle'] for s in fwd_ss])
    rev_mech = np.unwrap([s['mech_angle'] for s in rev_ss])

    t_fwd = np.array([s['timestamp_ms'] * 1e-3 for s in fwd_ss])
    t_rev = np.array([s['timestamp_ms'] * 1e-3 for s in rev_ss])
    t_fwd -= t_fwd[0]
    t_rev -= t_rev[0]

    # Linear fit to ideal constant-velocity ramp
    slope_f, intercept_f = np.polyfit(t_fwd, fwd_mech, 1)
    ideal_fwd = slope_f * t_fwd + intercept_f
    err_fwd = ideal_fwd - fwd_mech

    slope_r, intercept_r = np.polyfit(t_rev, rev_mech, 1)
    ideal_rev = slope_r * t_rev + intercept_r
    err_rev = ideal_rev - rev_mech

    # Resample error over angle range [0, 2*pi)
    phase_fwd = np.fmod(fwd_mech - fwd_mech[0], 2.0*math.pi)
    phase_fwd = (phase_fwd + 2.0*math.pi) % (2.0*math.pi)

    phase_rev = np.fmod(rev_mech - rev_mech[0], 2.0*math.pi)
    phase_rev = (phase_rev + 2.0*math.pi) % (2.0*math.pi)

    bins = np.linspace(0, 2.0*math.pi, LUT_SIZE, endpoint=False)
    lut_fwd = np.zeros(LUT_SIZE)
    lut_rev = np.zeros(LUT_SIZE)

    for i in range(LUT_SIZE):
        b_low = bins[i]
        b_high = bins[(i+1)%LUT_SIZE] if i < LUT_SIZE-1 else 2.0*math.pi
        mask_f = (phase_fwd >= b_low) & (phase_fwd < b_high)
        if np.any(mask_f):
            lut_fwd[i] = np.mean(err_fwd[mask_f])
        mask_r = (phase_rev >= b_low) & (phase_rev < b_high)
        if np.any(mask_r):
            lut_rev[i] = np.mean(err_rev[mask_r])

    # Average forward & reverse to cancel lag/drag
    lut_avg = (lut_fwd + lut_rev) * 0.5
    lut_avg -= np.mean(lut_avg) # Remove DC

    # Fourier filter (harmonics 1..4)
    fft_c = np.fft.rfft(lut_avg)
    fft_c[5:] = 0.0
    smooth_lut = np.fft.irfft(fft_c, n=LUT_SIZE)

    p2p_mech_deg = math.degrees(np.max(smooth_lut) - np.min(smooth_lut))
    p2p_elec_deg = p2p_mech_deg * POLE_PAIRS

    print(f"\n[3] Non-Linearity Results (Steady-State):")
    print(f"  Physical Mechanical Eccentricity: {p2p_mech_deg:.3f} deg")
    print(f"  Electrical Angle Distortion: {p2p_elec_deg:.1f} deg electrical")

    if p2p_mech_deg > 3.5:
        print(f"WARNING: Measured distortion ({p2p_mech_deg:.1f} deg) exceeds physical bounds. Not applying LUT.")
        return

    # Convert to fixed-point units (1 count = 0.0001 rad)
    lut_int = np.round(smooth_lut * 10000.0).astype(np.int16)

    # Upload to driver
    print(f"\n[4] Uploading calibrated LUT ({LUT_SIZE} points) to driver...")
    ser = serial.Serial(PORT, BAUD, timeout=0.1)
    for idx, val in enumerate(lut_int):
        send_cmd(ser, f"LUT {idx} {val}")
        time.sleep(0.005)
    send_cmd(ser, "USE_LUT 1")
    send_cmd(ser, "STOP")
    ser.close()

    print("  LUT successfully uploaded and activated!")
    print("=" * 75)

if __name__ == '__main__':
    main()
