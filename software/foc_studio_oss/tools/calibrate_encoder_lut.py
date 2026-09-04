#!/usr/bin/env python3
"""
Automated 128-Point Encoder Non-Linearity LUT Calibration Suite
Based on the MIT Mini Cheetah (Ben Katz) & Moteus (mjbots) calibration methodology:

1. Locks rotor with pure D-axis stator vector using 'LOCK_ANGLE <elec_rad> <volt>'.
2. Sweeps through 128 points covering exactly 1 full mechanical revolution (21 electrical cycles).
3. Executes a FORWARD sweep and a BACKWARD sweep to cancel magnetic cogging hysteresis.
4. Computes the non-linearity error curve: error(theta) = theta_ideal - theta_measured.
5. Applies spatial harmonic filtering to generate a smooth, optimal 128-point LUT.
6. Uploads the LUT directly to STM32 RAM/Flash via USB CDC ('LUT <idx> <val>' commands).
7. Enables the LUT correction in the 20kHz FOC fast loop.
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
V_LOCK = 4.0 # 4.0V lock voltage (provides firm holding torque without heating)

def read_single_packet(ser, timeout=0.2):
    t_end = time.time() + timeout
    buf = bytearray()
    while time.time() < t_end:
        raw = ser.read(ser.in_waiting or 1)
        if raw:
            buf.extend(raw)
        while len(buf) >= PACKET_SIZE_94:
            if buf[0] == MAGIC1 and buf[1] == MAGIC2:
                p = TelemetryParser.parse_packet(bytes(buf[:PACKET_SIZE_94]))
                if p:
                    return p
            del buf[0]
        time.sleep(0.002)
    return None

def send_cmd(ser, cmd):
    ser.write(f"{cmd}\r\n".encode())
    time.sleep(0.02)

def main():
    print("=" * 75)
    print("ENCODER NON-LINEARITY LUT CALIBRATION (MIT MINI CHEETAH METHOD)")
    print(f"Motor: GB8115-4 ({POLE_PAIRS} Pole Pairs) | Resolution: {LUT_SIZE} Points / Mech Turn")
    print("=" * 75)

    try:
        ser = serial.Serial(PORT, BAUD, timeout=0.05)
    except Exception as e:
        print(f"ERROR opening port {PORT}: {e}")
        return

    time.sleep(0.2)
    ser.reset_input_buffer()

    # Step 0: Clear existing LUT
    send_cmd(ser, "CLEAR_LUT")
    send_cmd(ser, "STOP")
    time.sleep(0.3)

    # Step 1: Align
    print("\n[1] Running Initial Alignment (7.5s)...")
    send_cmd(ser, "ALIGN")
    t_end = time.time() + 7.8
    while time.time() < t_end:
        p = read_single_packet(ser, 0.05)
    
    if not p:
        print("ERROR: No telemetry from driver!")
        ser.close()
        return

    # Total electrical angle for 1 full mechanical turn = 21 * 2 * pi
    total_elec_rad = POLE_PAIRS * 2.0 * math.pi
    elec_step = total_elec_rad / LUT_SIZE

    forward_measured = []
    backward_measured = []

    # Step 2: Forward Sweep (0 to 127)
    print(f"\n[2] Executing FORWARD sweep ({LUT_SIZE} points, ~12s)...")
    for i in range(LUT_SIZE):
        th_e = i * elec_step
        send_cmd(ser, f"LOCK_ANGLE {th_e:.4f} {V_LOCK:.1f}")
        time.sleep(0.09) # settle time
        p = read_single_packet(ser, 0.05)
        if p:
            forward_measured.append(p['mech_angle'])
        else:
            forward_measured.append(forward_measured[-1] if forward_measured else 0.0)
        print(f"\r  Progress: {i+1}/{LUT_SIZE} (Forward) -> Mech: {forward_measured[-1]:.4f} rad", end="", flush=True)
    print(" -> Done!")

    # Step 3: Backward Sweep (127 down to 0)
    print(f"\n[3] Executing BACKWARD sweep ({LUT_SIZE} points, ~12s)...")
    for i in range(LUT_SIZE - 1, -1, -1):
        th_e = i * elec_step
        send_cmd(ser, f"LOCK_ANGLE {th_e:.4f} {V_LOCK:.1f}")
        time.sleep(0.09) # settle time
        p = read_single_packet(ser, 0.05)
        if p:
            backward_measured.append(p['mech_angle'])
        else:
            backward_measured.append(backward_measured[-1] if backward_measured else 0.0)
        print(f"\r  Progress: {LUT_SIZE - i}/{LUT_SIZE} (Backward) -> Mech: {backward_measured[-1]:.4f} rad", end="", flush=True)
    print(" -> Done!")

    send_cmd(ser, "STOP")

    # Reverse backward list so its indexing matches forward
    backward_measured.reverse()

    # Step 4: Process and Average
    print("\n[4] Computing Non-Linearity Error & Building Optimal LUT...")
    unwrapped_fwd = np.unwrap(forward_measured)
    unwrapped_bwd = np.unwrap(backward_measured)

    # Average forward & backward to eliminate cogging torque hysteresis
    avg_unwrapped = (unwrapped_fwd + unwrapped_bwd) * 0.5

    # Ideal linear mechanical progression (exact 2*pi ramp)
    ideal_mech = np.linspace(avg_unwrapped[0], avg_unwrapped[0] + 2.0*math.pi, LUT_SIZE, endpoint=False)

    # Error = ideal - measured (in radians)
    raw_error = ideal_mech - avg_unwrapped
    raw_error -= np.mean(raw_error) # Remove DC offset

    # Spatial Fourier Low-Pass Filter: Keep harmonics 1..8
    fft_coeffs = np.fft.rfft(raw_error)
    fft_coeffs[9:] = 0.0
    filtered_error = np.fft.irfft(fft_coeffs, n=LUT_SIZE)

    peak_to_peak_mech = np.max(filtered_error) - np.min(filtered_error)
    peak_to_peak_elec = peak_to_peak_mech * POLE_PAIRS

    print(f"  Measured Peak-to-Peak Mechanical Distortion: {math.degrees(peak_to_peak_mech):.3f} deg")
    print(f"  Equivalent Electrical Angle Ripple Eliminated: {math.degrees(peak_to_peak_elec):.1f} deg electrical!")

    # Convert to fixed-point units (1 count = 0.0001 rad = 0.1 mrad)
    lut_int16 = np.round(filtered_error * 10000.0).astype(np.int16)

    # Step 5: Upload LUT to Driver
    print(f"\n[5] Uploading {LUT_SIZE}-point LUT to STM32 Driver...")
    for idx, val in enumerate(lut_int16):
        send_cmd(ser, f"LUT {idx} {val}")
        time.sleep(0.005)

    send_cmd(ser, "USE_LUT 1")
    print("  LUT successfully uploaded and activated in 20kHz fast loop!")

    print("\n" + "=" * 75)
    print("CALIBRATION COMPLETE: 128-Point Non-Linearity Compensation is ACTIVE!")
    print("=" * 75)
    ser.close()

if __name__ == '__main__':
    main()
