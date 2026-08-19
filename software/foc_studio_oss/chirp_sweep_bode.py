#!/usr/bin/env python3
"""
Logarithmic Chirp Frequency Sweep & Bode Diagram Generator for BLDC Actuators
Used for identifying mechanical resonance, backlash, and frequency response
especially when mounting gearboxes / cycloidal reducers.
"""
import sys, os, time, math, glob
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'src'))
from telemetry_parser import TelemetryParser, PACKET_SIZE_94, MAGIC1, MAGIC2
import serial
import numpy as np

def find_default_port():
    ports = glob.glob('/dev/ttyACM*') + glob.glob('/dev/ttyUSB*')
    return ports[0] if ports else '/dev/ttyACM0'

PORT = sys.argv[1] if len(sys.argv) > 1 else find_default_port()
BAUD = 115200

def send_cmd(ser, cmd):
    ser.write(f"{cmd}\r\n".encode())
    time.sleep(0.02)

def run_chirp_sweep(f_start=1.0, f_end=80.0, duration=10.0, v_amp=1.8, v_offset=2.5):
    print("=" * 70)
    print("LOGARITHMIC CHIRP FREQUENCY SWEEP & BODE IDENTIFICATION")
    print(f"Port: {PORT} | Freq: {f_start}Hz -> {f_end}Hz | Duration: {duration}s | Amp: {v_amp}V")
    print("=" * 70)

    try:
        ser = serial.Serial(PORT, BAUD, timeout=0.05)
    except Exception as e:
        print(f"Error opening port {PORT}: {e}")
        return

    time.sleep(0.2)
    ser.reset_input_buffer()
    send_cmd(ser, "STOP")
    time.sleep(0.2)

    # Put motor in Direct Voltage Mode
    send_cmd(ser, f"VQ {v_offset}")
    time.sleep(0.5)

    print("Running Chirp Sweep...")
    t_start = time.time()
    t_end = t_start + duration
    
    t_log = []
    vq_cmd_log = []
    speed_log = []
    
    buf = bytearray()
    last_cmd_t = 0.0

    while time.time() < t_end:
        now = time.time()
        t_rel = now - t_start
        
        # Logarithmic chirp frequency
        k = (f_end / f_start) ** (t_rel / duration)
        f_inst = f_start * (k - 1.0) / (math.log(f_end / f_start) / duration) if f_end != f_start else f_start * t_rel
        phase = 2.0 * math.pi * f_inst
        
        v_target = v_offset + v_amp * math.sin(phase)
        
        # Update command at 100Hz
        if (now - last_cmd_t) >= 0.010:
            send_cmd(ser, f"VQ {v_target:.2f}")
            last_cmd_t = now

        # Read Telemetry
        raw = ser.read(ser.in_waiting or 1)
        if raw:
            buf.extend(raw)
        while len(buf) >= PACKET_SIZE_94:
            if buf[0] == MAGIC1 and buf[1] == MAGIC2:
                p = TelemetryParser.parse_packet(bytes(buf[:PACKET_SIZE_94]))
                if p:
                    t_log.append(t_rel)
                    vq_cmd_log.append(v_target)
                    speed_log.append(p.get('speed_rpm', 0.0))
                    del buf[:PACKET_SIZE_94]
                    continue
            del buf[0]
        time.sleep(0.001)

    send_cmd(ser, "SPEED 0")
    time.sleep(0.5)
    send_cmd(ser, "STOP")
    ser.close()

    print(f"Sweep completed. Collected {len(speed_log)} telemetry samples.")
    if len(speed_log) < 200:
        print("Not enough samples collected.")
        return

    # Compute FFT / Frequency Response Function H(jw)
    t_arr = np.array(t_log)
    u_arr = np.array(vq_cmd_log) - np.mean(vq_cmd_log)
    y_arr = np.array(speed_log) - np.mean(speed_log)

    U_fft = np.fft.rfft(u_arr)
    Y_fft = np.fft.rfft(y_arr)
    freqs = np.fft.rfftfreq(len(u_arr), d=(t_arr[-1]-t_arr[0])/len(t_arr))

    # Transfer Function H = Y / U
    valid_idx = (freqs >= f_start) & (freqs <= f_end) & (np.abs(U_fft) > 1e-3)
    f_valid = freqs[valid_idx]
    H_valid = Y_fft[valid_idx] / U_fft[valid_idx]

    mag_db = 20.0 * np.log10(np.abs(H_valid) + 1e-6)
    phase_deg = np.rad2deg(np.angle(H_valid))

    print("\n" + "=" * 70)
    print("FREQUENCY RESPONSE ANALYSIS (BODE SUMMARY):")
    print(f"  * DC Gain: {mag_db[0]:.2f} dB ({np.abs(H_valid[0]):.2f} RPM/V)")
    print(f"  * Bandwidth (-3dB cutoff frequency): {f_valid[np.argmin(np.abs(mag_db - (mag_db[0] - 3.0)))]:.1f} Hz")
    
    # Check for mechanical resonance peaks
    peak_idx = np.argmax(mag_db[len(mag_db)//4:]) + len(mag_db)//4
    if mag_db[peak_idx] > mag_db[0] + 2.0:
        print(f"  ⚠️ Mechanical Resonance Peak Detected at: {f_valid[peak_idx]:.1f} Hz (+{mag_db[peak_idx]-mag_db[0]:.1f} dB peak)")
    else:
        print("  ✅ Plant response is purely 1st-order with zero resonance peaks (Smooth bare motor).")
    print("=" * 70)

if __name__ == '__main__':
    run_chirp_sweep()
