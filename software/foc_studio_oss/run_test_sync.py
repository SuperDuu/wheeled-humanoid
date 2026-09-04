#!/usr/bin/env python3
import sys, os, time
sys.stdout.reconfigure(line_buffering=True)
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'src'))
from telemetry_parser import TelemetryParser, PACKET_SIZES, MAGIC1, MAGIC2
import serial

PORT = '/dev/ttyACM0'
BAUD = 115200

def send_cmd(ser, cmd):
    ser.write(f"{cmd}\r\n".encode())
    ser.flush()
    time.sleep(0.05)

def main():
    ser = serial.Serial(PORT, BAUD, timeout=0.05, write_timeout=0.5)
    time.sleep(0.2)

    print("======================================================================", flush=True)
    print("🔬 TEST SMOOTH STARTUP (50 RPM Closed Loop with 2.5ms Blanking)", flush=True)
    print("======================================================================", flush=True)

    send_cmd(ser, "STOP")
    time.sleep(0.5)
    ser.reset_input_buffer()

    print("Running ALIGN (18s)...", flush=True)
    send_cmd(ser, "ALIGN")
    time.sleep(20.0)
    print("ALIGN complete! Clearing buffer...", flush=True)
    ser.reset_input_buffer()
    time.sleep(0.5)

    print("Sending SPEED 50...", flush=True)
    send_cmd(ser, "SPEED 50")
    
    buf = bytearray()
    records = []
    t_start = time.time()
    last_p = t_start

    while time.time() - t_start < 10.0:
        if ser.in_waiting:
            buf.extend(ser.read(ser.in_waiting))
        while len(buf) >= 78:
            if buf[0] != MAGIC1 or buf[1] != MAGIC2:
                del buf[0]
                continue
            psize = int(buf[3]) + 4
            if psize not in PACKET_SIZES or len(buf) < psize:
                break
            p = TelemetryParser.parse_packet(bytes(buf[:psize]))
            if p:
                del buf[:psize]
                p['t'] = time.time() - t_start
                records.append(p)
                continue
            del buf[0]

        now = time.time()
        if now - last_p >= 0.5:
            last_p = now
            if records:
                r = records[-1]
                vq_val = r.get('vq', 0.0)
                print(f"[t={r['t']:4.2f}s] Speed: {r['speed_rpm']:5.1f} RPM | Iq_target: {r['i_q_target']:4.2f}A | Iq_meas: {r['i_q']:4.2f}A | Vq: {vq_val:4.2f}V", flush=True)
        time.sleep(0.005)

    send_cmd(ser, "STOP")
    time.sleep(0.5)
    ser.close()

    if records:
        iq_meas = [r['i_q'] for r in records[:50]]
        iq_tgt = [r['i_q_target'] for r in records[:50]]
        print("\n======================================================================", flush=True)
        print("STARTUP TRANSIENT (FIRST 500ms):", flush=True)
        print(f"Max Iq Target in first 500ms: {max(iq_tgt):.3f} A", flush=True)
        print(f"Max Iq Measured in first 500ms: {max(iq_meas):.3f} A", flush=True)
        print(f"Initial Iq Target at t=0: {iq_tgt[0]:.3f} A", flush=True)
        print(f"Initial Iq Measured at t=0: {iq_meas[0]:.3f} A", flush=True)

if __name__ == '__main__':
    main()
