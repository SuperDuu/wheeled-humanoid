import serial
import time
import struct
import math

PKT_FMT = "<BBBBI 16f 3Bb 4f Bb H"
PKT_SIZE = struct.calcsize(PKT_FMT)

s = serial.Serial("/dev/ttyACM0", 115200, timeout=0.05)
time.sleep(0.1)
s.reset_input_buffer()

def read_one():
    buf = bytearray()
    t_end = time.time() + 0.3
    while time.time() < t_end:
        c = s.read(64)
        if c:
            buf.extend(c)
            while len(buf) >= PKT_SIZE:
                idx = buf.find(b"\xaa\x55")
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
                    unp = struct.unpack(PKT_FMT, pkt)
                    return {
                        "joint_deg": math.degrees(unp[16]),
                        "spd": unp[17],
                        "iq": unp[9],
                        "iq_tgt": unp[10],
                        "vq": unp[26],
                        "mode": unp[21]
                    }
                except:
                    pass
    return None

def wait_for_steady(target_deg, timeout=4.0):
    t0 = time.time()
    last_samples = []
    while time.time() - t0 < timeout:
        pkt = read_one()
        if pkt and pkt["mode"] == 5:
            last_samples.append(pkt)
            if len(last_samples) > 10:
                last_samples.pop(0)
                # Check if speed is near 0 for 10 samples (~0.5s)
                spds = [abs(p["spd"]) for p in last_samples]
                if max(spds) < 3.0 and (time.time() - t0) > 1.2:
                    break
        time.sleep(0.05)
    
    if last_samples:
        avg_deg = sum(p["joint_deg"] for p in last_samples) / len(last_samples)
        avg_iq = sum(p["iq"] for p in last_samples) / len(last_samples)
        err = avg_deg - target_deg
        return avg_deg, err, avg_iq
    return 0.0, 999.0, 999.0

# Setup: stop, bare gear ratio, set home
s.write(b"STOP\r\n")
time.sleep(0.2)
s.write(b"GEAR 1.0\r\n")
time.sleep(0.2)
s.write(b"SETHOME\r\n")
time.sleep(0.3)
s.reset_input_buffer()

p0 = read_one()
print(f"Homed Position: {p0['joint_deg']:.2f}°")

targets = [45.0, 90.0, 180.0, 90.0, 0.0]

for tgt in targets:
    s.write(f"POS {tgt:.1f}\r\n".encode())
    meas, err, iq = wait_for_steady(tgt, timeout=4.0)
    print(f"Target: {tgt:5.1f}° | Measured: {meas:6.2f}° | Err: {err:+5.2f}° | Hold Iq: {iq:5.3f} A")

s.write(b"STOP\r\n")
s.close()
