import serial
import time
import struct
import math

PKT_FMT = "<BBBBI 16f 3Bb 4f Bb H"
PKT_SIZE = struct.calcsize(PKT_FMT)

s = serial.Serial("/dev/ttyACM0", 115200, timeout=0.05)
time.sleep(0.1)
s.reset_input_buffer()

def read_telemetry(duration_s):
    samples = []
    t_end = time.time() + duration_s
    buf = bytearray()
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
                    samples.append({
                        "spd": unp[17],
                        "iq": unp[9],
                        "id": unp[8],
                        "vd": unp[25],
                        "vq": unp[26],
                        "mech": unp[15],
                        "fault": unp[23]
                    })
                except:
                    pass
        time.sleep(0.005)
    return samples

s.write(b"STOP\r\n")
time.sleep(0.2)
s.write(b"GEAR 1.0\r\n")
time.sleep(0.1)

speeds = [50.0, 100.0, 150.0, 200.0]

for spd in speeds:
    s.write(f"SPEED {spd:.1f}\r\n".encode())
    # Ramp up time
    time.sleep(1.5)
    
    # 2.0 seconds sample collection
    samples = read_telemetry(2.0)
    s.write(b"STOP\r\n")
    time.sleep(0.5)
    
    if samples:
        spds = [x["spd"] for x in samples]
        iqs = [x["iq"] for x in samples]
        mean_spd = sum(spds) / len(spds)
        std_spd = math.sqrt(sum((x - mean_spd)**2 for x in spds) / len(spds))
        mean_iq = sum(iqs) / len(iqs)
        err = mean_spd - spd
        err_pct = abs(err) / spd * 100.0
        print(f"Target: {spd:5.1f} RPM | Measured: {mean_spd:5.1f} RPM | StdDev: {std_spd:4.2f} RPM | Err: {err:+5.1f} RPM ({err_pct:4.1f}%) | Mean Iq: {mean_iq:5.3f} A")

s.close()
