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

print("Initializing motor state...")
s.write(b"STOP\r\n")
time.sleep(0.2)
s.write(b"GEAR 1.0\r\n")
time.sleep(0.2)

# Check alignment
s.write(b"ALIGN_INFO\r\n")
time.sleep(0.2)
resp = s.read_all().decode("utf-8", errors="ignore")
print(f"Alignment status: {resp.strip()}")
if "aligned: 1" not in resp:
    print("Running ALIGN...")
    s.write(b"ALIGN\r\n")
    time.sleep(3.0)
    s.write(b"ALIGN_INFO\r\n")
    time.sleep(0.2)
    print(s.read_all().decode("utf-8", errors="ignore").strip())

speeds = [50.0, 100.0, 150.0, 200.0, -50.0, -100.0, -150.0, -200.0]

print(f"{'TARGET':>8} | {'MEASURED':>10} | {'ERR':>7} | {'STD':>6} | {'IQ':>8} | {'ID':>8} | {'VD':>8} | {'VQ':>8}")
print("-" * 80)

for spd in speeds:
    s.write(f"SPEED {spd:.1f}\r\n".encode())
    time.sleep(1.5)
    
    samples = read_telemetry(2.0)
    s.write(b"STOP\r\n")
    time.sleep(0.5)
    
    if samples:
        spds = [x["spd"] for x in samples]
        iqs = [x["iq"] for x in samples]
        ids = [x["id"] for x in samples]
        vds = [x["vd"] for x in samples]
        vqs = [x["vq"] for x in samples]
        mean_spd = sum(spds) / len(spds)
        std_spd = math.sqrt(sum((x - mean_spd)**2 for x in spds) / len(spds))
        mean_iq = sum(iqs) / len(iqs)
        mean_id = sum(ids) / len(ids)
        mean_vd = sum(vds) / len(vds)
        mean_vq = sum(vqs) / len(vqs)
        err = mean_spd - spd
        print(f"{spd:+8.1f} | {mean_spd:+10.1f} | {err:+7.2f} | {std_spd:6.2f} | {mean_iq:+8.3f} | {mean_id:+8.3f} | {mean_vd:+8.2f} | {mean_vq:+8.2f}")
    else:
        print(f"{spd:+8.1f} | NO SAMPLES RECEIVED")

s.close()
