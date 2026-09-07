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
                        "vd": unp[25]
                    }
                except:
                    pass
    return None

off = 6.0372
s.write(b"STOP\r\n")
time.sleep(0.2)
s.write(b"GEAR 1.0\r\n")
time.sleep(0.2)
s.write(f"OFFSET {off:.4f}\r\n".encode())
time.sleep(0.2)
s.write(b"SETHOME\r\n")
time.sleep(0.3)

print("Pre-move:", read_one())
s.write(b"POS 90.0\r\n")
t0 = time.time()
while time.time() - t0 < 3.5:
    pkt = read_one()
    if pkt:
        t = time.time() - t0
        print(f"t={t:5.2f}s | joint={pkt['joint_deg']:6.2f} deg | spd={pkt['spd']:5.1f} | iq={pkt['iq']:5.3f}A | iq_tgt={pkt['iq_tgt']:5.3f}A | vq={pkt['vq']:5.2f}V")
    time.sleep(0.1)

s.write(b"STOP\r\n")
s.close()
