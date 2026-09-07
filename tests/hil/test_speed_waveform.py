import serial
import time
import struct

PKT_FMT = "<BBBBI 16f 3Bb 4f Bb H"
PKT_SIZE = struct.calcsize(PKT_FMT)

s = serial.Serial("/dev/ttyACM0", 115200, timeout=0.05)
time.sleep(0.1)
s.reset_input_buffer()

def read_one():
    buf = bytearray()
    t_end = time.time() + 0.1
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
                        "t": unp[4],
                        "spd": unp[17],
                        "iq": unp[9],
                        "iq_t": unp[10],
                        "vq": unp[26]
                    }
                except:
                    pass
    return None

s.write(b"STOP\r\n")
time.sleep(0.1)
s.write(b"GEAR 1.0\r\n")
time.sleep(0.1)
s.write(b"SPEED 50.0\r\n")
time.sleep(1.0)

samples = []
t0 = time.time()
while time.time() - t0 < 0.4:
    p = read_one()
    if p:
        samples.append(p)

s.write(b"STOP\r\n")
s.close()

for p in samples:
    print(f"t={p['t']}ms | spd={p['spd']:6.1f} RPM | iq={p['iq']:5.3f}A | iq_tgt={p['iq_t']:5.3f}A | vq={p['vq']:5.2f}V")
