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
                        "mech": unp[15],
                        "spd": unp[17],
                        "iq": unp[9],
                        "vd": unp[25],
                        "vq": unp[26]
                    }
                except:
                    pass
    return None

s.write(b"STOP\r\n")
time.sleep(0.1)
s.reset_input_buffer()
p_start = read_one()
print("Start:", p_start)

s.write(b"IQ 0.3\r\n")
t0 = time.time()
while time.time() - t0 < 1.5:
    p = read_one()
    if p:
        t = time.time() - t0
        print(f"t={t:5.2f}s | mech={p['mech']:6.3f} rad | spd={p['spd']:6.1f} RPM | iq={p['iq']:5.3f}A | vq={p['vq']:5.2f}V")
    time.sleep(0.08)

s.write(b"STOP\r\n")
s.close()
