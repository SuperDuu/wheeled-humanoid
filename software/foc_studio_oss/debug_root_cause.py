#!/usr/bin/env python3
"""
ROOT CAUSE DEBUGGER: Find WHY Vq=4.1V instead of 5.1V at SPEED 200
Tests multiple hypotheses systematically using live telemetry.
"""
import serial, time, struct, threading

PORT = '/dev/ttyACM0'
BAUD = 115200
POLE_PAIRS = 21

class TelemetrySniffer:
    def __init__(self, ser):
        self.ser = ser
        self.samples = []
        self.running = True
        self.t = threading.Thread(target=self._read, daemon=True)
        self.t.start()

    def _read(self):
        buf = b''
        while self.running:
            try:
                chunk = self.ser.read(512)
                if chunk:
                    buf += chunk
                    while b'\n' in buf:
                        line, buf = buf.split(b'\n', 1)
                        try:
                            text = line.decode('utf-8', errors='replace').strip()
                            if 'Telemetry' in text and 'RUNNING' in text:
                                self._parse(text)
                        except:
                            pass
            except:
                pass

    def _parse(self, line):
        try:
            import re
            rpm_m = re.search(r'RPM=([+-]?\d+\.\d+)/([+-]?\d+\.\d+)', line)
            vq_m = re.search(r'Vq=([+-]?\d+\.\d+)V', line)
            id_m = re.search(r'Id=([+-]?\d+\.\d+)A', line)
            iq_m = re.search(r'Iq=([+-]?\d+\.\d+)A', line)
            if rpm_m and vq_m:
                self.samples.append({
                    'rpm': float(rpm_m.group(1)),
                    'rpm_tgt': float(rpm_m.group(2)),
                    'vq': float(vq_m.group(1)),
                    'id': float(id_m.group(1)) if id_m else 0,
                    'iq': float(iq_m.group(1)) if iq_m else 0,
                    't': time.time()
                })
        except:
            pass

    def stop(self):
        self.running = False

def cmd(ser, text, wait=0.2):
    ser.write((text + '\r\n').encode())
    time.sleep(wait)

def analyze_steady_state(samples, t_start, t_end):
    window = [s for s in samples if t_start <= s['t'] <= t_end]
    if not window:
        return None
    rpms = [s['rpm'] for s in window]
    vqs  = [s['vq']  for s in window]
    return {
        'n': len(window),
        'rpm_mean': sum(rpms)/len(rpms),
        'rpm_std':  (sum((r-sum(rpms)/len(rpms))**2 for r in rpms)/len(rpms))**0.5,
        'vq_mean':  sum(vqs)/len(vqs),
        'vq_min':   min(vqs),
        'vq_max':   max(vqs),
    }

print("=" * 70)
print("  ROOT CAUSE DEBUGGER - FOC Speed Oscillation")
print("=" * 70)

ser = serial.Serial(PORT, BAUD, timeout=0.1)
time.sleep(0.5)
sniff = TelemetrySniffer(ser)

# ──────────────────────────────────────────────────────────────────────
# STEP 1: ALIGN
# ──────────────────────────────────────────────────────────────────────
print("\n[1] ALIGN...")
cmd(ser, 'STOP', 0.3)
cmd(ser, 'ALIGN', 0.5)
time.sleep(7.0)
print("    Alignment done.")

# ──────────────────────────────────────────────────────────────────────
# STEP 2: Run at SPEED 200, capture Vq in steady state
#         → back-calculate effective lambda_m = Vq / (ERPM * 0.10472)
# ──────────────────────────────────────────────────────────────────────
print("\n[2] SPEED 200 → measure Vq steady-state (0 feedback, only FF)")
cmd(ser, 'KP_S 0.0', 0.2)
cmd(ser, 'KI_S 0.0', 0.2)
cmd(ser, 'KD_S 0.0', 0.2)
sniff.samples.clear()
t0 = time.time()
cmd(ser, 'SPEED 200', 0.1)
time.sleep(10.0)
cmd(ser, 'STOP', 0.3)

result_kp0 = analyze_steady_state(sniff.samples, t0+3, t0+9)
if result_kp0:
    erpm = result_kp0['rpm_mean'] * POLE_PAIRS
    lambda_eff = result_kp0['vq_mean'] / (erpm * 0.104719755) if erpm > 1 else 0
    print(f"  KP=0: RPM = {result_kp0['rpm_mean']:.1f} ± {result_kp0['rpm_std']:.1f}")
    print(f"  KP=0: Vq  = {result_kp0['vq_mean']:.3f}V  (min={result_kp0['vq_min']:.2f} max={result_kp0['vq_max']:.2f})")
    print(f"  → Effective lambda_m = {lambda_eff:.5f} Wb  (expected 0.01160)")
    print(f"  → Back-EMF const Kv  = {1/(lambda_eff*POLE_PAIRS*0.10472):.1f} RPM/V" if lambda_eff>0 else "")

time.sleep(2.0)

# ──────────────────────────────────────────────────────────────────────
# STEP 3: Correct lambda_m from measurement → set FLUX to correct value
# ──────────────────────────────────────────────────────────────────────
print("\n[3] Set FLUX to correct measured value and re-test...")
if result_kp0 and result_kp0['rpm_mean'] > 50:
    erpm = result_kp0['rpm_mean'] * POLE_PAIRS
    lambda_measured = result_kp0['vq_mean'] / (erpm * 0.104719755)
    # Expected Vq for 200 RPM at current lambda_measured:
    target_vq = lambda_measured * (200 * POLE_PAIRS * 0.104719755)
    print(f"  Measured lambda_m = {lambda_measured:.5f} Wb")
    print(f"  Target Vq at 200 RPM = {target_vq:.3f}V (actual measured)")
    # Use measured value
    cmd(ser, f'FLUX {lambda_measured:.5f}', 0.3)
else:
    print("  Could not measure (motor did not run), using 0.0116")
    cmd(ser, 'FLUX 0.01160', 0.3)

time.sleep(1.0)

# ──────────────────────────────────────────────────────────────────────
# STEP 4: Test with corrected lambda + KP=0 for 20s
# ──────────────────────────────────────────────────────────────────────
print("\n[4] Corrected FLUX + KP=0.0 for 20s → should run at FLAT 200 RPM")
cmd(ser, 'ALIGN', 0.5)
time.sleep(7.0)
sniff.samples.clear()
t0 = time.time()
cmd(ser, 'SPEED 200', 0.1)
time.sleep(20.0)
cmd(ser, 'STOP', 0.3)

# Segment analysis
segs = []
for start in [3, 7, 11, 15]:
    r = analyze_steady_state(sniff.samples, t0+start, t0+start+4)
    if r:
        segs.append((start, r))

print("\n  Segmented RPM (should be ~200 flat):")
for (s, r) in segs:
    print(f"  [{s:2d}s-{s+4:2d}s]: RPM = {r['rpm_mean']:+.1f} ± {r['rpm_std']:.2f} | Vq = {r['vq_mean']:.3f}V")

time.sleep(2.0)

# ──────────────────────────────────────────────────────────────────────
# STEP 5: Test with corrected lambda + gentle KP=0.0001 for 20s
# ──────────────────────────────────────────────────────────────────────
print("\n[5] Corrected FLUX + KP=0.0001 + KI=0 + KD=0 for 20s → is Kp the culprit?")
cmd(ser, 'KP_S 0.0001', 0.2)
cmd(ser, 'ALIGN', 0.5)
time.sleep(7.0)
sniff.samples.clear()
t0 = time.time()
cmd(ser, 'SPEED 200', 0.1)
time.sleep(20.0)
cmd(ser, 'STOP', 0.3)

segs2 = []
for start in [3, 7, 11, 15]:
    r = analyze_steady_state(sniff.samples, t0+start, t0+start+4)
    if r:
        segs2.append((start, r))

print("\n  Segmented RPM with KP=0.0001:")
for (s, r) in segs2:
    print(f"  [{s:2d}s-{s+4:2d}s]: RPM = {r['rpm_mean']:+.1f} ± {r['rpm_std']:.2f} | Vq = {r['vq_mean']:.3f}V")

# ──────────────────────────────────────────────────────────────────────
# CONCLUSION
# ──────────────────────────────────────────────────────────────────────
print("\n" + "=" * 70)
print("  DIAGNOSIS COMPLETE")
if segs and segs2:
    std_kp0 = sum(r['rpm_std'] for _,r in segs)/len(segs)
    std_kp1 = sum(r['rpm_std'] for _,r in segs2)/len(segs2)
    print(f"  KP=0.0000 → avg σ(RPM) = {std_kp0:.2f}")
    print(f"  KP=0.0001 → avg σ(RPM) = {std_kp1:.2f}")
    if std_kp1 > std_kp0 * 2:
        print("  ⚠ KP is causing instability even at 0.0001")
        print("  → Root cause: speed feedback (m_speed_est_fast) is too noisy")
        print("  → Fix: reduce PLL BW or low-pass filter speed before PID")
    else:
        print("  ✅ KP=0.0001 is acceptable → increase gently for load holding")
print("=" * 70)

sniff.stop()
ser.close()
