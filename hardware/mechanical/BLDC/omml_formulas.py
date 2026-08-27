# -*- coding: utf-8 -*-
"""
OMML Math Formula Definitions
"""

def omml_r(text):
    return f'<m:r><m:t>{text}</m:t></m:r>'

def omml_sub(base, sub):
    return f'<m:sSub><m:e><m:r><m:t>{base}</m:t></m:r></m:e><m:sub><m:r><m:t>{sub}</m:t></m:r></m:sub></m:sSub>'

def omml_sup(base, sup):
    return f'<m:sSup><m:e><m:r><m:t>{base}</m:t></m:r></m:e><m:sup><m:r><m:t>{sup}</m:t></m:r></m:sup></m:sSup>'

def omml_subsup(base, sub, sup):
    return f'<m:sSubSup><m:e><m:r><m:t>{base}</m:t></m:r></m:e><m:sub><m:r><m:t>{sub}</m:t></m:r></m:sub><m:sup><m:r><m:t>{sup}</m:t></m:r></m:sup></m:sSubSup>'

def omml_frac(num_xml, den_xml):
    return f'<m:f><m:num>{num_xml}</m:num><m:den>{den_xml}</m:den></m:f>'

def omml_delim(content_xml, beg="(", end=")"):
    return f'<m:d><m:dPr><m:begChr m:val="{beg}"/><m:endChr m:val="{end}"/></m:dPr><m:e>{content_xml}</m:e></m:d>'

# 1. Cycloid Gear Ratio
eq_cycloid_ratio = f'''
{omml_r("i = ")}
{omml_frac(omml_sub("n", "in"), omml_sub("n", "out"))}
{omml_r(" = ")}
{omml_frac(omml_r("N"), f'{omml_sub("Z", "p")}{omml_r(" - N")}')}
{omml_r(" = ")}
{omml_frac(omml_r("17"), omml_r("18 - 17"))}
{omml_r(" = 17:1")}
'''

eq_cycloid_speed_torque = f'''
{omml_sub("n", "out")}
{omml_r(" = ")}
{omml_frac(omml_sub("n", "in"), omml_r("i"))}
{omml_r(" = ")}
{omml_frac(omml_r("534"), omml_r("17"))}
{omml_r(" ≈ 31.41 rpm   (")}
{omml_sub("ω", "out")}
{omml_r(" ≈ 3.29 rad/s)")}
'''

eq_cycloid_torque = f'''
{omml_sub("T", "out")}
{omml_r(" = ")}
{omml_sub("T", "in")}
{omml_r(" · i · η ≈ 2.0 Nm · 17 · 0.85 = 28.9 Nm   (")}
{omml_sub("T", "peak")}
{omml_r(" > 50 Nm)")}
'''

# 2. Cycloid Profile
eq_psi = f'''
{omml_r("ψ(φ) = atan2")}
{omml_delim(f'{omml_r("-sin(N · φ), ")}{omml_frac(omml_sub("R", "p_mod"), omml_r("e · ") + omml_sub("Z", "p"))}{omml_r(" - cos(N · φ)")}')}
'''

eq_x1_y1 = f'''
{omml_sub("x", "1")}
{omml_r("(φ) = ")}
{omml_sub("R", "p_mod")}
{omml_r(" · cos(φ) - e · cos(")}
{omml_sub("Z", "p")}
{omml_r(" · φ) - ")}
{omml_sub("R", "r_mod")}
{omml_r(" · cos(φ + ψ)")}
'''

eq_y1 = f'''
{omml_sub("y", "1")}
{omml_r("(φ) = ")}
{omml_sub("R", "p_mod")}
{omml_r(" · sin(φ) - e · sin(")}
{omml_sub("Z", "p")}
{omml_r(" · φ) - ")}
{omml_sub("R", "r_mod")}
{omml_r(" · sin(φ + ψ)")}
'''

eq_theta_rot = f'''
{omml_sub("θ", "rot")}
{omml_r(" = ")}
{omml_frac(omml_r("π"), omml_r("N"))}
{omml_r(" = ")}
{omml_frac(omml_r("180°"), omml_r("17"))}
{omml_r(" ≈ 10.588°")}
'''

eq_x2_y2 = f'''
{omml_sub("x", "2")}
{omml_r(" = ")}
{omml_sub("x", "1")}
{omml_r(" · cos(")}
{omml_sub("θ", "rot")}
{omml_r(") - ")}
{omml_sub("y", "1")}
{omml_r(" · sin(")}
{omml_sub("θ", "rot")}
{omml_r("),    ")}
{omml_sub("y", "2")}
{omml_r(" = ")}
{omml_sub("x", "1")}
{omml_r(" · sin(")}
{omml_sub("θ", "rot")}
{omml_r(") + ")}
{omml_sub("y", "1")}
{omml_r(" · cos(")}
{omml_sub("θ", "rot")}
{omml_r(")")}
'''

# 3. PMSM d-q Differential Equations
eq_vd = f'''
{omml_sub("V", "d")}
{omml_r(" = R · ")}
{omml_sub("I", "d")}
{omml_r(" + ")}
{omml_sub("L", "d")}
{omml_frac(omml_r("d") + omml_sub("I", "d"), omml_r("dt"))}
{omml_r(" - ")}
{omml_sub("ω", "e")}
{omml_sub("L", "q")}
{omml_sub("I", "q")}
'''

eq_vq = f'''
{omml_sub("V", "q")}
{omml_r(" = R · ")}
{omml_sub("I", "q")}
{omml_r(" + ")}
{omml_sub("L", "q")}
{omml_frac(omml_r("d") + omml_sub("I", "q"), omml_r("dt"))}
{omml_r(" + ")}
{omml_sub("ω", "e")}
{omml_sub("L", "d")}
{omml_sub("I", "d")}
{omml_r(" + ")}
{omml_sub("ω", "e")}
{omml_sub("ψ", "m")}
'''

eq_torque_elec = f'''
{omml_sub("τ", "e")}
{omml_r(" = ")}
{omml_frac(omml_r("3"), omml_r("2"))}
{omml_r(" · PP · ")}
{omml_sub("ψ", "m")}
{omml_r(" · ")}
{omml_sub("I", "q")}
{omml_r(" = ")}
{omml_sub("K", "t")}
{omml_r(" · ")}
{omml_sub("I", "q")}
{omml_r("   (với PP = 21, ")}
{omml_sub("ψ", "m")}
{omml_r(" = 0.01160 Wb  ⟹  ")}
{omml_sub("K", "t")}
{omml_r(" ≈ 0.3654 N·m/A)")}
'''

eq_decoupling = f'''
{omml_subsup("V", "d", "final")}
{omml_r(" = ")}
{omml_subsup("V", "d", "PI")}
{omml_r(" - ")}
{omml_sub("ω", "e")}
{omml_sub("L", "s")}
{omml_sub("I", "q")}
{omml_r(",      ")}
{omml_subsup("V", "q", "final")}
{omml_r(" = ")}
{omml_subsup("V", "q", "PI")}
{omml_r(" + ")}
{omml_sub("ω", "e")}
{omml_sub("L", "s")}
{omml_sub("I", "d")}
{omml_r(" + ")}
{omml_sub("ω", "e")}
{omml_sub("ψ", "m")}
'''

# 4. Current PI Pole-Zero
eq_pi_current = f'''
{omml_sub("K", "p_curr")}
{omml_r(" = ")}
{omml_sub("L", "s")}
{omml_r(" · ")}
{omml_sub("ω", "bw")}
{omml_r(" = 0.00120 H · 5026.5 rad/s ≈ 6.03 V/A")}
'''

eq_ki_current = f'''
{omml_sub("K", "i_curr")}
{omml_r(" = ")}
{omml_sub("R", "s")}
{omml_r(" · ")}
{omml_sub("ω", "bw")}
{omml_r(" = 3.90 Ω · 5026.5 rad/s ≈ 19603.0 V/(A·s)   (")}
{omml_sub("f", "bw")}
{omml_r(" = 800 Hz)")}
'''

# 5. Phase Advance
eq_phase_advance = f'''
{omml_sub("θ", "svm")}
{omml_r(" = ")}
{omml_sub("θ", "e")}
{omml_r(" + 1.5 · ")}
{omml_sub("ω", "e")}
{omml_r(" · ")}
{omml_sub("T", "s")}
{omml_r(" = ")}
{omml_sub("θ", "e")}
{omml_r(" + 1.5 · ")}
{omml_sub("ω", "e")}
{omml_r(" · 0.00005 s")}
'''

# 6. Friction tanh
eq_friction_tanh = f'''
{omml_sub("I", "q_fric_ff")}
{omml_r(" = ")}
{omml_sub("I", "breakaway")}
{omml_r(" · tanh")}
{omml_delim(omml_frac(omml_sub("ω", "m"), omml_sub("ω", "threshold")))}
{omml_r(" + ")}
{omml_sub("B", "m")}
{omml_r(" · ")}
{omml_sub("ω", "m")}
'''

# 7. MIT Impedance PD
eq_mit_torque = f'''
{omml_sub("τ", "des")}
{omml_r(" = ")}
{omml_sub("K", "p")}
{omml_delim(f'{omml_sub("θ", "target")}{omml_r(" - θ")}')}
{omml_r(" + ")}
{omml_sub("K", "d")}
{omml_delim(f'{omml_sub("ω", "target")}{omml_r(" - ω")}')}
{omml_r(" + ")}
{omml_sub("τ", "ff")}
'''

eq_mit_current = f'''
{omml_sub("I", "q_cmd")}
{omml_r(" = clamp")}
{omml_delim(f'{omml_frac(omml_sub("τ", "des"), omml_sub("K", "t") + omml_r(" · Gear_Ratio"))}{omml_r(", -")}{omml_sub("I", "max")}{omml_r(", +")}{omml_sub("I", "max")}')}
'''

# 8. Quintic S-Curve
eq_quintic_s = f'''
{omml_r("s(τ) = 10 · ")}
{omml_sup("τ", "3")}
{omml_r(" - 15 · ")}
{omml_sup("τ", "4")}
{omml_r(" + 6 · ")}
{omml_sup("τ", "5")}
{omml_r("   (với τ = t / T ∈ [0, 1])")}
'''

print("All OMML math formulas defined successfully.")
