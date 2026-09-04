# GB8115 FOC Current State & System Specification

**Consolidated Date:** 2026-09-04  
**Author:** Vu Duc Du  
**Target Hardware:** STM32G473RET6 + DRV8353RS + AS5048A (SPI3) + GB8115-4 Gimbal Motor (21 Pole Pairs) + Cycloidal Gearbox 1:17  
**Validation Tooling:** FOC Studio OSS (FastAPI + High-Speed Telemetry Streaming @ 100 Hz, ST-LINK SWD CLI)

---

## 1. System Identification & Hardware Parameters

Through empirical regression and locked-rotor identification, the true physical parameters of the actuator have been established:

| Parameter | Symbol | Value | Unit | Method / Verification |
| :--- | :--- | :--- | :--- | :--- |
| **Number of Pole Pairs** | $p$ | 21 | - | Physical stator/magnet count (36S/42P) |
| **Phase Resistance** | $R_s$ | 2.263 | $\Omega$ | Locked-rotor regression $V = R \cdot I + V_{drop}$ (RMSE 0.058V) |
| **Phase Inductance** | $L_s$ | 100 | $\mu\text{H}$ | 20 kHz current rise-time measurement |
| **Permanent Magnet Flux Linkage** | $\lambda$ | 0.0300 | Wb | Identified from open-circuit BEMF & closed-loop $V_q$ tracking |
| **Motor Torque Constant** | $K_{t,motor}$ | 0.945 | $\text{N}\cdot\text{m/A}$ | $K_t = \frac{3}{2} p \lambda = 1.5 \times 21 \times 0.0300$ |
| **Cycloid Reduction Ratio** | $N$ | 17.0 | - | Mechanical pin/roller ratio |
| **Joint Torque Constant (Theoretical)** | $K_{t,joint}$ | 16.065 | $\text{N}\cdot\text{m/A}$ | $K_{t,motor} \times 17$ |
| **Joint Torque Constant (Real-world)** | $K_{t,joint,eff}$ | $\approx 12.85$ | $\text{N}\cdot\text{m/A}$ | Accounting for $\eta \approx 80\%$ cycloidal forward efficiency |
| **Continuous Safe Current** | $I_{cont}$ | 1.5 | A | $P_{loss} \approx 7.6\text{ W}$ (Continuous cool operation $< 50^\circ\text{C}$) |
| **Dynamic Acceleration Peak Current** | $I_{peak}$ | 3.5 – 4.0 | A | Up to 10 seconds for breakaway and acceleration |

---

## 2. Architecture: Correction of Earlier Hypotheses

> [!NOTE]
> **Historic Clarification (Debunking Voltage-Mode Control):**  
> Earlier notes (August 2026) hypothesized that the closed-loop current controller should be abandoned in favor of *"Voltage-Mode FOC"* with artificial clamps ($V_q \le 6.25\text{ V}$, $V_d \le 1.4\text{ V}$).  
> **Experimental testing in September 2026 proved this hypothesis was incorrect:** The torque pull-out and stall above 100 RPM were directly caused by the artificial $V_d$ clamp starving the d-axis voltage margin as BEMF rose, combined with SysTick interrupt starvation (priority 15 SysTick starved by priority 0 ADC).  
> Once the interrupt priority inversion and blocking SPI loops were fixed, **a pure 20 kHz cascaded FOC Current Loop proved vastly superior, robust, and completely stable across the full operating envelope.**

---

## 3. Active Control Architecture (Firmware State)

### A. Fast Inner Loop (20 kHz ADC Injected ISR)
* **Sampling:** Low-side shunt current sampling synchronized to PWM timer bottom (`TIM_COUNTER_ZERO`).
* **Decoupling:** Dynamic d-q cross-coupling and BEMF feedforward decoupling:
  $$v_d = v_d^{PI} - \omega_e L_s i_q$$
  $$v_q = v_q^{PI} + \omega_e L_s i_d + \omega_e \lambda$$
* **Current PI Tuning (Pole-Zero Cancellation):**
  $$\frac{K_i}{K_p} = \frac{R_s}{L_s} = \frac{2.263}{0.000100} = 22630\text{ rad/s}$$
  * $K_p = 0.80\text{ V/A}$, $K_i = 18100\text{ V/(A}\cdot\text{s)}$.
  * Current loop bandwidth: $\approx 1270\text{ Hz}$ (settling time $< 0.2\text{ ms}$).

### B. Speed Loop & Friction Compensation (1 kHz)
* **Speed Observer:** 2nd-order PLL with 32 Hz filter bandwidth (`s_pid_kd_filter = 0.20`), eliminating phase lag.
* **Gains:** $K_p = 0.0006$, $K_i = 0.0015$ with anti-windup clamping.
* **Friction Feedforward:** Continuous hyperbolic tangent model replacing discontinuous step jump:
  $$i_{q,ff} = 0.12 \cdot \tanh\left(\frac{\omega}{15}\right)\text{ [A]}$$

### C. Position Trajectory & Impedance Control (1 kHz)
* **Quintic Minimum-Jerk Trajectory:** $C^2$-continuous position, velocity, and acceleration profile:
  $$s(\tau) = 10\tau^3 - 15\tau^4 + 6\tau^5$$
* **Near-Target Stiction Compensation:** Bounded integral active within $|\Delta\theta| < 5^\circ$ ($K_i = 1.0$, limit $0.5\text{ A}$), ensuring steady-state error $\le 0.30^\circ$ without limit cycles.
* **MIT Mini Cheetah Real-Time Impedance Protocol:**
  $$\tau_{cmd} = K_p (p_{des} - p_{actual}) + K_d (v_{des} - v_{actual}) + \tau_{ff}$$
  Returns true measured contact torque ($t_{actual} = K_{t,joint} \cdot I_{q,filter}$) over CAN and USB.

---

## 4. Hardware Verification Summary

Three consecutive trials across all operational modes confirmed 100% repeatability:
* **Speed Mode (50, 100, 150, 200 RPM):** Steady-state error $\le 1.0\text{ RPM}$, ripple standard deviation $\sigma = 2.6 - 4.7\text{ RPM}$ (reduced from 23.4 RPM).
* **Position Mode ($+45^\circ, +90^\circ, +180^\circ, -90^\circ$):** Final settling accuracy $0.00^\circ - 0.03^\circ$, overshoot $0\%$.
* **CAN & USB Telemetry:** 100.2 Hz streaming with zero dropped packets and zero CRC errors.
