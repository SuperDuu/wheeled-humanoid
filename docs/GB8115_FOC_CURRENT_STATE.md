# GB8115 FOC Current State

Last consolidated: 2026-08-20

## System

- Motor: GB8115-4 gimbal BLDC/PMSM, 21 pole pairs, approximately R = 3.90 ohm, L = 1.20 mH, Kv = 39.5 RPM/V.
- Gearbox: cycloidal reducer 1:17. High static friction and time-dependent mechanical/thermal losses are expected.
- MCU/driver: STM32G473 + DRV8353, 20 kHz PWM/FOC ISR, AS5048A SPI encoder.
- Control direction: open-loop voltage vector rotation works smoothly, so power stage, phase wiring, PWM/SVPWM, and bus supply are not the primary failure.

## Current Problem

Open-loop or fixed-voltage operation can spin the motor smoothly, but closed-loop speed mode loses torque/stalls after some time. The latest long-duration fixed-voltage test showed the speed also decays over time even at fixed Vq, including Vq = 5.0 V. That means the slow decay is not only a Speed PI bug; thermal winding resistance rise and cycloid loss are real contributors.

The firmware still had a separate closed-loop problem: Speed mode was allowed to command voltage far above the experimentally stable region. Earlier sweep data showed:

- Vq around 6.20 V to 6.30 V: best operating region, roughly 190 RPM to 197 RPM.
- Vq around 6.35 V: beginning of pull-out/torque degradation.
- Vq above 6.35 V: more oscillation and loss of torque.

Before the current fix, target 200 RPM could allow approximately target-BEMF plus 4.50 V headroom, about 9.56 V at 200 RPM. That contradicts the measured pull-out boundary.

## Root-Cause Summary

1. Open-loop being smooth only proves the inverter/SVPWM path is good. It does not validate encoder phase alignment or the closed-loop voltage law.
2. In Speed mode, `m_iq_set` is used as direct Vq in volts, but some comments and logic still treated it like current in amperes.
3. Holding high Vq while RPM falls is physically wrong for this voltage-mode setup. As BEMF falls, the voltage drop across phase resistance rises, causing more current, heating, and magnetic pull-out.
4. The safest control law for this motor is to command a bounded torque-voltage delta over actual BEMF:
   `Vq = Vbemf_actual + delta_torque`, with both absolute Vq and delta_torque limited.

## Current Code State

Relevant files:

- `firmware/joint_driver/joint-driver-8115/Core/Src/foc_math.c`
- `firmware/joint_driver/joint-driver-8115/Core/Src/foc_control.c`
- `firmware/joint_driver/joint-driver-8115/Core/Src/comm_telemetry.c`

Applied changes:

- Speed loop is now documented and implemented as voltage-mode output in volts.
- `Vq` is capped at 6.25 V.
- Torque-producing voltage delta is capped at 1.50 V continuous.
- Breakaway pulse is explicitly voltage-mode: 2.20 V, not 0.35 A.
- Speed derivative state now consistently uses mechanical RPM where it is compared as mechanical RPM.
- Spinup state is reset on init, STOP/OFF, MODE 0, CLOSELOOP/START, and SPEED commands.
- Added `<stddef.h>` to `foc_math.c` for explicit `NULL` definition.

Expected behavior after this fix:

- At 200 RPM, Vq cap remains 6.25 V.
- At 180 RPM, Vq cap is about 6.06 V.
- At 100 RPM, Vq cap is about 4.03 V.
- At 40 RPM, Vq cap is about 2.51 V.
- At 0 RPM, Vq cap is about 1.50 V.

This prevents the controller from dumping 6 V to 10 V into a nearly stopped motor after speed decay.

## Verification Done

- Host syntax check passed for the edited C files using `gcc -fsyntax-only`.
- Full STM32 build was not completed because `arm-none-eabi-gcc` is not available in the current PATH.
- No hardware validation was possible in this session.

Note: the first `make -C firmware/joint_driver/joint-driver-8115/Debug` invocation hit the generated default clean target and removed untracked build artifacts under `Debug/`. Those artifacts are ignored by git.

## Next Hardware Test

Use fresh power-on and motor cold state where possible.

1. `ALIGN`
2. `SPEED 150` for 40 seconds
3. `STOP`
4. `SPEED 180` for 40 seconds
5. `STOP`
6. `SPEED 190` for 40 seconds

Log at least:

- RPM actual/target
- Vq, Vd
- Id, Iq
- Vbus
- FET temperature

Pass condition:

- When RPM drops, Vq must drop with actual BEMF instead of staying pinned high.
- No negative RPM pole-slip.
- No slow current growth toward locked-rotor behavior.

If it still decays with this voltage law, the next variables to isolate are phase advance (`75 us` vs the previously working `320 us`) and encoder zero/alignment accuracy.
