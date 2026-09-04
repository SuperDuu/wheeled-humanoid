/*
	Copyright 2016 - 2022 Benjamin Vedder	benjamin@vedder.se
	Adapted for STM32G4 HAL Joint Driver - Cycloidal Actuator Project

	This file is part of the VESC firmware.
	GNU General Public License v3. See <http://www.gnu.org/licenses/>.
 */

#include "vesc_conf.h"
#include <math.h>

/**
  * @brief Load default VESC parameters tuned for GB8115-4 Motor & 1:17 Cycloidal Joint
  */
void vesc_conf_set_defaults(mc_configuration *conf)
{
    if (conf == NULL) return;

    // Switching Frequency & Limits
    conf->foc_f_zv = 20000.0f;           // 20 kHz PWM Frequency
    conf->l_max_duty = 0.92f;            // 92% max duty → 13.01V headroom, 4µs low-side window for ADC
    conf->l_min_duty = 0.005f;           // 0.5% min duty cycle

    // Controller phase-domain parameters for the assembled GB8115-4.
    // R was identified by the locked-rotor regression V=R*I+Vdrop (RMSE 0.058 V).
    // L is the per-phase value from the 3.14 mH line-to-line specification.
    // Lambda was identified from the 3 V direct-Vq run: (Vq-R*Iq)/omega_e ~= 0.030 Wb.
    conf->foc_motor_pole_pairs = 21;       // 21 Pole Pairs (42 Magnets)
    conf->foc_motor_r = 2.263f;            // Phase resistance 2.263 Ohm
    conf->foc_motor_l = 0.000100f;         // Phase inductance 100 µH
    conf->foc_motor_flux_linkage = 0.0300f;// Flux linkage 30.0 mWb (from direct identification)
    conf->foc_motor_ld_lq_diff = 0.0f;     // Surface PMSM (non-salient)

    // 1:17 Cycloid Gearbox Mode
    conf->gear_ratio = 17.0f;              // 1:17 Cycloidal Gearbox Reduction Ratio
    conf->encoder_direction = 1;           // Standard forward electrical angle rotation
    conf->joint_pos_min = -1000000.0f;     // Unlimited continuous rotation
    conf->joint_pos_max =  1000000.0f;     // Unlimited continuous rotation

    // Current Controller (PI D/Q) - 20kHz Inner Loop with Pole-Zero Cancellation (Ki/Kp = R/L = 22630 rad/s)
    conf->foc_current_kp = 0.80f;          // Bandwidth ~1270 Hz
    conf->foc_current_ki = 18100.0f;       // Instant current tracking (~0.18ms settling time)
    conf->foc_current_filter_const = 0.18f;
    conf->foc_cc_decoupling = FOC_CC_DECOUPLING_BEMF; // Bù khử ghép chéo d-q

    // Speed Controller (Feedforward + Damping + bounded Integral for zero steady-state error)
    conf->s_pid_kp = 0.0015f;             // Damping stiffness
    conf->s_pid_ki = 0.0020f;             // Bounded I-term (zero steady-state speed error)
    conf->s_pid_kd = 0.0f;                // Zero D-term
    conf->s_pid_kd_filter = 0.08f;        // 12 Hz filter
    conf->s_pid_min_erpm = 10.0f;          // 10 ERPM deadband (~0.48 RPM motor)
    conf->s_pid_ramp_erpms_s = 1500.0f;    // 1500 ERPM/s (~71 RPM/s smooth ramp)

    // Position Controller (MIT Mini Cheetah Impedance PD with near-target stiction compensation)
    conf->p_pid_kp = 12.0f;                // Kp_pos = 12.0 A/rad (~16.3 Nm/rad Joint Stiffness)
    conf->p_pid_ki = 1.0f;                 // Stiction compensation (< 5 deg, bounded to 0.5A)
    conf->p_pid_kd = 0.40f;                // Kd_pos = 0.40 A/(rad/s) (Virtual Joint Damping)
    conf->p_pid_kd_proc = 0.05f;           // Damping on measurement
    conf->p_pid_kd_filter = 0.2f;
    conf->p_pid_ang_div = 1.0f;
    conf->p_pid_gain_dec_angle = 0.0f;

    // Observer & Sensorless Configuration
    conf->foc_observer_type = FOC_OBSERVER_ORTEGA_ORIGINAL;
    // VESC Ortega scaling uses gamma * lambda^2 ~= 1000.
    conf->foc_observer_gain = 1.11e6f;
    // PLL Speed Estimator (wn=84 rad/s ~13Hz BW, zeta=1.0 for silent velocity tracking)
    conf->foc_pll_kp = 160.0f;             // K_pll_1 = 160.0
    conf->foc_pll_ki = 7000.0f;            // K_pll_2 = 7000.0
    conf->foc_sl_erpm = 2000.0f;

    // Field Weakening
    conf->foc_fw_current_max = 1.0f;       // 1.0A max field weakening for low-L GB8115
    conf->foc_fw_duty_start = 0.85f;       // Start FW only near voltage boundary (85% duty)
    conf->foc_fw_ramp_time = 0.2f;         // 200ms ramp time
    conf->foc_fw_backoff = 0.5f;

    // Overmodulation & Voltage Vector Limits
    conf->foc_overmod_factor = 1.0f;       // Standard space vector modulation
    conf->foc_mag_vd_max = 0.1f;           // Max 10% voltage in Vd (prevent d-axis stealing sampling margin)

    // Protection & Safety Limits (Nominal 2.1A, Dynamic Acceleration Peak 4.00A)
    conf->l_current_max = 4.00f;           // 4.0A peak for dynamic acceleration & breakaway
    conf->l_current_min = -4.00f;          // -4.0A braking limit
    conf->l_in_current_max = 20.0f;        // 20A max power supply input
    conf->l_in_current_min = -10.0f;       // -10A max regen charging
    conf->l_max_erpm = 11214.0f;           // 534 RPM * 21 = 11214 ERPM (Max Datasheet Speed)
    conf->l_min_erpm = -11214.0f;
    conf->l_max_erpm_fbreak = 15000.0f;
    conf->l_max_erpm_fbreak_cc = 15000.0f;
    conf->l_voltage_max = 50.0f;           // 50V max bus voltage (OVP)
    conf->l_voltage_min = 12.0f;           // 12V min bus voltage (UVP)
    conf->l_temp_fet_start = 85.0f;      // MOSFET thermal warning at 85C
    conf->l_temp_fet_end = 95.0f;        // MOSFET cutoff at 95C
    conf->l_temp_motor_start = 80.0f;    // Motor thermal warning at 80C
    conf->l_temp_motor_end = 90.0f;      // Motor cutoff at 90C

    // Sensor Mode
    conf->foc_sensor_mode = FOC_SENSOR_MODE_ENCODER; // SPI Encoder (AS5048A)
    conf->foc_encoder_inverted = false;
}
