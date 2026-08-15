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
    conf->l_max_duty = 0.92f;            // 92% max duty cycle
    conf->l_min_duty = 0.005f;           // 0.5% min duty cycle

    // Motor Parameters (GB8115-4 Gimbal/Actuator Motor - Official Datasheet)
    conf->foc_motor_pole_pairs = 21;       // 21 Pole Pairs
    conf->foc_motor_r = 3.89f;             // 3.89 Ohm Phase Resistance
    conf->foc_motor_l = 0.00314f;          // 3.14 mH Phase Inductance
    conf->foc_motor_flux_linkage = 0.02127f; // 0.02127 Wb Flux Linkage (Kt = 0.67 N.m/A, 2*Kt/(3*Pp))
    conf->foc_motor_ld_lq_diff = 0.0f;     // Surface PMSM (non-salient)

    // Cycloidal Gearbox & Joint Safety Limits
    conf->gear_ratio = 17.0f;              // 1:17 Cycloidal reduction ratio
    conf->encoder_direction = 1;           // Normal encoder direction
    conf->joint_pos_min = -3.14159265f;    // -180 degrees (-PI rad)
    conf->joint_pos_max =  3.14159265f;    // +180 degrees (+PI rad)

    // Current Controller (PI D/Q) - Pole-zero cancellation exact match: Kp=R=3.89, Ki=Kp*(R/L)=4819.0
    conf->foc_current_kp = 3.89f;          // Kp = R = 3.89 V/A
    conf->foc_current_ki = 4819.0f;        // Ki = Kp * (R/L) = 3.89 * (3.89/0.00314) = 4819.0 V/(A*s)
    conf->foc_current_filter_const = 0.1f;
    conf->foc_cc_decoupling = FOC_CC_DECOUPLING_DISABLED;

    // Speed Controller (PID) - Tuned for GB8115-4 (Kt = 0.67 N.m/A, J = 2574 g.cm^2)
    conf->s_pid_kp = 0.0005f;              // Proportional gain
    conf->s_pid_ki = 0.005f;               // Integral gain
    conf->s_pid_kd = 0.000005f;
    conf->s_pid_kd_filter = 0.2f;
    conf->s_pid_min_erpm = 5.0f;          // 5 ERPM (~0.2 RPM)
    conf->s_pid_ramp_erpms_s = 2000.0f;   // Ramp gia tốc mượt 2000 ERPM/s (~95 RPM/s)

    // Position Controller (PID + Process D) - Tuned for 1:17 Cycloid Joint
    conf->p_pid_kp = 3.5f;                 // Smooth proportional gain
    conf->p_pid_ki = 0.2f;                 // Low integral gain to remove steady-state error
    conf->p_pid_kd = 0.08f;                // Damping gain against oscillations
    conf->p_pid_kd_proc = 0.05f;           // Damping on measurement
    conf->p_pid_kd_filter = 0.2f;
    conf->p_pid_ang_div = 1.0f;
    conf->p_pid_gain_dec_angle = 0.0f;

    // Observer & Sensorless Configuration
    conf->foc_observer_type = FOC_OBSERVER_ORTEGA_ORIGINAL;
    conf->foc_observer_gain = 1000.0f;
    conf->foc_pll_kp = 100.0f;
    conf->foc_pll_ki = 1000.0f;
    conf->foc_sl_erpm = 2000.0f;

    // Field Weakening
    conf->foc_fw_current_max = 5.0f;       // 5A max field weakening
    conf->foc_fw_duty_start = 0.90f;       // Start FW at 90% duty
    conf->foc_fw_ramp_time = 0.2f;         // 200ms ramp time
    conf->foc_fw_backoff = 0.5f;

    // Overmodulation & Voltage Vector Limits
    conf->foc_overmod_factor = 1.0f;       // Standard space vector modulation
    conf->foc_mag_vd_max = 0.2f;           // Max 20% voltage in Vd

    // Protection & Safety Limits (Datasheet: Nominal 2.1A, Stall 6.6A, Max Speed 534 RPM = 11214 ERPM)
    conf->l_current_max = 6.6f;            // 6.6A Stall current limit
    conf->l_current_min = -6.6f;           // -6.6A Braking current limit
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
