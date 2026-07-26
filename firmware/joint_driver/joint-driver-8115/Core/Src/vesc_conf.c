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
    conf->l_max_duty = 0.95f;            // 95% max duty cycle
    conf->l_min_duty = 0.005f;           // 0.5% min duty cycle

    // Motor Parameters (GB8115-4 Gimbal/Actuator Motor)
    conf->foc_motor_pole_pairs = 21;     // 21 Pole Pairs
    conf->foc_motor_r = 0.090f;          // ~90 mOhm phase resistance
    conf->foc_motor_l = 0.000120f;       // ~120 uH phase inductance
    conf->foc_motor_flux_linkage = 0.0045f; // ~4.5 mWb flux linkage
    conf->foc_motor_ld_lq_diff = 0.0f;   // Non-salient PMSM motor

    // Cycloidal Gearbox & Joint Safety Limits
    conf->gear_ratio = 17.0f;            // 1:17 Cycloidal reduction ratio
    conf->encoder_direction = 1;         // Normal encoder direction
    conf->joint_pos_min = -3.14159265f;  // -180 degrees (-PI rad)
    conf->joint_pos_max =  3.14159265f;  // +180 degrees (+PI rad)

    // Current Controller (PI D/Q)
    conf->foc_current_kp = 0.25f;
    conf->foc_current_ki = 150.0f;
    conf->foc_current_filter_const = 0.1f;
    conf->foc_cc_decoupling = FOC_CC_DECOUPLING_CROSS_BEMF; // BEMF + Cross decoupling

    // Speed Controller (PID)
    conf->s_pid_kp = 0.02f;
    conf->s_pid_ki = 0.4f;
    conf->s_pid_kd = 0.0001f;
    conf->s_pid_kd_filter = 0.2f;
    conf->s_pid_min_erpm = 10.0f;
    conf->s_pid_ramp_erpms_s = 50000.0f;

    // Position Controller (PID + Process D)
    conf->p_pid_kp = 15.0f;
    conf->p_pid_ki = 0.0f;
    conf->p_pid_kd = 0.03f;
    conf->p_pid_kd_proc = 0.02f;
    conf->p_pid_kd_filter = 0.2f;
    conf->p_pid_ang_div = 1.0f;
    conf->p_pid_gain_dec_angle = 0.0f;

    // Observer & Sensorless Configuration
    conf->foc_observer_type = FOC_OBSERVER_ORTEGA_ORIGINAL;
    conf->foc_observer_gain = 1000.0f;
    conf->foc_pll_kp = 2000.0f;
    conf->foc_pll_ki = 40000.0f;
    conf->foc_sl_erpm = 2000.0f;

    // Field Weakening
    conf->foc_fw_current_max = 5.0f;     // 5A max field weakening
    conf->foc_fw_duty_start = 0.90f;     // Start FW at 90% duty
    conf->foc_fw_ramp_time = 0.2f;       // 200ms ramp time
    conf->foc_fw_backoff = 0.5f;

    // Overmodulation & Voltage Vector Limits
    conf->foc_overmod_factor = 1.0f;     // Standard space vector modulation
    conf->foc_mag_vd_max = 0.2f;         // Max 20% voltage in Vd

    // Protection & Safety Limits
    conf->l_current_max = 25.0f;         // 25A max motor current
    conf->l_current_min = -25.0f;        // -25A max braking current
    conf->l_in_current_max = 20.0f;      // 20A max battery current
    conf->l_in_current_min = -10.0f;     // -10A max regen current
    conf->l_max_erpm = 100000.0f;        // 100k ERPM max
    conf->l_min_erpm = -100000.0f;
    conf->l_max_erpm_fbreak = 150000.0f;
    conf->l_max_erpm_fbreak_cc = 150000.0f;
    conf->l_voltage_max = 50.0f;         // 50V max bus voltage (OVP)
    conf->l_voltage_min = 12.0f;         // 12V min bus voltage (UVP)
    conf->l_temp_fet_start = 85.0f;      // MOSFET thermal warning at 85C
    conf->l_temp_fet_end = 95.0f;        // MOSFET cutoff at 95C
    conf->l_temp_motor_start = 80.0f;    // Motor thermal warning at 80C
    conf->l_temp_motor_end = 90.0f;      // Motor cutoff at 90C

    // Sensor Mode
    conf->foc_sensor_mode = FOC_SENSOR_MODE_ENCODER; // SPI Encoder (AS5048A)
    conf->foc_encoder_inverted = false;
}
