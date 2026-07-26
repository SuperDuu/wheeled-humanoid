/*
	Copyright 2016 - 2022 Benjamin Vedder	benjamin@vedder.se
	Adapted for STM32G4 HAL Joint Driver - Cycloidal Actuator Project

	This file is part of the VESC firmware.
	GNU General Public License v3. See <http://www.gnu.org/licenses/>.
 */

#include "motor_interface.h"
#include "vesc_utils.h"

/**
  * @brief  Initialize FOC, Gate Driver (DRV8353RS), and Magnetic Encoder (AS5048A)
  */
void motor_init(SPI_HandleTypeDef *hspi1_drv, SPI_HandleTypeDef *hspi3_enc)
{
    // Initialize FOC Control System
    FOC_Control_Init(&g_foc_controller, hspi1_drv, hspi3_enc);

    // Initialize DRV8353RS Gate Driver via SPI1
    DRV8353_Init(&g_foc_controller.drv8353, hspi1_drv, DRV_CS_GPIO_Port, DRV_CS_Pin, DRV_EN_GPIO_Port, DRV_EN_Pin);
    DRV8353_SetCSAGain(&g_foc_controller.drv8353, DRV8353_CSA_GAIN_20V); // 20V/V gain

    // Initialize AS5048A Encoder via SPI3
    AS5048A_Init(&g_foc_controller.encoder, hspi3_enc, ENC_CS_GPIO_Port, ENC_CS_Pin);
}

/**
  * @brief  Set Target Joint Position in Degrees (-180 to +180 deg)
  */
void motor_set_position(float deg)
{
    g_foc_controller.motor.m_control_mode = CONTROL_MODE_POS;
    g_foc_controller.motor.m_pos_pid_set = DEG2RAD_f(deg);
}

/**
  * @brief  Set Target Joint Speed in RPM
  */
void motor_set_speed(float rpm)
{
    g_foc_controller.motor.m_control_mode = CONTROL_MODE_SPEED;
    g_foc_controller.motor.m_speed_command_rpm = rpm;
}

/**
  * @brief  Set Target Torque Current Iq in Amperes
  */
void motor_set_current(float iq_amps)
{
    g_foc_controller.motor.m_control_mode = CONTROL_MODE_CURRENT;
    g_foc_controller.motor.m_motor_state.iq_target = iq_amps;
}

/**
  * @brief  Get Current Output Joint Position in Degrees (after 1:17 Cycloidal reduction)
  */
float motor_get_position(void)
{
    return RAD2DEG_f(g_foc_controller.motor.m_joint_angle);
}

/**
  * @brief  Get Current Output Joint Speed in RPM
  */
float motor_get_speed(void)
{
    float motor_rpm = RADPS2RPM_f(g_foc_controller.motor.m_speed_est_fast);
    return motor_rpm / g_foc_controller.conf.gear_ratio;
}

/**
  * @brief  Get Current Iq Torque Current in Amperes
  */
float motor_get_current(void)
{
    return g_foc_controller.motor.m_motor_state.iq_filter;
}

/**
  * @brief  Get Bus Voltage VBUS in Volts
  */
float motor_get_vbus(void)
{
    return g_foc_controller.motor.m_motor_state.v_bus;
}

/**
  * @brief  Get Safety Fault Status
  */
mc_fault_code motor_get_fault(void)
{
    return g_foc_controller.fault;
}

/**
  * @brief  Release Motor (Disable PWM)
  */
void motor_release(void)
{
    g_foc_controller.motor.m_state = MC_STATE_OFF;
    g_foc_controller.duty_a = g_foc_controller.duty_b = g_foc_controller.duty_c = 0.5f;
}

/**
  * @brief  Full Brake Motor (Short 3 phases)
  */
void motor_full_brake(void)
{
    g_foc_controller.motor.m_state = MC_STATE_FULL_BRAKE;
    g_foc_controller.duty_a = g_foc_controller.duty_b = g_foc_controller.duty_c = 0.0f;
}
