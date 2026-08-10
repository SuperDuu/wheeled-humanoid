/*
	Copyright 2016 - 2022 Benjamin Vedder	benjamin@vedder.se
	Adapted for STM32G4 HAL Joint Driver - Cycloidal Actuator Project

	This file is part of the VESC firmware.
	GNU General Public License v3. See <http://www.gnu.org/licenses/>.
 */

#ifndef MOTOR_INTERFACE_H_
#define MOTOR_INTERFACE_H_

#include "main.h"
#include "foc_control.h"

/* High-Level User API Functions for main.c */
void motor_init(SPI_HandleTypeDef *hspi1_drv, SPI_HandleTypeDef *hspi3_enc);
void motor_set_position(float deg);
void motor_set_speed(float rpm);
void motor_set_current(float iq_amps);

float motor_get_position(void);  // Output Joint Angle in Degrees (after 1:17 gearbox)
float motor_get_speed(void);     // Output Joint Velocity in RPM
float motor_get_current(void);   // Iq Torque Current in Amperes
float motor_get_vbus(void);      // VBUS Voltage in Volts
mc_fault_code motor_get_fault(void);

void motor_release(void);
void motor_full_brake(void);

#endif /* MOTOR_INTERFACE_H_ */
