#pragma once
#include <Arduino.h>
#include "Wiring.h"
#include <MotorController.h>

/**
 * Define all used pins on Arduino UNO
 */
// Miscellaneous
#define PIN_BLINKER LED_BUILTIN

// Drivetrain
#define PIN_MOTOR_A_IN1 PB5
#define PIN_MOTOR_A_IN2 PB4
#define PIN_MOTOR_A_ENABLE PB3
#define PIN_MOTOR_B_IN1 PB0
#define PIN_MOTOR_B_IN2 PB1
#define PIN_MOTOR_B_ENABLE PB2
#define PIN_MOTOR_C_IN1 PD7
#define PIN_MOTOR_C_IN2 PD6
#define PIN_MOTOR_C_ENABLE PD5
#define PIN_MOTOR_GRIPPER_IN1 PD4
#define PIN_MOTOR_GRIPPER_IN2 PD2
#define PIN_MOTOR_GRIPPER_ENABLE PD3

/**
 * Global objects
 */
MotorController drivetrainMotorA(PIN_MOTOR_A_ENABLE, PIN_MOTOR_A_IN1, PIN_MOTOR_A_IN2);
MotorController drivetrainMotorB(PIN_MOTOR_B_ENABLE, PIN_MOTOR_B_IN1, PIN_MOTOR_B_IN2);
MotorController drivetrainMotorC(PIN_MOTOR_C_ENABLE, PIN_MOTOR_C_IN1, PIN_MOTOR_C_IN2);
MotorController gripperMotor(PIN_MOTOR_GRIPPER_ENABLE, PIN_MOTOR_GRIPPER_IN1, PIN_MOTOR_GRIPPER_IN2);

/**
 * @brief Initialize all wiring and pin modes for the drivetrain board.
 */
void Wiring_Init()
{
	// Blinker
	pinMode(PIN_BLINKER, OUTPUT);

	// Drivetrain
	drivetrainMotorA.init();
	drivetrainMotorB.init();
	drivetrainMotorC.init();

	// Gripper
	gripperMotor.init();
}
