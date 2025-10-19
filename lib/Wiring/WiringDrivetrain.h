#pragma once
#include <Arduino.h>
#include <CommsInterface.h>
#include <MotorController.h>
#include <Drivetrain.h>
#include "Wiring.h"

/**
 * Define all used pins on Arduino UNO
 */
#define PIN_BLINKER LED_BUILTIN

/* UART */
#define UART_INTERNAL_TO_CONTROLLER (&Serial)
#define PIN_CONTROLLER_RX PD0
#define PIN_CONTROLLER_TX PD1

/* Drivetrain */
#define PIN_MOTOR_A_IN1 13
#define PIN_MOTOR_A_IN2 12
#define PIN_MOTOR_A_ENABLE 11
#define PIN_MOTOR_B_IN1 9
#define PIN_MOTOR_B_IN2 8
#define PIN_MOTOR_B_ENABLE 10
#define PIN_MOTOR_C_IN1 7
#define PIN_MOTOR_C_IN2 6
#define PIN_MOTOR_C_ENABLE 5
#define PIN_MOTOR_GRIPPER_IN1 PD4
#define PIN_MOTOR_GRIPPER_IN2 PD2
#define PIN_MOTOR_GRIPPER_ENABLE PD3

/**
 * @brief Initialize all wiring and pin modes for the drivetrain board.
 */
void Wiring_InitPins()
{
	// Blinker
	pinMode(PIN_BLINKER, OUTPUT);
}

/**
 * @brief Initialize drivetrain
 * 
 */
void Wiring_InitDrivetrain(Drivetrain *drivetrain)
{
	// Internal communications setup
	drivetrain->init(
		PIN_MOTOR_A_ENABLE, PIN_MOTOR_A_IN1, PIN_MOTOR_A_IN2,
		PIN_MOTOR_B_ENABLE, PIN_MOTOR_B_IN1, PIN_MOTOR_B_IN2,
		PIN_MOTOR_C_ENABLE, PIN_MOTOR_C_IN1, PIN_MOTOR_C_IN2
	);
}

/**
 * @brief Initialize communications protocols
 * 
 */
void Wiring_InitComms(CommsInterface *controllerComms)
{
	// Internal communications setup
	controllerComms->init(UART_INTERNAL_TO_CONTROLLER, INTERNAL_COMMS_BAUD_RATE);
}
