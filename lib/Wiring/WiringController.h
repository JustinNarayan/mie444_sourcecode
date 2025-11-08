#pragma once
#include "Settings.h"
#include "Types.h"
#include <CommsInterface.h>
#include <Drivetrain.h>
#include "Wiring.h"

/**
 * External Communication Protocol
 * Ensure only one of USE_SERIAL or USE_BLE is commented
 */
// #define USE_SERIAL
#define USE_BLE

/*****************************************************
 *                  PIN SELECTIONS                   *
 *****************************************************/

#define PIN_BLINKER LED_BUILTIN

/* UART */
#define UART_EXTERNAL_SERIAL (&Serial) // PE0 (RX) / PE1 (TX)
#define UART_EXTERNAL_BLE (&Serial2) // PH0 (RX) / PH1 (TX)
#define UART_INTERNAL_TO_PERIPHERAL (&Serial1) // PD2 (RX) / PD3 (TX)

/* Ultrasonic Sensors */
#define PIN_US_TEST_TRIGGER PB7
#define PIN_US_TEST_ECHO PB6

/* Drivetrain */
#define PIN_MOTOR_1_IN1 6
#define PIN_MOTOR_1_IN2 7
#define PIN_MOTOR_1_ENABLE 5
#define PIN_MOTOR_2_IN1 8
#define PIN_MOTOR_2_IN2 9
#define PIN_MOTOR_2_ENABLE 10
#define PIN_MOTOR_3_IN1 12
#define PIN_MOTOR_3_IN2 13
#define PIN_MOTOR_3_ENABLE 11

/*****************************************************
 *                   INITIALIZERS                    *
 *****************************************************/

void Wiring_InitPins()
{
	// Blinker
	pinMode(PIN_BLINKER, OUTPUT);

	// Ultrasonic Sensors
	pinMode(PIN_US_TEST_TRIGGER, OUTPUT);
	pinMode(PIN_US_TEST_ECHO, INPUT);
}

/**
 * @brief Initialize communications protocols, including external communications protocols
 * 
 */
void Wiring_InitComms(CommsInterface *externalComms, CommsInterface *peripheralComms)
{
	// External communications setup
#if defined(USE_SERIAL)
	externalComms->init(UART_EXTERNAL_SERIAL, EXTERNAL_COMMS_BAUD_RATE);
#elif defined(USE_BLE)
	externalComms->init(UART_EXTERNAL_BLE, EXTERNAL_COMMS_BAUD_RATE);
#endif

	// Internal communications setup
	peripheralComms->init(UART_INTERNAL_TO_PERIPHERAL, INTERNAL_COMMS_BAUD_RATE);
}

/**
 * @brief Initialize drivetrain
 * 
 */
void Wiring_InitDrivetrain(Drivetrain *drivetrain)
{
	drivetrain->init(
		PIN_MOTOR_1_ENABLE, PIN_MOTOR_1_IN1, PIN_MOTOR_1_IN2,
		PIN_MOTOR_2_ENABLE, PIN_MOTOR_2_IN1, PIN_MOTOR_2_IN2,
		PIN_MOTOR_3_ENABLE, PIN_MOTOR_3_IN1, PIN_MOTOR_3_IN2
	);
}
