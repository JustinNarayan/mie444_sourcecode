#pragma once
#include <Arduino.h>
#include "Wiring.h"

/**
 * Define all used pins on Arduino Mega 2560
 */
#define PIN_BLINKER LED_BUILTIN

/* Communication */
#define PIN_BLUETOOTH_RX PD2
#define PIN_BLUETOOTH_TX PD3
#define PIN_DRIVETRAIN_RX PE0
#define PIN_DRIVETRAIN_TX PE1

/* Ultrasonic Sensors */
#define PIN_US_TEST_TRIGGER PB7
#define PIN_US_TEST_ECHO PB6

/**
 * @brief Initialize all wiring and pin modes for the main board.
 */
void Wiring_Init()
{
	// Blinker
	pinMode(PIN_BLINKER, OUTPUT);

	// Ultrasonic Sensors
	pinMode(PIN_US_TEST_TRIGGER, OUTPUT);
	pinMode(PIN_US_TEST_ECHO, INPUT);
}
