#pragma once
#include <Arduino.h>
#include <CommsInterface.h>
#include "Settings.h"
#include "Wiring.h"

/**
 * External Communication Protocol
 * Ensure only one of USE_SERIAL or USE_BLE is commented
 */
// #define USE_SERIAL
#define USE_BLE

/**
 * Define all used pins on Arduino Mega 2560
 */
#define PIN_BLINKER LED_BUILTIN

/* UART */
#define UART_EXTERNAL_SERIAL (&Serial) // PE0 (RX) / PE1 (TX)
#define UART_EXTERNAL_BLE (&Serial2) // PH0 (RX) / PH1 (TX)
#define UART_INTERNAL_TO_DRIVETRAIN (&Serial1) // PD2 (RX) / PD3 (TX)

/* Ultrasonic Sensors */
#define PIN_US_TEST_TRIGGER PB7
#define PIN_US_TEST_ECHO PB6

/**
 * @brief Initialize all wiring and pin modes for the main board.
 */
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
void Wiring_InitComms(CommsInterface *externalComms, CommsInterface *drivetrainComms)
{
	// External communications setup
#if defined(USE_SERIAL)
	externalComms->init(UART_EXTERNAL_SERIAL, EXTERNAL_COMMS_BAUD_RATE);
#elif defined(USE_BLE)
	externalComms->init(UART_EXTERNAL_BLE, EXTERNAL_COMMS_BAUD_RATE);
#endif

	// Internal communications setup
	drivetrainComms->init(UART_INTERNAL_TO_DRIVETRAIN, INTERNAL_COMMS_BAUD_RATE);
}
