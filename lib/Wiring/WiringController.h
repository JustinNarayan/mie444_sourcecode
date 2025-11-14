#pragma once
#include "Settings.h"
#include "Types.h"
#include <CommsInterface.h>
#include <Lidar.h>
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
#define UART_LIDAR (&Serial3)

/* Ultrasonic Sensors */
#define PIN_US_TEST_TRIGGER PB7
#define PIN_US_TEST_ECHO PB6

/* Lidar */
#define PIN_LIDAR_MOTOCTRL 9


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
 * @brief Initialize Lidar
 * 
 */
void Wiring_InitLidar(Lidar *lidar)
{
	// Initialize Lidar serial
	UART_LIDAR->begin(LIDAR_BAUD_RATE);
	
	lidar->init(
		PIN_LIDAR_MOTOCTRL, UART_LIDAR
	);
}