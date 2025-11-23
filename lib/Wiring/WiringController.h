#pragma once
#include "Settings.h"
#include "Types.h"
#include <CommsInterface.h>
#include <Lidar.h>
#include <Ultrasonic.h>
#include <Gripper.h>
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
#define PIN_ULTRASONIC_1_TRIGGER 54
#define PIN_ULTRASONIC_1_ECHO 55
#define PIN_ULTRASONIC_2_TRIGGER 56
#define PIN_ULTRASONIC_2_ECHO 57

/* Lidar */
#define PIN_LIDAR_MOTOCTRL 9

/* Gripper */
#define PIN_GRIPPER_ARM 2
#define PIN_GRIPPER_WRIST 3

/*****************************************************
 *                   INITIALIZERS                    *
 *****************************************************/

void Wiring_InitPins()
{
	// Blinker
	pinMode(PIN_BLINKER, OUTPUT);
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

/**
 * @brief Initialize ultrasonics
 * 
 */
void Wiring_InitUltrasonics(Ultrasonic* ultrasonic1, Ultrasonic* ultrasonic2)
{
    ultrasonic1->init(PIN_ULTRASONIC_1_ECHO, PIN_ULTRASONIC_1_TRIGGER);
    ultrasonic2->init(PIN_ULTRASONIC_2_ECHO, PIN_ULTRASONIC_2_TRIGGER);
}

/**
 * @brief Initialize servos
 * 
 */
void Wiring_InitGripper(Gripper* gripper)
{
    gripper->init(PIN_GRIPPER_ARM, PIN_GRIPPER_WRIST);
}
