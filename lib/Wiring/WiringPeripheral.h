#pragma once
#include "Settings.h"
#include "Types.h"
#include <CommsInterface.h>
#include "Wiring.h"

/**
 * Define all used pins on Arduino UNO
 */
#define PIN_BLINKER LED_BUILTIN

/* UART */
#define UART_INTERNAL_TO_CONTROLLER (&Serial)
#define PIN_CONTROLLER_RX PD0
#define PIN_CONTROLLER_TX PD1

/**
 * @brief Initialize all wiring and pin modes for the drivetrain board.
 */
void Wiring_InitPins()
{
	// Blinker
	pinMode(PIN_BLINKER, OUTPUT);
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
