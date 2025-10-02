#pragma once
#include <Arduino.h>
#include "Wiring.h"

/**
 * Define all used pins on Arduino UNO
 */
#define PIN_BLINKER LED_BUILTIN

/**
 * @brief Initialize all wiring and pin modes for the drivetrain board.
 */
void Wiring_Init()
{
	// Blinker
	pinMode(PIN_BLINKER, OUTPUT);
}
