#include <Arduino.h>
#include <Wiring.h>
#include "Errors.h"

/**
 * Global objects
 */
CommsInterface g_controllerComms;

/**
 * @brief Global setup functions for board
 */
void setup()
{
	Wiring_InitPins();
	Wiring_InitComms(&g_controllerComms);
}

/**
 * @brief Runtime loop for board
 */
void loop()
{
}