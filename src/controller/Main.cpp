#include <Arduino.h>

#include <Wiring.h>
#include <ExternalComms.h> // either USE_SERIAL or USE_BLE
#include "Settings.h"
#include "Errors.h"

/**
 * Global objects
 */
ExternalCommsActive g_externalComms;

/**
 * @brief Global setup functions for board
 */
void setup()
{
	Wiring_Init();
	g_externalComms.init();
}

/**
 * @brief Runtime loop for board
 */
void loop()
{
	delay(1000);
}