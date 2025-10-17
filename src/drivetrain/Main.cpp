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
	// Receive comms
	g_controllerComms.receive();

	// Echo back comms with an ACK
	char buffer[MESSAGE_LENGTH_MAX];
	if (g_controllerComms.popMessage(buffer))
	{
		g_controllerComms.sendMessage(buffer);
		g_controllerComms.sendMessage("Acked$");
	}
}