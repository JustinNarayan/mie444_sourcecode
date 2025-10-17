#include <Arduino.h>

#include <Wiring.h>
#include <CommsInterface.h>
#include "Settings.h"
#include "Errors.h"

/**
 * Global objects
 */
CommsInterface g_externalComms;
CommsInterface g_drivetrainComms;

/**
 * @brief Global setup functions for board
 */
void setup()
{
	Wiring_InitPins();
	Wiring_InitComms(&g_externalComms, &g_drivetrainComms);
}

/**
 * @brief Runtime loop for board
 */
void loop()
{
	g_externalComms.receive();
	char buffer[MESSAGE_LENGTH_MAX];
	if (g_externalComms.popMessage(buffer))
	{
		g_externalComms.sendMessage(buffer);
	}
}