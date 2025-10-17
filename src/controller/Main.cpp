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
	// Receive external comms
	g_externalComms.receive();
	
	// Receive internal comms
	g_drivetrainComms.receive();

	// Echo external comms down internal comms
	char bufferExt[MESSAGE_LENGTH_MAX];
	if (g_externalComms.popMessage(bufferExt))
	{
		g_drivetrainComms.sendMessage(bufferExt);
	}

	// Echo internal comms up external_comms
	char bufferInt[MESSAGE_LENGTH_MAX];
	if (g_drivetrainComms.popMessage(bufferInt))
	{
		g_externalComms.sendMessage(bufferInt);
	}
}