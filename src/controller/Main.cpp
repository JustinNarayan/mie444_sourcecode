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
	Message messageExt;
	if (g_externalComms.popMessage(&messageExt))
	{
		g_drivetrainComms.sendMessage(&messageExt);
	}

	// Echo internal comms up external_comms
	Message messageInt;
	if (g_drivetrainComms.popMessage(&messageInt))
	{
		g_externalComms.sendMessage(&messageInt);
	}
}