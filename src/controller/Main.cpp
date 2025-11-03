#include <Arduino.h>

#include <Wiring.h>
#include <CommsInterface.h>
#include <DriveController.h>
#include "Settings.h"
#include "Errors.h"

/**
 * Global objects
 */
CommsInterface g_externalComms;
CommsInterface g_peripheralComms;
Drivetrain g_drivetrain;
DriveController g_driveController;

/**
 * @brief Global setup functions for board
 */
void setup()
{
	Wiring_InitPins();
	Wiring_InitComms(&g_externalComms, &g_peripheralComms);
	Wiring_InitDrivetrain(&g_drivetrain);
}

/**
 * @brief Runtime loop for board
 */
void loop()
{
	// Receive external comms
	g_externalComms.receive();
	
	// Receive internal comms
	g_peripheralComms.receive();

	// Echo internal comms up external_comms
	Message messageInt;
	if (g_peripheralComms.popMessage(&messageInt))
	{
		g_externalComms.sendMessage(&messageInt);
	}
	
	// Process drive controller communications
	g_driveController.processCommsForDrivetrain(&g_externalComms, &g_drivetrain);
}