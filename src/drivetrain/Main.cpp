#include <Arduino.h>
#include <Wiring.h>
#include <DriveController.h>
#include "Errors.h"

/**
 * Global objects
 */
CommsInterface g_controllerComms;
Drivetrain g_drivetrain;
DriveController g_driveController;

/**
 * @brief Global setup functions for board
 */
void setup()
{
	Wiring_InitPins();
	Wiring_InitDrivetrain(&g_drivetrain);
	Wiring_InitComms(&g_controllerComms);
}

/**
 * @brief Runtime loop for board
 */
void loop()
{
	g_driveController.processCommsForDrivetrain(&g_controllerComms, &g_drivetrain);
}