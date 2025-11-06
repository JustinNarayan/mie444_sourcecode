#include <Arduino.h>

#include <Wiring.h>
#include <CommsInterface.h>
#include <Taskmaster.h>
#include <Controller.h>
#include "DriveController.h"
#include "Settings.h"
#include "Errors.h"

/*****************************************************
 *                   COMMUNICATIONS                  *
 *****************************************************/
CommsInterface g_externalComms;
CommsInterface g_peripheralComms;

/*****************************************************
 *                    CONTROLLERS                    *
 *****************************************************/
Drivetrain g_drivetrain;
DriveController g_driveController(&g_drivetrain);

/*****************************************************
 *                    TASKMASTERS                    *
 *****************************************************/
static ControllerGeneric* primaryControllers[] = { &g_driveController };
TASKMASTER_DECLARE(primaryTaskmaster, &g_externalComms, primaryControllers)


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
	// // Receive internal comms
	// g_peripheralComms.receive();

	primaryTaskmaster.execute();	

	// Echo internal comms up external_comms
	Message messageInt;
	if (g_peripheralComms.popMessage(&messageInt))
	{
		g_externalComms.sendMessage(&messageInt);
	}
}