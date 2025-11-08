#include <Arduino.h>

#include <Wiring.h>
#include <CommsInterface.h>
#include <Taskmaster.h>
#include "DriveController.h"
#include "EchoController.h"
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
EchoController g_echoController; // echos messages on both Serial channels
Drivetrain g_drivetrain;
DriveController g_driveController(&g_drivetrain);

/*****************************************************
 *                    TASKMASTERS                    *
 *****************************************************/
static ControllerGeneric* primaryControllers[] = { 
	&g_echoController,
	&g_driveController
};
TASKMASTER_DECLARE(primaryTaskmaster, &g_externalComms, primaryControllers)
static ControllerGeneric* peripheralControllers[] = {
	&g_echoController,
};
TASKMASTER_DECLARE(peripheralTaskmaster, &g_peripheralComms, peripheralControllers)


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
	primaryTaskmaster.execute();
	peripheralTaskmaster.execute();
}