#include <Arduino.h>

#include <Wiring.h>
#include <CommsInterface.h>
#include <CommsRepeater.h>
#include <Taskmaster.h>
#include "DriveController.h"
#include "LidarController.h"
#include "Settings.h"
#include "Errors.h"

/*****************************************************
 *                   COMMUNICATIONS                  *
 *****************************************************/
CommsInterface g_externalComms;
CommsInterface g_peripheralComms;
CommsRepeater g_commsRepeater(&g_peripheralComms, &g_externalComms);

/*****************************************************
 *                    CONTROLLERS                    *
 *****************************************************/
Drivetrain g_drivetrain;
DriveController g_driveController(&g_drivetrain);
Lidar g_lidar;
LidarController g_lidarController(&g_lidar, &g_driveController);

/*****************************************************
 *                    TASKMASTERS                    *
 *****************************************************/
static ControllerGeneric* primaryControllers[] = { 
	&g_driveController, 
	&g_lidarController 
};
TASKMASTER_DECLARE(primaryTaskmaster, &g_externalComms, primaryControllers)


/**
 * @brief Global setup functions for board
 */
void setup()
{
	// Wiring
	Wiring_InitPins();
	Wiring_InitComms(&g_externalComms, &g_peripheralComms);
	Wiring_InitDrivetrain(&g_drivetrain);
	Wiring_InitLidar(&g_lidar);
}

/**
 * @brief Runtime loop for board
 */
void loop()
{
	primaryTaskmaster.execute();

	g_commsRepeater.process();
}