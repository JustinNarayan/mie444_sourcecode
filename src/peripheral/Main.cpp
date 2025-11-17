#include <Arduino.h>

#include <Wiring.h>
#include <CommsInterface.h>
#include <Taskmaster.h>
#include "DriveController.h"
#include "DriveEncoderController.h"
#include "Settings.h"
#include "Errors.h"
#include "Translate.h"

/*****************************************************
 *                   COMMUNICATIONS                  *
 *****************************************************/
static CommsInterface g_controllerComms;

/*****************************************************
 *                    CONTROLLERS                    *
 *****************************************************/
static DrivetrainEncoders g_drivetrainEncoders;
static DriveEncoderController g_driveEncoderController(&g_drivetrainEncoders);
Drivetrain g_drivetrain;
DriveController g_driveController(&g_drivetrain, &g_driveEncoderController);

/*****************************************************
 *                    TASKMASTERS                    *
 *****************************************************/
static ControllerGeneric* controllers[] = {
	&g_driveController,
	&g_driveEncoderController
};
TASKMASTER_DECLARE(taskmaster, &g_controllerComms, controllers)

/**
 * @brief Global setup functions for board
 */
void setup()
{
	Wiring_InitPins();
	Wiring_InitComms(&g_controllerComms);
	Wiring_InitDrivetrainEncoders(&g_drivetrainEncoders);
	Wiring_InitDrivetrain(&g_drivetrain);
}

/**
 * @brief Runtime loop for board
 */
void loop()
{
	taskmaster.execute();
}