#include <Arduino.h>

#include <Wiring.h>
#include <CommsInterface.h>
#include <Taskmaster.h>
#include "PeripheralEnvoy.h"
#include "PeripheralEcho.h"
#include "PeripheralForwardingController.h"
#include "LidarController.h"
#include "Settings.h"
#include "Errors.h"

/*****************************************************
 *                   COMMUNICATIONS                  *
 *****************************************************/
CommsInterface g_externalComms;
CommsInterface g_peripheralComms;
PeripheralEnvoy g_envoyToPeripheral(&g_peripheralComms);

/*****************************************************
 *                    CONTROLLERS                    *
 *****************************************************/
PeripheralForwardingController g_peripheralForwardingController(&g_envoyToPeripheral);
Lidar g_lidar;
LidarController g_lidarController(&g_lidar, &g_envoyToPeripheral);

/*****************************************************
 *                    TASKMASTERS                    *
 *****************************************************/
static ControllerGeneric* primaryControllers[] = {
	&g_peripheralForwardingController,
	&g_lidarController 
};
TASKMASTER_DECLARE(primaryTaskmaster, &g_externalComms, primaryControllers)

/*****************************************************
 *                  PERIPHERAL ECHO                  *
 *****************************************************/

PeripheralEcho g_peripheralEcho(
	&g_peripheralComms, // Read from peripheral
	&g_externalComms,  // Echo on external
	&primaryTaskmaster // Priority sending
);


/**
 * @brief Global setup functions for board
 */
void setup()
{
	// Wiring
	Wiring_InitPins();
	Wiring_InitComms(&g_externalComms, &g_peripheralComms);
	Wiring_InitLidar(&g_lidar);
}

/**
 * @brief Runtime loop for board
 */
void loop()
{
	primaryTaskmaster.execute();

	g_peripheralEcho.process();
}