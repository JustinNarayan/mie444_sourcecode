#pragma once
#include <Arduino.h>
#include <CommsInterface.h>
#include <Drivetrain.h>
#include "DrivetrainDefs.h"

class DriveController
{
private:
	/**
	 * Store last received command from comms
	 */
	DrivetrainManualCommand lastReceivedCommand;
	unsigned long lastReceivedCommandTimestampMillis;

	/**
	 * Store last issued command to drivetrain
	 */
	DrivetrainManualCommand lastIssuedCommand;

	
	DrivetrainManualCommand readCommsForCommand(CommsInterface* comms);
	void arbitrateCommand(DrivetrainManualCommand command, Drivetrain* drivetrain);
	void applyCommand(DrivetrainManualCommand command, Drivetrain* drivetrain);
	bool shouldHalt(void);
	void sendResponse(CommsInterface* comms, DrivetrainManualResponse response);
	void sendCommandEcho(CommsInterface* comms, DrivetrainManualCommand command);

public:
	void processCommsForDrivetrain(CommsInterface* comms, Drivetrain* drivetrain);
};