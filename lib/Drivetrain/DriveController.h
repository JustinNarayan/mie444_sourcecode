#pragma once
#include <Arduino.h>
#include <CommsInterface.h>
#include <Drivetrain.h>
#include "MessageDefs.h"

class DriveController
{
private:
	/**
	 * Store last received command from comms
	 */
	DrivetrainCommand lastReceivedCommand;
	unsigned long lastReceivedCommandTimestampMillis;

	/**
	 * Store last issued command to drivetrain
	 */
	DrivetrainCommand lastIssuedCommand;

	
	DrivetrainCommand readCommsForCommand(CommsInterface* comms);
	void arbitrateCommand(DrivetrainCommand command, Drivetrain* drivetrain);
	void applyCommand(DrivetrainCommand command, Drivetrain* drivetrain);
	bool shouldHalt(void);
	void sendResponse(CommsInterface* comms, DrivetrainResponse response);
	void sendCommandEcho(CommsInterface* comms, DrivetrainCommand command);

public:
	void processCommsForDrivetrain(CommsInterface* comms, Drivetrain* drivetrain);
};