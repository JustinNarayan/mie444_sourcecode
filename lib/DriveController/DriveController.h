#pragma once
#include <Arduino.h>
#include <CommsInterface.h>
#include <Drivetrain.h>
#include "DrivetrainCommands.h"

class DriveController
{
private:
	/**
	 * Store last received command from comms
	 */
	DrivetrainCommands lastReceivedCommand;
	unsigned long lastReceivedCommandTimestampMillis;

	/**
	 * Store last issued command to drivetrain
	 */
	DrivetrainCommands lastIssuedCommand;

	
	DrivetrainCommands readCommsForCommand(CommsInterface* comms);
	void arbitrateCommand(DrivetrainCommands command, Drivetrain* drivetrain);
	void applyCommand(DrivetrainCommands command, Drivetrain* drivetrain);
	bool shouldHalt(void);
	void sendResponse(CommsInterface* comms, DrivetrainResponses response); 

public:
	void processCommsForDrivetrain(CommsInterface* comms, Drivetrain* drivetrain);
};