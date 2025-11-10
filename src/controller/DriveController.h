#pragma once
#include "Types.h"
#include <CommsInterface.h>
#include <Controller.h>
#include <Drivetrain.h>
#include "DrivetrainDefs.h"

/*****************************************************
 *                INPUT / OUTPUT TYPES               *
 *****************************************************/
using MessageTypesInDrive = MessageTypes<
    MessageType::DrivetrainManualCommand // Manual commands
>;
using MessageTypesOutDrive = MessageTypes<
    MessageType::DrivetrainManualResponse, // Manual command responses
	MessageType::DrivetrainManualCommand // Manual command echo
>;

/*****************************************************
 *                     CONTROLLER                    *
 *****************************************************/
class DriveController : public Controller<
	MessageTypesInDrive, 
	MessageTypesOutDrive
>
{
private:
	/**
	 * Reference to Drivetrain
	 */
	Drivetrain* drivetrain;

	/**
	 * Store last received command from comms
	 */
	DrivetrainManualCommand lastReceivedValidCommand;
	unsigned long lastReceivedValidCommandTimestampMillis;

	/**
	 * Store last issued command to drivetrain
	 */
	DrivetrainManualCommand lastIssuedCommand;

	/**
	 * @brief Communication utilities
	 */
	DrivetrainManualCommand getDrivetrainManualCommand(void);
	void sendDrivetrainManualResponse(DrivetrainManualResponse response);
	void echoDrivetrainManualCommand(DrivetrainManualCommand command);

	/**
	 * @brief Process utilities
	 */
	void arbitrateCommand(DrivetrainManualCommand command);
	void applyCommand(DrivetrainManualCommand command);
	bool shouldHalt(void);
public:
	DriveController(Drivetrain* drivetrain);
	void process(void);

	void demandHalt(void);
};