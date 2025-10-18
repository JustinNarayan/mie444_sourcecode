#include "DriveController.h"
#include "Settings.h"

/**
 * @brief Interpret commands from comms interface
 * 
 * @param comms 
 * @return DrivetrainCommands received command, including NoReceivedCommand and InvalidCommand
 */
DrivetrainCommands DriveController::readCommsForCommand(CommsInterface* comms)
{
	// Poll comms interface
	comms->receive();

	// Determine if command was received
	char buffer[MESSAGE_LENGTH_MAX];
	bool hasCommandToRead = comms->popMessage(buffer);
	if (false == hasCommandToRead)
	{
		return DrivetrainCommands::NoReceivedCommand;
	}

	// Interpret message
	return stringAsDrivetrainCommand(buffer);
}


/**
 * @brief Arbitrate command to drivetrain and record it.
 * 
 * @param command 
 * @param drivetrain
 */
void DriveController::arbitrateCommand(DrivetrainCommands command, Drivetrain* drivetrain)
{
	// Record command as received
	this->lastReceivedCommand = command;
	this->lastReceivedCommandTimestampMillis = millis();

	// Apply command if not a continued command and record command as issued
	if (command != this->lastIssuedCommand)
	{
		applyCommand(command, drivetrain);
		this->lastIssuedCommand = command;
	}
}

/**
 * @brief Apply command to drivetrain.
 * 
 * @param command 
 * @param drivetrain
 */
void DriveController::applyCommand(DrivetrainCommands command, Drivetrain* drivetrain)
{
	switch (command)
	{
		case DrivetrainCommands::TranslateForward:
			drivetrain->setTranslate(DRIVETRAIN_TRANSLATE_SPEED, true);
			break;
		case DrivetrainCommands::TranslateBackward:
			drivetrain->setTranslate(DRIVETRAIN_TRANSLATE_SPEED, false);
			break;
		case DrivetrainCommands::RotateLeft:
			drivetrain->setRotate(DRIVETRAIN_ROTATE_SPEED, true);
			break;
		case DrivetrainCommands::RotateRight:
			drivetrain->setRotate(DRIVETRAIN_ROTATE_SPEED, false);
			break;
		case DrivetrainCommands::Halt:
		default:
			drivetrain->halt();
			break;
	}
}

/**
 * @brief Determine if drivetrain should halt. If no command has been recently received and 
 * drivetrain is not currently halted, halt.
 * 
 */
bool DriveController::shouldHalt(void)
{
	return (
		// Do not halt repeatedly
		(this->lastIssuedCommand != DrivetrainCommands::Halt) &&

		// Halt if too much time elapsed since received command
		((millis() - this->lastReceivedCommandTimestampMillis) > DRIVETRAIN_TIME_TO_HALT_AFTER_LAST_RECEIVED_COMMAND)
	);
}

/**
 * @brief Send command echo on comms interface
 * 
 */
void DriveController::sendCommandEcho(CommsInterface* comms, DrivetrainCommands command)
{
	char buffer[MESSAGE_LENGTH_MAX];
	drivetrainCommandAsString(command, buffer);
	comms->sendMessage(buffer);
}

/**
 * @brief Send response on comms interface
 * 
 */
void DriveController::sendResponse(CommsInterface* comms, DrivetrainResponses response)
{
	char buffer[MESSAGE_LENGTH_MAX];
	drivetrainResponseAsString(response, buffer);
	comms->sendMessage(buffer);
}


/**
 * @brief Receive commands from the comms interface and issue to Drivetrain. Send back an
 * appropriate acknowledge.
 * 
 * @param commsInterface 
 * @param drivetrain 
 */
void DriveController::processCommsForDrivetrain(CommsInterface* comms, Drivetrain* drivetrain)
{
	DrivetrainCommands currentCommand = readCommsForCommand(comms);

	// Send acknowledgement if invalid command
	if (currentCommand == DrivetrainCommands::InvalidCommand)
	{
		this->sendResponse(comms, DrivetrainResponses::AcknowledgeInvalidCommand);
		return;
	}
	
	// Send echo and acknowledgement if valid command
	if (currentCommand != DrivetrainCommands::NoReceivedCommand)
	{
		arbitrateCommand(currentCommand, drivetrain);
		this->sendCommandEcho(comms, currentCommand);
		this->sendResponse(comms, DrivetrainResponses::AcknowledgeValidCommand);
		return;
	}

	// Check if should halt based on no recent commands
	if (this->shouldHalt())
	{
		arbitrateCommand(DrivetrainCommands::Halt, drivetrain);
		this->sendCommandEcho(comms, DrivetrainCommands::Halt);
		this->sendResponse(comms, DrivetrainResponses::NotifyHalting);
		return;
	}
}