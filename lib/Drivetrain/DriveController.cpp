#include "DriveController.h"
#include "Settings.h"

/**
 * @brief Interpret commands from comms interface
 * 
 * @param comms 
 * @return DrivetrainCommands received command, including NoReceivedCommand and InvalidCommand
 */
DrivetrainCommand DriveController::readCommsForCommand(CommsInterface* comms)
{
	// Poll comms interface
	comms->receive();

	// Determine if command was received
	Message message;
	bool hasCommandToRead = comms->popMessage(&message);
	if (false == hasCommandToRead)
	{
		return DrivetrainCommand::NoReceived;
	}

	// Interpret message
	char buffer[MESSAGE_CONTENT_LENGTH_MAX];
	message.getContent(buffer);
	return getDrivetrainCommandFromString(buffer);
}


/**
 * @brief Arbitrate command to drivetrain and record it.
 * 
 * @param command 
 * @param drivetrain
 */
void DriveController::arbitrateCommand(DrivetrainCommand command, Drivetrain* drivetrain)
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
void DriveController::applyCommand(DrivetrainCommand command, Drivetrain* drivetrain)
{
	switch (command)
	{
		case DrivetrainCommand::TranslateForward:
			drivetrain->setTranslate(DRIVETRAIN_TRANSLATE_SPEED, true);
			break;
		case DrivetrainCommand::TranslateBackward:
			drivetrain->setTranslate(DRIVETRAIN_TRANSLATE_SPEED, false);
			break;
		case DrivetrainCommand::RotateLeft:
			drivetrain->setRotate(DRIVETRAIN_ROTATE_SPEED, true);
			break;
		case DrivetrainCommand::RotateRight:
			drivetrain->setRotate(DRIVETRAIN_ROTATE_SPEED, false);
			break;
		case DrivetrainCommand::Halt:
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
		(this->lastIssuedCommand != DrivetrainCommand::Halt) &&

		// Halt if too much time elapsed since received command
		((millis() - this->lastReceivedCommandTimestampMillis) > DRIVETRAIN_TIME_TO_HALT_AFTER_LAST_RECEIVED_COMMAND)
	);
}

/**
 * @brief Send command echo on comms interface
 * 
 */
void DriveController::sendCommandEcho(CommsInterface* comms, DrivetrainCommand command)
{
	char buffer[STRING_LENGTH_MAX];
	getStringFromDrivetrainCommand(command, buffer);
	Message message;
	message.init(MessageType::DrivetrainCommand, buffer);
	comms->sendMessage(&message);
}

/**
 * @brief Send response on comms interface
 * 
 */
void DriveController::sendResponse(CommsInterface* comms, DrivetrainResponse response)
{
	char buffer[STRING_LENGTH_MAX];
	getStringFromDrivetrainResponse(response, buffer);
	Message message;
	message.init(MessageType::DrivetrainResponse, buffer);
	comms->sendMessage(&message);
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
	DrivetrainCommand currentCommand = readCommsForCommand(comms);

	// Send acknowledgement if invalid command
	if (currentCommand == DrivetrainCommand::Invalid)
	{
		this->sendResponse(comms, DrivetrainResponse::AcknowledgeInvalidCommand);
		return;
	}
	
	// Send echo and acknowledgement if valid command
	if (currentCommand != DrivetrainCommand::NoReceived)
	{
		arbitrateCommand(currentCommand, drivetrain);
		this->sendCommandEcho(comms, currentCommand);
		this->sendResponse(comms, DrivetrainResponse::AcknowledgeValidCommand);
		return;
	}

	// Check if should halt based on no recent commands
	if (this->shouldHalt())
	{
		arbitrateCommand(DrivetrainCommand::Halt, drivetrain);
		this->sendCommandEcho(comms, DrivetrainCommand::Halt);
		this->sendResponse(comms, DrivetrainResponse::NotifyHalting);
		return;
	}
}