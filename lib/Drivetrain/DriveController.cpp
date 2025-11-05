#include "DriveController.h"
#include "Settings.h"
#include <Translate.h>

/**
 * @brief Interpret commands from comms interface
 * 
 * @param comms 
 * @return DrivetrainManualCommands received command, including NoReceivedCommand and InvalidCommand
 */
DrivetrainManualCommand DriveController::readCommsForCommand(CommsInterface* comms)
{
	// Poll comms interface
	comms->receive();

	// Determine if command was received
	Message message;
	bool hasCommandToRead = comms->popMessage(&message);
	if (false == hasCommandToRead)
	{
		return DrivetrainManualCommand::NoReceived;
	}

	// Interpret message
	return DrivetrainManualCommandTranslation.asEnum(&message);
}


/**
 * @brief Arbitrate command to drivetrain and record it.
 * 
 * @param command 
 * @param drivetrain
 */
void DriveController::arbitrateCommand(DrivetrainManualCommand command, Drivetrain* drivetrain)
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
void DriveController::applyCommand(DrivetrainManualCommand command, Drivetrain* drivetrain)
{
	switch (command)
	{
		case DrivetrainManualCommand::TranslateForward:
			drivetrain->setTranslate(DRIVETRAIN_TRANSLATE_SPEED, true);
			break;
		case DrivetrainManualCommand::TranslateBackward:
			drivetrain->setTranslate(DRIVETRAIN_TRANSLATE_SPEED, false);
			break;
		case DrivetrainManualCommand::RotateLeft:
			drivetrain->setRotate(DRIVETRAIN_ROTATE_SPEED, true);
			break;
		case DrivetrainManualCommand::RotateRight:
			drivetrain->setRotate(DRIVETRAIN_ROTATE_SPEED, false);
			break;
		case DrivetrainManualCommand::Halt:
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
		(this->lastIssuedCommand != DrivetrainManualCommand::Halt) &&

		// Halt if too much time elapsed since received command
		((millis() - this->lastReceivedCommandTimestampMillis) > DRIVETRAIN_TIME_TO_HALT_AFTER_LAST_RECEIVED_COMMAND)
	);
}

/**
 * @brief Send command echo on comms interface
 * 
 */
void DriveController::sendCommandEcho(CommsInterface* comms, DrivetrainManualCommand command)
{
	Message message;
	DrivetrainManualCommandTranslation.asMessage(command, &message);
	comms->sendMessage(&message);
}

/**
 * @brief Send response on comms interface
 * 
 */
void DriveController::sendResponse(CommsInterface* comms, DrivetrainManualResponse response)
{
	Message message;
	DrivetrainManualResponseTranslation.asMessage(response, &message);
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
	DrivetrainManualCommand currentCommand = readCommsForCommand(comms);

	// Send acknowledgement if invalid command
	if (currentCommand == DrivetrainManualCommand::Invalid)
	{
		this->sendCommandEcho(comms, currentCommand);
		this->sendResponse(comms, DrivetrainManualResponse::AcknowledgeInvalidCommand);
		return;
	}
	
	// Send echo and acknowledgement if valid command
	if (currentCommand != DrivetrainManualCommand::NoReceived)
	{
		arbitrateCommand(currentCommand, drivetrain);
		this->sendCommandEcho(comms, currentCommand);
		this->sendResponse(comms, DrivetrainManualResponse::AcknowledgeValidCommand);
		return;
	}

	// Check if should halt based on no recent commands
	if (this->shouldHalt())
	{
		arbitrateCommand(DrivetrainManualCommand::Halt, drivetrain);
		this->sendCommandEcho(comms, DrivetrainManualCommand::Halt);
		this->sendResponse(comms, DrivetrainManualResponse::NotifyHalting);
		return;
	}
}