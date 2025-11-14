#include "DriveController.h"
#include "Settings.h"
#include <Translate.h>

/**
 * @brief Construct a new DriveController
 * 
 * @param drivetrain
 */
DriveController::DriveController(Drivetrain* drivetrain) : 
	drivetrain(drivetrain), 
	lastReceivedValidCommand(DrivetrainManualCommand::NoReceived),
	lastIssuedCommand(DrivetrainManualCommand::Halt) {}

/**
 * @brief Read input messages for DrivetrainManualCommand type
 * 
 * @return DrivetrainManualCommands received command, including NoReceivedCommand and InvalidCommand
 */
DrivetrainManualCommand DriveController::getDrivetrainManualCommand(void)
{
	// Dequeue DrivetrainManualCommand
	Message message;
	ControllerMessageQueueOutput ret = \
		this->read(MessageType::DrivetrainManualCommand, &message);

	// Translate DrivetrainManualCommand
	if (ret == ControllerMessageQueueOutput::DequeueQueueEmpty)
		return DrivetrainManualCommand::NoReceived;
	if (ret == ControllerMessageQueueOutput::DequeueSuccess)
		return DrivetrainManualCommandTranslation.asEnum(&message);
	return DrivetrainManualCommand::Invalid;
}

/**
 * @brief Write DrivetrainManualResponse
 * 
 * @param response To write
 */
void DriveController::sendDrivetrainManualResponse(DrivetrainManualResponse response)
{
	Message message;
	DrivetrainManualResponseTranslation.asMessage(response, &message);
	this->post(&message);
}


/**
 * @brief Arbitrate command to drivetrain and record it.
 * 
 * @param command 
 * @param drivetrain
 */
void DriveController::arbitrateCommand(DrivetrainManualCommand command)
{
	// Record command as received
	this->lastReceivedValidCommand = command;
	this->lastReceivedValidCommandTimestampMillis = millis();

	// Apply command if not a continued command and record command as issued
	if (command != this->lastIssuedCommand)
	{
		this->applyCommand(command);
		this->lastIssuedCommand = command;
	}
}

/**
 * @brief Apply command to drivetrain.
 * 
 * @param command 
 * @param drivetrain
 */
void DriveController::applyCommand(DrivetrainManualCommand command)
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
		((millis() - this->lastReceivedValidCommandTimestampMillis) > DRIVETRAIN_TIME_TO_HALT_AFTER_LAST_RECEIVED_COMMAND)
	);
}


/**
 * @brief Receive commands and issue to Drivetrain. Send back an appropriate acknowledge.
 * 
 */
void DriveController::process(void)
{
	DrivetrainManualCommand currentCommand = this->getDrivetrainManualCommand();

	// Send acknowledgement if invalid command
	if (currentCommand == DrivetrainManualCommand::Invalid)
	{
		this->sendDrivetrainManualResponse(DrivetrainManualResponse::AcknowledgeInvalidCommand);
		return;
	}
	
	// Send echo and acknowledgement if valid command
	if (currentCommand != DrivetrainManualCommand::NoReceived)
	{
		this->arbitrateCommand(currentCommand);
		this->sendDrivetrainManualResponse(DrivetrainManualResponse::AcknowledgeValidCommand);
		return;
	}

	// Check if should halt based on no recent commands
	if (this->shouldHalt())
	{
		this->voluntaryHalt();
		return;
	}
}

/**
 * @brief Bring drivetrain to a halt without direct command.
 * 
 */
void DriveController::voluntaryHalt(void)
{
	// Come to a stop and notify
	this->arbitrateCommand(DrivetrainManualCommand::Halt);
	this->sendDrivetrainManualResponse(DrivetrainManualResponse::NotifyHalting);

	// Prevent other stale from coming in immediately after
	this->purge(MessageType::DrivetrainManualCommand);
}