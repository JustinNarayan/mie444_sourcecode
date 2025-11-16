#include "PeripheralForwardingController.h"
#include <Translate.h>

/**
 * @brief Construct a new PeripheralForwardingController
 * 
 * @param envoy To send encoder requests to peripheral board
 */
PeripheralForwardingController::PeripheralForwardingController(PeripheralEnvoy *envoy) :
	envoy(envoy),
	drivetrainManualCommand(DrivetrainManualCommand::NoReceived) {}

/**
 * @brief Read input messages for DrivetrainEncoderState type
 * 
 */
void PeripheralForwardingController::checkEncoderState(void)
{
	// Dequeue DrivetrainEncoderState
	Message message;
	ControllerMessageQueueOutput ret = \
		this->read(MessageType::DrivetrainEncoderState, &message);
	
	if (ret == ControllerMessageQueueOutput::DequeueSuccess)
	{
		// This controller only handles only requests for localization
		if (
			DrivetrainEncoderStateTranslation.asEnum(&message) != DrivetrainEncoderState::Request
		)
			return;

		this->hasUnaddressedEncoderRequest = true;

		// Eliminate stale messages
		this->purge(MessageType::DrivetrainEncoderState);
	}
}

/**
 * @brief Ping encoder for a reading
 * 
 */
void PeripheralForwardingController::envoyEncoderRequest(void)
{
	this->envoy->envoyEncoderRequest(); // request from localization
	this->hasUnaddressedEncoderRequest = false;
}

/**
 * @brief Determine if encoder should be pinged
 * 
 * @return if should ping
 */
bool PeripheralForwardingController::shouldEnvoyRequest(void)
{
	return (
		// Addressing a request
		this->hasUnaddressedEncoderRequest
	);
}

/**
 * @brief Read input messages for DrivetrainManualCommand type
 * 
 */
void PeripheralForwardingController::checkDrivetrainManualCommand(void)
{
	// Dequeue DrivetrainManualCommand
	Message message;
	ControllerMessageQueueOutput ret = \
		this->read(MessageType::DrivetrainManualCommand, &message);
	
	if (ret == ControllerMessageQueueOutput::DequeueSuccess)
	{
		this->drivetrainManualCommand = DrivetrainManualCommandTranslation.asEnum(&message);
		this->drivetrainManualCommandLastReceivedTime = millis();
	}
}

/**
 * @brief Forward drivetrain manual command
 * 
 */
void PeripheralForwardingController::envoyDrivetrainManualCommand(void)
{
	this->envoy->envoyDrivetrainManualCommand(this->drivetrainManualCommand);
	this->drivetrainManualCommand = DrivetrainManualCommand::NoReceived;
}

/**
 * @brief Determine if the current stored drivetrain manual command should be sent to the drivetrain
 * 
 * @return true 
 * @return false 
 */
bool PeripheralForwardingController::shouldEnvoyDrivetrainManualCommand(void)
{
	return (
		// Has a command to send (permit invalid commands for debugging purposes)
		(this->drivetrainManualCommand != DrivetrainManualCommand::NoReceived)
	);
}

/**
 * @brief Check if drivetrain manual command is stale. Discard if stale.
 * 
 * @return Whether command was not stale
 */
bool PeripheralForwardingController::validateDrivetrainManualCommandFresh(void)
{
	if (
		(millis() - this->drivetrainManualCommandLastReceivedTime) >
		DRIVETRAIN_MANUAL_COMMAND_FORWARDING_TIME_TO_DISCARD
	)
	{
		this->drivetrainManualCommand = DrivetrainManualCommand::NoReceived;
		return false;
	}
	return true;
}

/**
 * @brief Send encoder information upon request or periodically
 * 
 */
void PeripheralForwardingController::process(void)
{
	// Monitor incoming messages
	this->checkEncoderState();
	this->checkDrivetrainManualCommand();

	// Check if should ping encoders
	if (this->shouldEnvoyRequest())
	{
		this->envoyEncoderRequest();
	}

	// Check if should send drivetrain manual commands
	if (this->shouldEnvoyDrivetrainManualCommand())
	{
		if (this->validateDrivetrainManualCommandFresh())
		{
			this->envoyDrivetrainManualCommand();
		}
	}
}