#include "MemoryUtilities.h"
#include "PeripheralForwardingController.h"
#include <Translate.h>

/**
 * @brief Construct a new PeripheralForwardingController
 * 
 * @param envoy To send encoder requests to peripheral board
 */
PeripheralForwardingController::PeripheralForwardingController(PeripheralEnvoy *envoy) :
	envoy(envoy),
    hasUnaddressedEncoderRequest(false),
	drivetrainManualCommand(DrivetrainManualCommand::NoReceived),
    drivetrainManualCommandLastReceivedTime(0),
	drivetrainAutomatedCommand({0}),
    drivetrainAutomatedCommandLastReceivedTime(0) {}

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
bool PeripheralForwardingController::shouldEnvoyEncoderRequest(void)
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
 * @return If a command is ready to be sent
 */
bool PeripheralForwardingController::shouldEnvoyDrivetrainManualCommand(void)
{
	return (
        // External drivetrain commands not blocked
        (false == this->blockingExternalDrivetrainCommands) &&
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
 * @brief Read input messages for DrivetrainAutomatedCommand type
 * 
 */
void PeripheralForwardingController::checkDrivetrainAutomatedCommand(void)
{
	// Dequeue DrivetrainAutomatedCommand
	Message message;
	ControllerMessageQueueOutput ret = \
		this->read(MessageType::DrivetrainAutomatedCommand, &message);
	
	if (ret == ControllerMessageQueueOutput::DequeueSuccess)
	{
		DrivetrainAutomatedCommandTranslation.asStruct(&message, &this->drivetrainAutomatedCommand);
		this->drivetrainAutomatedCommandLastReceivedTime = millis();
        this->hasUnforwardDrivetrainAutomatedCommand = true;
	}
}

/**
 * @brief Forward drivetrain automated command
 * 
 */
void PeripheralForwardingController::envoyDrivetrainAutomatedCommand(void)
{
	this->envoy->envoyDrivetrainAutomatedCommand(&this->drivetrainAutomatedCommand);
	this->drivetrainAutomatedCommand = {0};
    this->hasUnforwardDrivetrainAutomatedCommand = false;
}

/**
 * @brief Determine if the current stored drivetrain automated command should be sent to the 
 * drivetrain
 * 
 * @return If a command is ready to be sent
 */
bool PeripheralForwardingController::shouldEnvoyDrivetrainAutomatedCommand(void)
{
	return (
        // Not blocking external drivetrain commands
        (false == this->blockingExternalDrivetrainCommands) &&
		// Has a command to send
		(this->hasUnforwardDrivetrainAutomatedCommand)
	);
}

/**
 * @brief Forward internal drivetrain automated command
 *  
 */
void PeripheralForwardingController::envoyInternalDrivetrainCommand(void)
{
	this->envoy->envoyDrivetrainAutomatedCommand(&this->internalDrivetrainCommand);
	this->internalDrivetrainCommand = {0};
    this->hasInternalDrivetrainCommand = false;
}

/**
 * @brief Determine if current internal drivetrain command should be envoyed.
 * 
 * @return If a command is ready to be sent
 */
bool PeripheralForwardingController::shouldEnvoyInternalDrivetrainCommand(void)
{
    return (
        // Only if blocking external drivetrain commands
        this->blockingExternalDrivetrainCommands &&
        // Has internal command
        this->hasInternalDrivetrainCommand
    );
}

/**
 * @brief Block or unblock external drivetrain commands. Used by external interfaces (i.e. other 
 * controllers)
 * 
 * @param shouldBlock 
 */
void PeripheralForwardingController::blockExternalDrivetrainCommands(bool shouldBlock)
{
    this->blockingExternalDrivetrainCommands = shouldBlock;
}

/**
 * @brief Copy command into internal drivetrain command
 * 
 * @param command 
 */
void PeripheralForwardingController::provideInternalDrivetrainCommand(
    DrivetrainAutomatedCommand* command
)
{
    memoryCopy(&this->internalDrivetrainCommand, command, sizeof(DrivetrainAutomatedCommand));
    this->hasInternalDrivetrainCommand = true;
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
	this->checkDrivetrainAutomatedCommand();

	// Check if should ping encoders
	if (this->shouldEnvoyEncoderRequest())
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

	// Check if should send drivetrain automated commands
	if (this->shouldEnvoyDrivetrainAutomatedCommand())
	{
        this->envoyDrivetrainAutomatedCommand();
	}

    // Check if should send internal drivetrain automated commands
    if (this->shouldEnvoyInternalDrivetrainCommand())
    {
        this->envoyInternalDrivetrainCommand();
    }
}