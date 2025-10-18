#pragma once

/**
 * String literal messages for drivetrain communications
 */
#define DRIVETRAIN_COMMAND_TRANSLATE_FORWARD "w$\0"
#define DRIVETRAIN_COMMAND_TRANSLATE_BACKWARD "s$\0"
#define DRIVETRAIN_COMMAND_ROTATE_LEFT "a$\0"
#define DRIVETRAIN_COMMAND_ROTATE_RIGHT "d$\0"
#define DRIVETRAIN_COMMAND_HALT "h$\0"

#define DRIVETRAIN_RESPONSES_ACKNOWLEDGE_VALID_COMMAND "ackval$\0"
#define DRIVETRAIN_RESPONSES_ACKNOWLEDGE_INVALID_COMMAND "ackinv$\0"
#define DRIVETRAIN_RESPONSES_NOTIFY_HALTING "notifhalt$\0"

/**
 * Valid commands to be issued to the drivetrain.
 */
enum DrivetrainCommands
{
	InvalidCommand,
	NoReceivedCommand,
	TranslateForward,
	TranslateBackward,
	RotateLeft,
	RotateRight,
	Halt
};

/**
 * Valid messages for drivetrain board to issue back.
 */
enum DrivetrainResponses
{
	InvalidResponse,
	NoReceivedResponse,
	AcknowledgeValidCommand,
	AcknowledgeInvalidCommand,
	NotifyHalting
};

/**
 * @brief Convert drivetrain command to string.
 * 
 * @param command 
 * @param buffer 
 */
static inline void drivetrainCommandAsString(DrivetrainCommands command, char* buffer)
{
	switch (command)
	{
		case DrivetrainCommands::TranslateForward:
			memoryCopy(buffer, DRIVETRAIN_COMMAND_TRANSLATE_FORWARD, MESSAGE_LENGTH_MAX);
			break;
		case DrivetrainCommands::TranslateBackward:
			memoryCopy(buffer, DRIVETRAIN_COMMAND_TRANSLATE_BACKWARD, MESSAGE_LENGTH_MAX);
			break;
		case DrivetrainCommands::RotateLeft:
			memoryCopy(buffer, DRIVETRAIN_COMMAND_ROTATE_LEFT, MESSAGE_LENGTH_MAX);
			break;
		case DrivetrainCommands::RotateRight:
			memoryCopy(buffer, DRIVETRAIN_COMMAND_ROTATE_RIGHT, MESSAGE_LENGTH_MAX);
			break;
		case DrivetrainCommands::Halt:
			memoryCopy(buffer, DRIVETRAIN_COMMAND_HALT, MESSAGE_LENGTH_MAX);
			break;
		default:
			memorySet(buffer, 0, MESSAGE_LENGTH_MAX);
	}
}

/**
 * @brief Interpret string as drivetrain command.
 * 
 * @param buffer 
 * @return DrivetrainCommands 
 */
static inline DrivetrainCommands stringAsDrivetrainCommand(char* buffer)
{
	if (stringLength(buffer) == 0)
		return DrivetrainCommands::NoReceivedCommand;

	if (stringCompare(buffer, DRIVETRAIN_COMMAND_TRANSLATE_FORWARD) == true)
		return DrivetrainCommands::TranslateForward;
	if (stringCompare(buffer, DRIVETRAIN_COMMAND_TRANSLATE_BACKWARD) == true)
		return DrivetrainCommands::TranslateBackward;
	if (stringCompare(buffer, DRIVETRAIN_COMMAND_ROTATE_LEFT) == true)
		return DrivetrainCommands::RotateLeft;
	if (stringCompare(buffer, DRIVETRAIN_COMMAND_ROTATE_RIGHT) == true)
		return DrivetrainCommands::RotateRight;
	if (stringCompare(buffer, DRIVETRAIN_COMMAND_HALT) == true)
		return DrivetrainCommands::Halt;
	
	return DrivetrainCommands::InvalidCommand;
}

/**
 * @brief Convert drivetrain response to string.
 * 
 * @param response 
 * @param buffer 
 */
static inline void drivetrainResponseAsString(DrivetrainResponses response, char* buffer)
{
	switch (response)
	{
		case DrivetrainResponses::AcknowledgeValidCommand:
			memoryCopy(buffer, DRIVETRAIN_RESPONSES_ACKNOWLEDGE_VALID_COMMAND, MESSAGE_LENGTH_MAX);
			break;
		case DrivetrainResponses::AcknowledgeInvalidCommand:
			memoryCopy(buffer, DRIVETRAIN_RESPONSES_ACKNOWLEDGE_INVALID_COMMAND, MESSAGE_LENGTH_MAX);
			break;
		case DrivetrainResponses::NotifyHalting:
			memoryCopy(buffer, DRIVETRAIN_RESPONSES_NOTIFY_HALTING, MESSAGE_LENGTH_MAX);
			break;
		default:
			memorySet(buffer, 0, MESSAGE_LENGTH_MAX);
	}
}

/**
 * @brief Interpret string as drivetrain response.
 * 
 * @param buffer 
 * @return DrivetrainResponses 
 */
static inline DrivetrainResponses stringAsDrivetrainResponses(char* buffer)
{
	if (stringLength(buffer) == 0)
		return DrivetrainResponses::NoReceivedResponse;

	if (stringCompare(buffer, DRIVETRAIN_RESPONSES_ACKNOWLEDGE_VALID_COMMAND) == true)
		return DrivetrainResponses::AcknowledgeValidCommand;
	if (stringCompare(buffer, DRIVETRAIN_RESPONSES_ACKNOWLEDGE_INVALID_COMMAND) == true)
		return DrivetrainResponses::AcknowledgeInvalidCommand;
	if (stringCompare(buffer, DRIVETRAIN_RESPONSES_NOTIFY_HALTING) == true)
		return DrivetrainResponses::NotifyHalting;
	
	return DrivetrainResponses::InvalidResponse;
}
