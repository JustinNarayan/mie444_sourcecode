#pragma once
#include "Settings.h"
#include "MemoryUtilities.h"
// #include "MessageEnumTemplate.h"

/**
 * String literal messages for valid drivetrain commands.
 */
#define DRIVETRAIN_COMMAND_TRANSLATE_FORWARD "w\0"
#define DRIVETRAIN_COMMAND_TRANSLATE_BACKWARD "s\0"
#define DRIVETRAIN_COMMAND_ROTATE_LEFT "a\0"
#define DRIVETRAIN_COMMAND_ROTATE_RIGHT "d\0"
#define DRIVETRAIN_COMMAND_HALT "h\0"

/**
 * Valid commands to be issued to the drivetrain.
 */
enum class DrivetrainCommand
{
	Invalid,
	NoReceived,
	TranslateForward,
	TranslateBackward,
	RotateLeft,
	RotateRight,
	Halt
};

/**
 * String literal messages for valid drivetrain responses.
 */
#define DRIVETRAIN_RESPONSES_ACKNOWLEDGE_VALID_COMMAND "ackval\0"
#define DRIVETRAIN_RESPONSES_ACKNOWLEDGE_INVALID_COMMAND "ackinv\0"
#define DRIVETRAIN_RESPONSES_NOTIFY_HALTING "notifhalt\0"

/**
 * Valid messages for drivetrain board to issue back.
 */
enum class DrivetrainResponse
{
	Invalid,
	NoReceived,
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
static inline void getStringFromDrivetrainCommand(DrivetrainCommand command, char* buffer)
{
	switch (command)
	{
		case DrivetrainCommand::TranslateForward:
			memoryCopy(buffer, DRIVETRAIN_COMMAND_TRANSLATE_FORWARD, MESSAGE_CONTENT_LENGTH_MAX);
			break;
		case DrivetrainCommand::TranslateBackward:
			memoryCopy(buffer, DRIVETRAIN_COMMAND_TRANSLATE_BACKWARD, MESSAGE_CONTENT_LENGTH_MAX);
			break;
		case DrivetrainCommand::RotateLeft:
			memoryCopy(buffer, DRIVETRAIN_COMMAND_ROTATE_LEFT, MESSAGE_CONTENT_LENGTH_MAX);
			break;
		case DrivetrainCommand::RotateRight:
			memoryCopy(buffer, DRIVETRAIN_COMMAND_ROTATE_RIGHT, MESSAGE_CONTENT_LENGTH_MAX);
			break;
		case DrivetrainCommand::Halt:
			memoryCopy(buffer, DRIVETRAIN_COMMAND_HALT, MESSAGE_CONTENT_LENGTH_MAX);
			break;
		default:
			memorySet(buffer, 0, MESSAGE_CONTENT_LENGTH_MAX);
	}
}

/**
 * @brief Interpret string as drivetrain command.
 * 
 * @param buffer 
 * @return DrivetrainCommand 
 */
static inline DrivetrainCommand getDrivetrainCommandFromString(char* buffer)
{
	if (stringLength(buffer) == 0)
		return DrivetrainCommand::NoReceived;

	if (stringCompare(buffer, DRIVETRAIN_COMMAND_TRANSLATE_FORWARD) == true)
		return DrivetrainCommand::TranslateForward;
	if (stringCompare(buffer, DRIVETRAIN_COMMAND_TRANSLATE_BACKWARD) == true)
		return DrivetrainCommand::TranslateBackward;
	if (stringCompare(buffer, DRIVETRAIN_COMMAND_ROTATE_LEFT) == true)
		return DrivetrainCommand::RotateLeft;
	if (stringCompare(buffer, DRIVETRAIN_COMMAND_ROTATE_RIGHT) == true)
		return DrivetrainCommand::RotateRight;
	if (stringCompare(buffer, DRIVETRAIN_COMMAND_HALT) == true)
		return DrivetrainCommand::Halt;
	
	return DrivetrainCommand::Invalid;
}

/**
 * @brief Convert drivetrain response to string.
 * 
 * @param response 
 * @param buffer 
 */
static inline void getStringFromDrivetrainResponse(DrivetrainResponse response, char* buffer)
{
	switch (response)
	{
		case DrivetrainResponse::AcknowledgeValidCommand:
			memoryCopy(buffer, DRIVETRAIN_RESPONSES_ACKNOWLEDGE_VALID_COMMAND, MESSAGE_CONTENT_LENGTH_MAX);
			break;
		case DrivetrainResponse::AcknowledgeInvalidCommand:
			memoryCopy(buffer, DRIVETRAIN_RESPONSES_ACKNOWLEDGE_INVALID_COMMAND, MESSAGE_CONTENT_LENGTH_MAX);
			break;
		case DrivetrainResponse::NotifyHalting:
			memoryCopy(buffer, DRIVETRAIN_RESPONSES_NOTIFY_HALTING, MESSAGE_CONTENT_LENGTH_MAX);
			break;
		default:
			memorySet(buffer, 0, MESSAGE_CONTENT_LENGTH_MAX);
	}
}

/**
 * @brief Interpret string as drivetrain response.
 * 
 * @param buffer 
 * @return DrivetrainResponse 
 */
static inline DrivetrainResponse getDrivetrainResponseFromString(char* buffer)
{
	if (stringLength(buffer) == 0)
		return DrivetrainResponse::NoReceived;

	if (stringCompare(buffer, DRIVETRAIN_RESPONSES_ACKNOWLEDGE_VALID_COMMAND) == true)
		return DrivetrainResponse::AcknowledgeValidCommand;
	if (stringCompare(buffer, DRIVETRAIN_RESPONSES_ACKNOWLEDGE_INVALID_COMMAND) == true)
		return DrivetrainResponse::AcknowledgeInvalidCommand;
	if (stringCompare(buffer, DRIVETRAIN_RESPONSES_NOTIFY_HALTING) == true)
		return DrivetrainResponse::NotifyHalting;
	
	return DrivetrainResponse::Invalid;
}

// /**
//  * Store all message lookup tables in the same place.
//  *
//  * Each table corresponds to a type of message.
//  * Each mapping relates an Enum of specific type to a short command string.
//  */

// /***************************************************
//  *                DrivetrainCommand                *
//  **************************************************/
// #define DRIVETRAIN_COMMAND_TABLE \
//     CMD(TranslateForward,  "w") \
//     CMD(TranslateBackward, "s") \
//     CMD(RotateLeft,        "a") \
//     CMD(RotateRight,       "d") \
//     CMD(Halt,              "h")

// #define MESSAGE_ENUM_NAME DrivetrainCommand
// #define MESSAGE_ENUM_TABLE DRIVETRAIN_COMMAND_TABLE
// #include "MessageEnumTemplate.h"


// /***************************************************
//  *               DrivetrainResponse                *
//  **************************************************/
// #define DRIVETRAIN_RESPONSE_TABLE \
//     CMD(AcknowledgeValid,	"ackval") \
//     CMD(AcknowledgeInvalid,	"ackinv") \
// 	CMD(NotifyHalting,		"notifhalt")

// #define MESSAGE_ENUM_NAME DrivetrainResponse
// #define MESSAGE_ENUM_TABLE DRIVETRAIN_RESPONSE_TABLE
// #include "MessageEnumTemplate.h"
