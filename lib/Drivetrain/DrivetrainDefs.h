#pragma once
#include "Types.h"

/*****************************************************
 *                       ENUMS                       *
 *****************************************************/

/**
 * Valid commands to be issued to the drivetrain under manual control.
 */
enum class DrivetrainManualCommand
{
	Invalid,
	NoReceived,
	TranslateForward,
	TranslateBackward,
	RotateLeft,
	RotateRight,
	Halt,

	Count
};

/**
 * Valid messages for drivetrain board to issue back under manual control.
 */
enum class DrivetrainManualResponse
{
	Invalid,
	NoReceived,
	AcknowledgeValidCommand,
	AcknowledgeInvalidCommand,
	NotifyHalting,
	
	Count
};

/**
 * Valid messages for drivetrain encoder communication
 * 
 */
enum class DrivetrainEncoderState
{
	Invalid,
	NoReceived,
	RequestFromLocalization,
	RequestFromController,
	Pinged,

	Count
};

/*****************************************************
 *                      STRUCTS                      *
 *****************************************************/

/**
 * Structure for encoder information
 */
struct DrivetrainEncoderDistances
{
	bool isForLocalization;
	float32_t encoder1Dist_in;
	float32_t encoder2Dist_in;
	float32_t encoder3Dist_in;
};
