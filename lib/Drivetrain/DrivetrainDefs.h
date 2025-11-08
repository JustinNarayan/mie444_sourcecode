#pragma once
#include "Types.h"

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
 * Structure for encoder information
 */
struct DrivetrainEncoderDistances
{
	float32_t encoder1Dist_cm;
	float32_t encoder2Dist_cm;
	float32_t encoder3Dist_cm;
};
