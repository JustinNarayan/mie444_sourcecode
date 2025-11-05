#pragma once

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
