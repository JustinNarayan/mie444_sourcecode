#pragma once
#include "Types.h"

/**
 * @brief Define all possible Message types.
 * Every Message object will begin with a char denoting the type of message
 * 
 */
enum class MessageType {
	/* Unused */
	Unused, // encodes to 0

	/* Generic Types */
	Generic,
    Error,

	/* Drivetrain */
	DrivetrainManualCommand,
	DrivetrainManualResponse,
	DrivetrainAutomatedCommand,
	DrivetrainAutomatedResponse,
	DrivetrainEncoderState,
	DrivetrainEncoderDistances,
    DrivetrainDisplacements,
    DrivetrainMotorCommand,

	/* Sensor Readings*/
	LidarState,
	LidarPointReading,
    UltrasonicState,
    UltrasonicPointReading,

    /* Gripper */
    GripperCommand,
    GripperState,

	Count
};

static_assert((uint8_t)MessageType::Count <= UINT_LEAST8_MAX, "MessageType must be uniquely captured in one byte");

/**
 * Wrap list of MessageType values into a MessageTypes object.
 */
template <MessageType... Types>
struct MessageTypes {};
