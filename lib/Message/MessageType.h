#pragma once
#include <stdint.h>

/**
 * @brief Define all possible Message types.
 * Every Message object will begin with a char denoting the type of message
 * 
 */
enum class MessageType {
	/* Default*/
	Empty,
	Generic,
	Error,
	/* Drivetrain */
	DrivetrainCommand,
	DrivetrainResponse,
	DrivetrainEncoder,
	/* Sensor Readings */
	ReadingLidar,

	Count
};
static_assert((uint8_t)MessageType::Count <= UINT_LEAST8_MAX, "Message type must be uniquely captured in one byte");