#pragma once
#include "Types.h"

/**
 * Project-wide settings applicable to all boards
 */
#define EXTERNAL_COMMS_BAUD_RATE 57600
#define INTERNAL_COMMS_BAUD_RATE 57600
#define BLUETOOTH_AT_BAUD_RATE 38400

/**
 * Global size limit on strings, including null-terminator
 */ 
#define STRING_LENGTH_MAX 64

/**
 * Settings for all communications and ring buffer architectures
 */
#define MESSAGE_END_CHAR '$'
#define MESSAGE_NUM_BUFFERS 8 // ring buffer for raw comms interface
#define MESSAGE_QUEUE_SIZE 4 // max number of stored messages for subsystems
// Each encoded message ends with this token
#define MESSAGE_END_CHAR '$'
// Each message includes encoding data of a (1) type char, (2) size char, and (3) end char
#define MESSAGE_ENCODING_LENGTH (3)
// Type char and size char are at beginning of message
#define MESSAGE_PRE_ENCODE_LENGTH (2)
// Each message allows space for encoding data and a null-terminator
#define MESSAGE_CONTENT_LENGTH_MAX (STRING_LENGTH_MAX - MESSAGE_ENCODING_LENGTH - 1)

/**
 * Centralized drivetrain control logic settings
 */
#define DRIVETRAIN_TIME_TO_HALT_AFTER_LAST_RECEIVED_COMMAND (100UL) // millis
#define DRIVETRAIN_TRANSLATE_SPEED (120) // 0 to 255
#define DRIVETRAIN_ROTATE_SPEED (80) // 0 to 255