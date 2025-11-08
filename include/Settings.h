#pragma once
#include "Types.h"

/*****************************************************
 *                  COMMUNICATIONS                   *
 *****************************************************/

#define EXTERNAL_COMMS_BAUD_RATE 57600
#define INTERNAL_COMMS_BAUD_RATE 57600
#define BLUETOOTH_AT_BAUD_RATE 38400

/*****************************************************
 *                      STRINGS                      *
 *****************************************************/
#define STRING_LENGTH_MAX 64

/*****************************************************
 *                     MESSAGES                      *
 *****************************************************/
/**
 * Break out buffer sizes by board
 */
#if defined(BOARD_CONTROLLER)
#define MESSAGE_NUM_BUFFERS 6 // ring buffer for raw comms interface
#define MESSAGE_QUEUE_SIZE 4 // max number of stored messages for subsystems
#elif defined(BOARD_PERIPHERAL)
#define MESSAGE_NUM_BUFFERS 2 // ring buffer for raw comms interface
#define MESSAGE_QUEUE_SIZE 2 // max number of stored messages for subsystems
#else
#error "Unsupported board! Please defined BOARD_xxx in platformio.ini"
#endif
// Each encoded message ends with this token
#define MESSAGE_END_CHAR '$'
// Each message includes encoding data of a (1) type char, (2) size char, and (3) end char
#define MESSAGE_ENCODING_LENGTH (3)
// Type char and size char are at beginning of message
#define MESSAGE_PRE_ENCODE_LENGTH (2)
// Each message allows space for encoding data and a null-terminator
#define MESSAGE_CONTENT_LENGTH_MAX (STRING_LENGTH_MAX - MESSAGE_ENCODING_LENGTH - 1)

/*****************************************************
 *                   DRIVETRAIN                      *
 *****************************************************/
/**
 * @brief Drivetrain encoder custom parameters
 * 
 */
#define ENCODER_1_GAIN_TO_CM ((float64_t)1.0)
#define ENCODER_2_GAIN_TO_CM ((float64_t)1.0)
#define ENCODER_3_GAIN_TO_CM ((float64_t)1.0)
#define ENCODER_TIME_TO_SEND_AFTER_LAST_SENT_DISTANCES (100UL) // millis

/**
 * @brief Drivetrain control custom parameters
 * 
 */
#define DRIVETRAIN_TIME_TO_HALT_AFTER_LAST_RECEIVED_COMMAND (100UL) // millis
#define DRIVETRAIN_TRANSLATE_SPEED (120) // 0 to 255
#define DRIVETRAIN_ROTATE_SPEED (80) // 0 to 255