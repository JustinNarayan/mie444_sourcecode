#pragma once
#include <Arduino.h>

/**
 * Project-wide settings applicable to all boards
 */
#define EXTERNAL_COMMS_BAUD_RATE 57600
#define INTERNAL_COMMS_BAUD_RATE 57600
#define BLUETOOTH_AT_BAUD_RATE 38400

/**
 * Settings for all communications and ring buffer architectures
 */
#define MESSAGE_LENGTH_MAX 64
#define MESSAGE_END_CHAR '$'
#define MESSAGE_NUM_BUFFERS 8

/**
 * Centralized drivetrain control logic settings
 */
#define DRIVETRAIN_TIME_TO_HALT_AFTER_LAST_RECEIVED_COMMAND (1000UL) // millis