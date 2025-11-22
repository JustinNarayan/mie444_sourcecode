#pragma once
#include "Settings.h"
#include "Types.h"
#include <DrivetrainDefs.h>

/*****************************************************
 *                 COMPILER UTILITIES                *
 *****************************************************/
#define ULTRASONIC_TIME_US_TO_INCH(us) M_TO_INCH(us * SPEED_OF_SOUND_DIV2_MPS)


/*****************************************************
 *                       ENUMS                       *
 *****************************************************/

/**
 * Valid messages for Ultrasonic communication
 */
enum class UltrasonicState
{
	Invalid,
	NoReceived,
	Request,
	Rejected,
    InProgress,
	Success,
	Complete,

	Count
};

/**
 * State of Ultrasonic sweep
 * 
 */
enum class UltrasonicSweepState
{
    Idle,
    FirstEncoderReading,
    SendingPoint,
    IssuingStep,
    ConfirmingStep,
    SweepEncoderReading,
    Complete
};

/*****************************************************
 *                      STRUCTS                      *
 *****************************************************/

/**
 * Structure for individual ultrasonic  point readings
 */
struct __attribute__((packed)) UltrasonicPointReading
{
    uint8_t whichUltrasonic;
	DrivetrainEncoderDistances encoders;
	ultrasonicDistance_in distance;
};