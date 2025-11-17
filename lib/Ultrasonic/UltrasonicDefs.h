#pragma once
#include "Settings.h"
#include "Types.h"

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
    IssuingStep,
    ConfirmingStep,
    SweepEncoderReading,
    SendingPoint,
    Complete
};

/*****************************************************
 *                      STRUCTS                      *
 *****************************************************/

/**
 * Structure for individual ultrasonic  point readings
 */
struct UltrasonicPointReading
{
	ultrasonicAngle_deg angle;
	ultrasonicDistance_in distance;
};