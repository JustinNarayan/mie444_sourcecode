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
	Request,

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
	float32_t encoder1Dist_in;
	float32_t encoder2Dist_in;
	float32_t encoder3Dist_in;
};

/**
 * Structure for drivetrain displacements
 * 
 */
struct DrivetrainDisplacements
{
	float32_t dX; // directly forward
	float32_t dY; // 90 deg CCW
	float32_t dTheta; // CCW
};

/**
 * Compute drivetrain displacements from encoder information
 * 
 * For the inverse kinematics of Kiwi drive, we take the following:
 * t1 = angle of motor1 = 180 deg
 * t2 = angle of motor2 = 60 deg
 * t3 = angle of motor3 = 300 deg
 * L = distance from pidpoint to wheel center = ~3.569 inches
 *  ---------------------       -----------------------
 * | delta encoder1 (in) |     | delta x (forward)     |
 * | delta encoder2 (in) | = A | delta y (90 deg ccw)  |
 * | delta encoder3 (in) |     | delta theta (+ = ccw) |
 *  ---------------------       -----------------------
 * 
 *           -----------------------     ----------------------
 *          | -sin(t1)   cos(t1)  L |   |      0        1    L |
 * with A = | -sin(t2)   cos(t2)  L | = | -sqrt(3)/2   1/2   L |
 *          | -sin(t3)   cos(t3)  L |   |  sqrt(3)/2   1/2   L |
 *           -----------------------     ----------------------
 * 
 * So to get robot displacements from motor displacements, we need Ainv
 * 
 *          ---------------------------------     --------------------------
 *        |   0      -sqrt(3)/3   sqrt(3)/3 |   | Ainv11   Ainv21   Ainv31 |
 * Ainv = | -2/3         1/3         1/3    | = | Ainv12   Ainv22   Ainv32 |
 *        | 1/(3L)      1/(3L)      1/(3L)  |   | Ainv13   Ainv23   Ainv33 |
 *          ---------------------------------     --------------------------
 */
static const float32_t Ainv[9] = {
  	( 0.000000), (-0.666666), ( 0.093396),
	(-0.577350), ( 0.333333), ( 0.093396),
	( 0.577350), ( 0.333333), ( 0.093396)
};

static inline void displacementsFromEncoderReadings(
	DrivetrainDisplacements *displacements,
	DrivetrainEncoderDistances *encodersBefore,
	DrivetrainEncoderDistances *encodersAfter
)
{
	if (!displacements || !encodersBefore || !encodersAfter)
		return;
	
	// Compute encoder displacements
	float32_t d1 = encodersAfter->encoder1Dist_in - encodersBefore->encoder1Dist_in;
	float32_t d2 = encodersAfter->encoder2Dist_in - encodersBefore->encoder2Dist_in;
	float32_t d3 = encodersAfter->encoder3Dist_in - encodersBefore->encoder3Dist_in;

	// Compute displacements
	displacements->dX = 	(Ainv[0] * d1) + (Ainv[1] * d2) + (Ainv[2] * d3);
	displacements->dY = 	(Ainv[3] * d1) + (Ainv[4] * d2) + (Ainv[5] * d3);
	displacements->dTheta = (Ainv[6] * d1) + (Ainv[7] * d2) + (Ainv[8] * d3);
}