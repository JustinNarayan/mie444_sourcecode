#pragma once
#include "MemoryUtilities.h"
#include "Settings.h"
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
	Automated,

	Count
};

/**
 * Valid messages for drivetrain board to issue back under manual control.
 */
enum class DrivetrainManualResponse
{
	Invalid,
	NoReceived,
	Acknowledge,
	NotifyHalting,
	
	Count
};

/**
 * Valid messages for drivetrain board to issue back under automated control
 */
enum class DrivetrainAutomatedResponse
{
	Invalid,
	NoReceived,
	Acknowledge,
	InProgress,
	Failure,
	Success,
	Aborted,

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
	encoderDistance_in encoder1Dist;
	encoderDistance_in encoder2Dist;
	encoderDistance_in encoder3Dist;
};

/**
 * Structure for drivetrain displacements
 * 
 */
struct DrivetrainDisplacements
{
	float32_t dX_in; // directly forward
	float32_t dY_in; // 90 deg CCW
	float32_t dTheta_rad; // CCW
};

/**
 * Structure for drivetrain automated commands
 * 
 */
struct DrivetrainAutomatedCommand
{
	int16_t dX_in;
    int16_t dY_in;
    int16_t dTheta_deg;
};

/**
 * Structure for storing the direction of drivetrain automated commands, for overshoot checking
 * 
 */
struct DrivetrainCommandDirection
{
	bool increase1ToTarget;
	bool increase2ToTarget;
	bool increase3ToTarget;
};

/**
 * Structure for motor commands on all three wheels to drivetrain
 */
struct DrivetrainMotorCommand

{
	motorDirection direction1;
	motorSpeedRaw speed1;
	motorDirection direction2;
	motorSpeedRaw speed2;
	motorDirection direction3;
	motorSpeedRaw speed3;
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
static const float32_t A[9] = {
	( 0.000000), ( 1.000000), ( 3.569000),
	(-0.866025), ( 0.500000), ( 3.569000),
	( 0.866025), ( 0.500000), ( 3.569000),
};

static const float32_t Ainv[9] = {
    ( 0.000000), (-0.577350), ( 0.577350),
    (-0.666666), ( 0.333333), ( 0.333333),
    ( 0.093396), ( 0.093396), ( 0.093396)
};

/**
 * @brief Convert a drivetrain command into a set of displacements as floats
 * 
 * @param displacements Updated after call
 * @param command 
 */
static inline void displacementsFromDrivetrainCommand(
	DrivetrainDisplacements *displacements,
    DrivetrainAutomatedCommand* command
)
{
    displacements->dX_in = (float32_t)command->dX_in;
    displacements->dY_in = (float32_t)command->dY_in;
    displacements->dTheta_rad = DEG_TO_RAD * (float32_t)command->dTheta_deg;
}


/**
 * @brief Convert a set of drivetrain displacement into a command 
 * 
 * @param command  Updated after call
 * @param displacements
 */
static inline void drivetrainCommandFromDisplacements(
    DrivetrainAutomatedCommand *command,
	DrivetrainDisplacements *displacements
)
{
    command->dX_in = (int16_t)displacements->dX_in;
    command->dY_in = (int16_t)displacements->dY_in;
    command->dTheta_deg = (int16_t)(RAD_TO_DEG * displacements->dTheta_rad);
}

/**
 * @brief Inverse kinematics. Get drivetrain displacement from encoder readings.
 * 
 * @param displacements Updated after call
 * @param encodersBefore 
 * @param encodersAfter 
 */
static inline void displacementsFromEncoderReadings(
	DrivetrainDisplacements *displacements,
	DrivetrainEncoderDistances *encodersBefore, 
	DrivetrainEncoderDistances *encodersAfter
)
{
	if (!displacements || !encodersBefore || !encodersAfter)
		return;
	
	// Compute encoder displacements
	const encoderDistance_in d1 = encodersAfter->encoder1Dist - encodersBefore->encoder1Dist;
	const encoderDistance_in d2 = encodersAfter->encoder2Dist - encodersBefore->encoder2Dist;
	const encoderDistance_in d3 = encodersAfter->encoder3Dist - encodersBefore->encoder3Dist;

	// Compute displacements
	displacements->dX_in      = (Ainv[0] * d1) + (Ainv[1] * d2) + (Ainv[2] * d3);
	displacements->dY_in      = (Ainv[3] * d1) + (Ainv[4] * d2) + (Ainv[5] * d3);
	displacements->dTheta_rad = (Ainv[6] * d1) + (Ainv[7] * d2) + (Ainv[8] * d3);
}

/**
 * @brief Forward kinematics. Get updated encoder readings after displacement.
 * 
 * @param displacements
 * @param encodersBefore 
 * @param encodersAfter Updated after call
 */
static inline void encoderReadingsFromDisplacement(
	DrivetrainDisplacements *displacements,
	DrivetrainEncoderDistances *encodersBefore,
	DrivetrainEncoderDistances *encodersAfter
)
{
	if (!displacements || !encodersBefore || !encodersAfter)
		return;

	// Compute displacements
	const float32_t d1 = \
		(A[0] * displacements->dX_in) + 
        (A[1] * displacements->dY_in) + 
        (A[2] * displacements->dTheta_rad);
	const float32_t d2 = \
		(A[3] * displacements->dX_in) + 
        (A[4] * displacements->dY_in) + 
        (A[5] * displacements->dTheta_rad);
	const float32_t d3 = \
		(A[6] * displacements->dX_in) + 
        (A[7] * displacements->dY_in) +
        (A[8] * displacements->dTheta_rad);

	// Compute encoder readings
	encodersAfter->encoder1Dist = encodersBefore->encoder1Dist + d1;
	encodersAfter->encoder2Dist = encodersBefore->encoder2Dist + d2;
	encodersAfter->encoder3Dist = encodersBefore->encoder3Dist + d3;
}

/**
 * @brief Compute the direction of a command from the desired and current encoder value
 * 
 * @param encodersStart
 * @param encodersTarget
 * @param direction Updated after call
 */
static inline void commandDirectionFromEncoderReadings(
	DrivetrainEncoderDistances *encodersStart,
	DrivetrainEncoderDistances *encodersTarget,
	DrivetrainCommandDirection *direction
)
{
	direction->increase1ToTarget = \
		(encodersTarget->encoder1Dist > encodersStart->encoder1Dist);
	direction->increase2ToTarget = \
		(encodersTarget->encoder2Dist > encodersStart->encoder2Dist);
	direction->increase3ToTarget = \
		(encodersTarget->encoder3Dist > encodersStart->encoder3Dist);
}

/**
 * @brief Stopping measure. Determine when the encoders have reached the desired destination.
 * 
 * @param encodersStart
 * @param encodersNow 
 * @param encodersTarget
 * 
 * @return DrivetrainAutomatedResponse
 *  InProgress if not yet at target
 *  Success if at target (all values within threshold)
 *  Failure if not at target but must stop (any value over threshold)
 */
static inline DrivetrainAutomatedResponse areEncodersAtTarget(
	DrivetrainCommandDirection *direction,
	DrivetrainEncoderDistances *encodersNow,
	DrivetrainEncoderDistances *encodersTarget
)
{
	// Store all encoder values as constants for optimized dereferencing    
	const float32_t n1 = encodersNow->encoder1Dist;
	const float32_t n2 = encodersNow->encoder2Dist;
	const float32_t n3 = encodersNow->encoder3Dist;
	
	const float32_t t1 = encodersTarget->encoder1Dist;
	const float32_t t2 = encodersTarget->encoder2Dist;
	const float32_t t3 = encodersTarget->encoder3Dist;

	// Compute delta from target
	const float32_t d1 = t1 - n1;
	const float32_t d2 = t2 - n2;
	const float32_t d3 = t3 - n3;

	// Compute if at target
	const bool t1Near = fabsf(d1) < DRIVETRAIN_ENCODERS_EQUAL_TOLERANCE_THRESHOLD;
	const bool t2Near = fabsf(d2) < DRIVETRAIN_ENCODERS_EQUAL_TOLERANCE_THRESHOLD;
	const bool t3Near = fabsf(d3) < DRIVETRAIN_ENCODERS_EQUAL_TOLERANCE_THRESHOLD;

	// Succeed if all motors at target
	if (t1Near && t2Near && t3Near) return DrivetrainAutomatedResponse::Success;

	// Determine if any motors are yet to pass target
	// bool undershoot1 = direction->increase1ToTarget ? (d1 > 0) : (d1 < 0);
	// bool undershoot2 = direction->increase2ToTarget ? (d2 > 0) : (d2 < 0);
	// bool undershoot3 = direction->increase3ToTarget ? (d3 > 0) : (d3 < 0);

	// Fail if any motor has overshot target and no longer near
	// if (
	// 	(!t1Near && !undershoot1) ||
	// 	(!t2Near && !undershoot2) ||
	// 	(!t3Near && !undershoot3)
	// ) return DrivetrainAutomatedResponse::Failure;

	// Otherwise, return in progress
	return DrivetrainAutomatedResponse::InProgress;
}

/**
 * @brief Forward kinematics. Get motor command from displacement. Limit motor commands
 * 	to a specific maximum value.
 * 
 * @param displacements
 * @param motorCommand // Updated after call
 */
static inline void motorCommandFromDisplacement(
	DrivetrainDisplacements *displacements,
	DrivetrainMotorCommand *motorCommand,
	motorSpeedRaw limitLow,
    motorSpeedRaw limitHigh
)
{
	if (!displacements || !motorCommand)
		return;

	// Compute displacements
	const float32_t d1 = \
		(A[0] * displacements->dX_in) + 
        (A[1] * displacements->dY_in) + 
        (A[2] * displacements->dTheta_rad);
	const float32_t d2 = \
		(A[3] * displacements->dX_in) + 
        (A[4] * displacements->dY_in) + 
        (A[5] * displacements->dTheta_rad);
	const float32_t d3 = \
		(A[6] * displacements->dX_in) + 
        (A[7] * displacements->dY_in) + 
        (A[8] * displacements->dTheta_rad);

	// Assign raw motor commands
	motorCommand->direction1 = d1 > 0;
	motorCommand->speed1 = fabsf(d1);
	motorCommand->direction2 = d2 > 0;
	motorCommand->speed2 = fabsf(d2);
	motorCommand->direction3 = d3 > 0;
	motorCommand->speed3 = fabsf(d3);

    // Check deadbands and find smallest nonzero motor speed
    motorSpeedRaw lowestSpeed = limitHigh;
    if (motorCommand->speed1 < DRIVETRAIN_AUTOMATED_DEADBAND) motorCommand->speed1 = 0;
    else lowestSpeed = min(lowestSpeed, motorCommand->speed1);
    if (motorCommand->speed2 < DRIVETRAIN_AUTOMATED_DEADBAND) motorCommand->speed2 = 0;
    else lowestSpeed = min(lowestSpeed, motorCommand->speed2);
    if (motorCommand->speed3 < DRIVETRAIN_AUTOMATED_DEADBAND) motorCommand->speed3 = 0;
    else lowestSpeed = min(lowestSpeed, motorCommand->speed3);

    // Scale commands up if any below lowest limit
    if (lowestSpeed < limitLow)
    {
		motorCommand->speed1 *= (limitLow / lowestSpeed);
		motorCommand->speed2 *= (limitLow / lowestSpeed);
		motorCommand->speed3 *= (limitLow / lowestSpeed);
    }
	
	// Determine motor command limits
	motorSpeedRaw absMotorCommandMax = max(
		max(motorCommand->speed1, motorCommand->speed2), motorCommand->speed3
    );

    // Enforce command limit not violated
	if (absMotorCommandMax > limitHigh)
	{
		motorCommand->speed1 *= (limitHigh / absMotorCommandMax);
		motorCommand->speed2 *= (limitHigh / absMotorCommandMax);
		motorCommand->speed3 *= (limitHigh / absMotorCommandMax);
	}
}

/**
 * @brief Compute the drivetrain motor speed with a slew limit.
 * 
 * @param rawSpeed
 * @param rawPrev
 */
static inline motorSpeedRaw applySlewLimit(motorSpeedRaw rawSpeed, motorSpeedRaw rawPrev)
{
	motorSpeedRaw delta = constrain(
		rawSpeed - rawPrev, 
		DRIVETRAIN_AUTOMATED_MAX_SLEW_DOWN, 
		DRIVETRAIN_AUTOMATED_MAX_SLEW_UP
	);
	return rawPrev + delta;
}

/**
 * @brief Compute the drivetrain motor command from the encoder values now and at target.
 * 
 * @param encodersNow
 * @param encodersTarget
 * @param command // updated after call
 */
static inline void getDrivetrainMotorCommand(
	DrivetrainEncoderDistances *encodersNow,
	DrivetrainEncoderDistances *encodersTarget,
	DrivetrainMotorCommand *command,
    bool isFirstCommand = true // Resets integral control
)
{
	// Compute inverse kinematics
	DrivetrainDisplacements displacements;
	displacementsFromEncoderReadings(
		&displacements,
		encodersNow,
		encodersTarget
	);

    // Update control variables
    static time_ms lastMillis = millis();
    static DrivetrainDisplacements integratedDisplacements = {0};
    static DrivetrainDisplacements lastDisplacements = {0};
    static DrivetrainDisplacements differentiatedDisplacements = {0};
	static DrivetrainMotorCommand prevCommand = {0};
    if (isFirstCommand)
    {
        lastMillis = millis();
        integratedDisplacements = {0};
        lastDisplacements = {0};
        prevCommand = {0};
    }
    else
    {
        // Compute time step
        time_ms now = millis();
        time_s dt = MS_TO_S(now - lastMillis);

        // Update integral and derivative terms if enough time elapsed
        if (dt < DRIVETRAIN_MINIMUM_DT_PID)
        {
            differentiatedDisplacements = {0};
        }
        else
        {
            // Integral
            integratedDisplacements.dX_in = constrain(
                integratedDisplacements.dX_in + displacements.dX_in * dt,
                -DRIVETRAIN_MAXIMUM_INTEGRAL, 
                DRIVETRAIN_MAXIMUM_INTEGRAL
            );
            integratedDisplacements.dY_in = constrain(
                integratedDisplacements.dY_in + displacements.dY_in * dt,
                -DRIVETRAIN_MAXIMUM_INTEGRAL, 
                DRIVETRAIN_MAXIMUM_INTEGRAL
            );
            integratedDisplacements.dTheta_rad = constrain(
                integratedDisplacements.dTheta_rad + displacements.dTheta_rad * dt,
                -DRIVETRAIN_MAXIMUM_INTEGRAL, 
                DRIVETRAIN_MAXIMUM_INTEGRAL
            );

            // Derivative
            differentiatedDisplacements.dX_in = \
                (displacements.dX_in - lastDisplacements.dX_in) / dt;
            differentiatedDisplacements.dY_in = \
                (displacements.dY_in - lastDisplacements.dY_in) / dt;
            differentiatedDisplacements.dTheta_rad = \
                (displacements.dTheta_rad - lastDisplacements.dTheta_rad) / dt;

            // Update static
            memoryCopy(&lastDisplacements, &displacements, sizeof(DrivetrainDisplacements));
            lastMillis = now;
        }
    }
	
	// Apply PID gains
    DrivetrainDisplacements controlDisplacements;
	controlDisplacements.dX_in = \
        (displacements.dX_in * DRIVETRAIN_AUTOMATED_GAIN_KP_DX) +
        (integratedDisplacements.dX_in * DRIVETRAIN_AUTOMATED_GAIN_KI_DX)
         + (differentiatedDisplacements.dX_in * DRIVETRAIN_AUTOMATED_GAIN_KD_DX);
	controlDisplacements.dY_in = \
        (displacements.dY_in * DRIVETRAIN_AUTOMATED_GAIN_KP_DY) +
        (integratedDisplacements.dY_in * DRIVETRAIN_AUTOMATED_GAIN_KI_DY)
         + (differentiatedDisplacements.dY_in * DRIVETRAIN_AUTOMATED_GAIN_KD_DY);
	controlDisplacements.dTheta_rad = \
        (displacements.dTheta_rad * DRIVETRAIN_AUTOMATED_GAIN_KP_DTHETA) +
        (integratedDisplacements.dTheta_rad * DRIVETRAIN_AUTOMATED_GAIN_KI_DTHETA)
         + (differentiatedDisplacements.dTheta_rad * DRIVETRAIN_AUTOMATED_GAIN_KD_DTHETA);
	
	// Compute forward kinematics after gain
	motorCommandFromDisplacement(
		&controlDisplacements,
		command,
        (motorSpeedRaw)DRIVETRAIN_AUTOMATED_MIN_SPEED,
		(motorSpeedRaw)DRIVETRAIN_AUTOMATED_MAX_SPEED
	);
	
	// Apply slew
	command->speed1 = applySlewLimit(command->speed1, prevCommand.speed1);
	command->speed2 = applySlewLimit(command->speed2, prevCommand.speed2);
	command->speed3 = applySlewLimit(command->speed3, prevCommand.speed3);
	
	// Store previous command	
	memoryCopy(&prevCommand, command, sizeof(DrivetrainMotorCommand));
}
