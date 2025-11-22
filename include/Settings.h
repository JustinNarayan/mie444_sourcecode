#pragma once
#include "Types.h"

/*****************************************************
 *                  COMMUNICATIONS                   *
 *****************************************************/

#define EXTERNAL_COMMS_BAUD_RATE 57600
#define INTERNAL_COMMS_BAUD_RATE 57600
#define BLUETOOTH_AT_BAUD_RATE 38400
#define LIDAR_BAUD_RATE 115200

/*****************************************************
 *                      STRINGS                      *
 *****************************************************/
/**
 * Break out buffer sizes by board
 */
#if defined(BOARD_CONTROLLER)
#define STRING_LENGTH_MAX 36
#elif defined(BOARD_PERIPHERAL)
#define STRING_LENGTH_MAX 24
#else
#error "Unsupported board! Please defined BOARD_xxx in platformio.ini"
#endif

/*****************************************************
 *                     MESSAGES                      *
 *****************************************************/
/**
 * Break out buffer sizes by board
 */
#if defined(BOARD_CONTROLLER)
#define MESSAGE_NUM_BUFFERS 3 // ring buffer for raw comms interface
#define MESSAGE_QUEUE_SIZE 3 // max number of stored messages for subsystems
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
 * @brief Drivetrain dimensions
 * 
 */
#define DISTANCE_CENTER_TO_WHEEL_MIDPOINT_IN (3.569)
#define WHEEL_DIAMETER_IN (2.20472)
#define WHEEL_CIRCUMFERENCE_IN (2 * PI * WHEEL_DIAMETER_IN)

/**
 * @brief Drivetrain encoder custom parameters
 * 
 */
#define DEFAULT_ENCODER_TICKS_PER_ROTATION (1000) // approx
#define DEFAULT_ENCODER_GAIN_IN ((WHEEL_CIRCUMFERENCE_IN) / (DEFAULT_ENCODER_TICKS_PER_ROTATION))
#define ENCODER_1_TUNING_GAIN ((float64_t)0.485)
#define ENCODER_2_TUNING_GAIN ((float64_t)0.485)
#define ENCODER_3_TUNING_GAIN ((float64_t)1.045)

#define ENCODER_1_TO_IN (DEFAULT_ENCODER_GAIN_IN * ENCODER_1_TUNING_GAIN)
#define ENCODER_2_TO_IN (DEFAULT_ENCODER_GAIN_IN * ENCODER_2_TUNING_GAIN)
#define ENCODER_3_TO_IN (DEFAULT_ENCODER_GAIN_IN * ENCODER_3_TUNING_GAIN)

#define ENCODER_WILL_VOLUNTEER_READINGS (false)
#define ENCODER_TIME_TO_SEND_AFTER_LAST_SENT_DISTANCES (250UL) // millis

/**
 * @brief Drivetrain control custom parameters
 * 
 */
#define DRIVETRAIN_MANUAL_COMMAND_FORWARDING_TIME_TO_DISCARD (100UL) // millis

#define DRIVETRAIN_TIME_TO_HALT_AFTER_LAST_RECEIVED_COMMAND (100UL) // millis
#define DRIVETRAIN_TRANSLATE_SPEED (150) // 0 to 255
#define DRIVETRAIN_ROTATE_SPEED (110) // 0 to 255
#define DRIVETRAIN_BRAKE_SPEED (180) // 0 to 255, holding torque
#define DRIVETRAIN_BRAKE_TIME_BEFORE_HALT_VOLUNTARY (50UL) // millis 

#define MOTOR_BROWNOUT_DIRECTION_TIMEOUT_US (1000) // micros
#define MOTOR_1_EMPIRICAL_GAIN(s) ((1.40f * s))
#define MOTOR_2_EMPIRICAL_GAIN(s) ((1.0f * s))
#define MOTOR_3_EMPIRICAL_GAIN(s) ((1.65f * s))

#define DRIVETRAIN_ENCODERS_EQUAL_TOLERANCE_THRESHOLD (0.35) // inches

/**
 * @brief Drivetrain automation custom parameters
 * 
 */
#define DRIVETRAIN_AUTOMATED_COMMAND_FORWARDING_TIME_TO_DISCARD (500UL) // millis

#define DRIVETRAIN_AUTOMATED_MIN_DELTA_X (2) // inches
#define DRIVETRAIN_AUTOMATED_MIN_DELTA_Y (2) // inches
#define DRIVETRAIN_AUTOMATED_MIN_DELTA_THETA (PI/8) // radians
#define DRIVETRAIN_AUTOMATED_COMMAND_MAX_TIME (3000UL) // millis

#define DRIVETRAIN_AUTOMATED_MAX_SPEED (170) // 0 to 255
#define DRIVETRAIN_AUTOMATED_MIN_SPEED (80) // 0 to 255
#define DRIVETRAIN_AUTOMATED_MAX_SLEW_UP (50) // delta speed to motor
#define DRIVETRAIN_AUTOMATED_MAX_SLEW_DOWN (-50) // delta speed to motor
#define DRIVETRAIN_AUTOMATED_DEADBAND (12)
#define DRIVETRAIN_MINIMUM_TIME_BETWEEN_CONTROL_ADJUSTEMENTS (10) // millis
#define DRIVETRAIN_AUTOMATED_POST_COMMAND_DISPLACEMENT_WAIT (50) // millis

#define DRIVETRAIN_AUTOMATED_APPLY_GAINS_TO_DISPLACEMENTS (false)

#if DRIVETRAIN_AUTOMATED_APPLY_GAINS_TO_DISPLACEMENTS
# define DRIVETRAIN_AUTOMATED_GAIN_KP_DX (10)
# define DRIVETRAIN_AUTOMATED_GAIN_KI_DX (15)
# define DRIVETRAIN_AUTOMATED_GAIN_KD_DX (5)
# define DRIVETRAIN_AUTOMATED_GAIN_KP_DY (10)
# define DRIVETRAIN_AUTOMATED_GAIN_KI_DY (15)
# define DRIVETRAIN_AUTOMATED_GAIN_KD_DY (5)
# define DRIVETRAIN_AUTOMATED_GAIN_KP_DTHETA (300)
# define DRIVETRAIN_AUTOMATED_GAIN_KI_DTHETA (40)
# define DRIVETRAIN_AUTOMATED_GAIN_KD_DTHETA (20)

# define DRIVETRAIN_MAXIMUM_INTEGRAL (20)
# define DRIVETRAIN_MAXIMUM_INTEGRAL_THETA (10)
# define DRIVETRAIN_MAXIMUM_DERIVATIVE (20)
# define DRIVETRAIN_MINIMUM_DTHETA_FOR_INTEGRAL (0.25) // rad ~ 15 deg
#else
# define DRIVETRAIN_BACKWARDS_ADDED_GAIN_MOTOR_1 (1.15)
# define DRIVETRAIN_BACKWARDS_ADDED_GAIN_MOTOR_2 (1.15)
# define DRIVETRAIN_BACKWARDS_ADDED_GAIN_MOTOR_3 (1.4)

# define DRIVETRAIN_AUTOMATED_GAIN_KP_MOTOR_1 (50)
# define DRIVETRAIN_AUTOMATED_GAIN_KI_MOTOR_1 (23)
# define DRIVETRAIN_AUTOMATED_GAIN_KD_MOTOR_1 (40)

# define DRIVETRAIN_AUTOMATED_GAIN_KP_MOTOR_2 (25)
# define DRIVETRAIN_AUTOMATED_GAIN_KI_MOTOR_2 (10)
# define DRIVETRAIN_AUTOMATED_GAIN_KD_MOTOR_2 (20)

# define DRIVETRAIN_AUTOMATED_GAIN_KP_MOTOR_3 (48)
# define DRIVETRAIN_AUTOMATED_GAIN_KI_MOTOR_3 (16)
# define DRIVETRAIN_AUTOMATED_GAIN_KD_MOTOR_3 (32)

# define DRIVETRAIN_MAXIMUM_PROPORTIONAL_MOTORS (2)
# define DRIVETRAIN_MAXIMUM_INTEGRAL_MOTORS (5)
# define DRIVETRAIN_MAXIMUM_DERIVATIVE_MOTORS (0.5)
# define DRIVETRAIN_ALLOWED_OVERSHOOT (1.5)
#endif

#define DRIVETRAIN_MINIMUM_DT_PID (0.05) // in seconds
#define DRIVETRAIN_HOLD_AT_TARGET_TIME (500) // millis

#define DRIVETRAIN_WILL_VOLUNTEER_AUTOMATED_DISPLACEMENTS (true)
#define DRIVETRAIN_WILL_VOLUNTEER_AUTOMATED_COMMANDS (true)
#define DRIVETRAIN_WILL_USE_SIMPLE_AUTOMATED_INTERFACE (false)
#define DRIVETRAIN_TRANSLATE_SPEED_AUTOMATED (160)
#define DRIVETRAIN_STRAFE_SPEED_AUTOMATED (140)
#define DRIVETRAIN_ROTATE_SPEED_AUTOMATED (120)

/*****************************************************
 *                       LIDAR                       *
 *****************************************************/
/**
 * @brief LiDAR control custom parameters
 */
#define LIDAR_MOTOR_SPIN_SPEED (200) // 0 - 255
#define LIDAR_MIN_TIME_SINCE_HALT_RECEIVED_COMMAND (200UL) // millis
#define LIDAR_MAX_TIME_TO_SEND_READING (3000UL) // millis, after this reading is assumed lost

/**
 * @brief LiDAR preprocessing custom parameters
 * 
 */
#define LIDAR_MINIMUM_QUALITY_TO_SEND (0) // 0 - 63, < 15 is noisy
#define LIDAR_CHECK_HEALTH_PERIOD_MS (100UL) // millis
#define LIDAR_GRANULARITY_NUM_POINTS (360) // X even samples across 360 degree sweep
#define LIDAR_SWEEP_STARTUP_MS (500UL) // millis
#define LIDAR_SWEEP_TIMEOUT_MS (1000UL) // millis
#define LIDAR_RESET_TIME_MS (2000UL) // millis

/*****************************************************
 *                     ULTRASONIC                    *
 *****************************************************/

#define ULTRASONIC_TIME_TRIGGER_RESET_MS (2) // millis
#define ULTRASONIC_TIME_TRIGGER_HIGH_US (10) // micros
#define SPEED_OF_SOUND_DIV2_MPS (170) // meters/second
#define ULTRASONIC_SWEEP_INCREMENTS_DEG (15) // degrres
#define ULTRASONIC_SWEEP_MAX_TIME_MS (10000) // millis
#define ULTRASONIC_SWEEP_MINIMUM_ENCODER_DISTANCE (23) // inches
#define ULTRASONIC_TIME_TO_PING_ENCODERS_REPEATEDLY (50) // millis
#define ULTRASONIC_TIME_TO_IGNORE_AUTOMATED_COMMAND_RESPONSE (3000) // millis