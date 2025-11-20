#include "Drivetrain.h"

/**
 * @brief Initialize all motor controllers
 * 
 */
void Drivetrain::init(
	uint8_t motor1_enable, uint8_t motor1_in1, uint8_t motor1_in2,
	uint8_t motor2_enable, uint8_t motor2_in1, uint8_t motor2_in2,
	uint8_t motor3_enable, uint8_t motor3_in1, uint8_t motor3_in2
)
{
	this->motor1 = new MotorController(motor1_enable, motor1_in1, motor1_in2);
	this->motor2 = new MotorController(motor2_enable, motor2_in1, motor2_in2);
	this->motor3 = new MotorController(motor3_enable, motor3_in1, motor3_in2);
}

/**
 * @brief Set the forward-facing motor to zero-speed and set the other motors to equal speed
 * 
 * @param speed 0 to 255
 * @param isForward
 * @return RET_SET_COMMAND_SUCCESS if successful
 * 			RET_SET_COMMAND_FAILURE otherwise
 */
int Drivetrain::setTranslate(float32_t rawSpeed, bool isForward)
{
	this->motor1->setBrake();
	this->motor1->setSpeed(DRIVETRAIN_BRAKE_SPEED);

	this->motor2->setDirection(!isForward);
	this->motor2->setSpeed(MOTOR_2_EMPIRICAL_GAIN(rawSpeed));

	this->motor3->setDirection(isForward);
	this->motor3->setSpeed(MOTOR_3_EMPIRICAL_GAIN(rawSpeed));

	// Failure states not yet implemented
	return RET_SET_COMMAND_SUCCESS;
}

/**
 * @brief Set all motor to move in the same direction with equal speed
 * 
 * @param speed 0 to 255
 * @param isLeft 
 * @return RET_SET_COMMAND_SUCCESS if successful
 * 			RET_SET_COMMAND_FAILURE otherwise
 */
int Drivetrain::setRotate(float32_t rawSpeed, bool isLeft)
{
	this->motor1->setDirection(!isLeft);
	this->motor1->setSpeed(MOTOR_1_EMPIRICAL_GAIN(rawSpeed));

	this->motor2->setDirection(!isLeft);
	this->motor2->setSpeed(MOTOR_2_EMPIRICAL_GAIN(rawSpeed));

	this->motor3->setDirection(!isLeft);
	this->motor3->setSpeed(MOTOR_3_EMPIRICAL_GAIN(rawSpeed));

	// Failure states not yet implemented
	return RET_SET_COMMAND_SUCCESS;
}

/**
 * @brief Set the flat motor to move and the other motors to rotate
 * 
 * @param speed 0 to 255
 * @param isLeft 
 * @return RET_SET_COMMAND_SUCCESS if successful
 * 			RET_SET_COMMAND_FAILURE otherwise
 */
int Drivetrain::setStrafe(float32_t rawSpeed, bool isLeft)
{
	this->motor1->setDirection(isLeft);
	this->motor1->setSpeed(MOTOR_1_EMPIRICAL_GAIN(rawSpeed));

	this->motor2->setDirection(!isLeft);
	this->motor2->setSpeed(MOTOR_2_EMPIRICAL_GAIN(rawSpeed / 2));

	this->motor3->setDirection(!isLeft);
	this->motor3->setSpeed(MOTOR_3_EMPIRICAL_GAIN(rawSpeed / 2));

	// Failure states not yet implemented
	return RET_SET_COMMAND_SUCCESS;
}

/**
 * @brief Set motors to move at a requested speed and direction
 * 
 * @param command // Apply raw values
 * @return RET_SET_COMMAND_SUCCESS if successful
 * 			RET_SET_COMMAND_FAILURE otherwise
 */
int Drivetrain::setMotors(DrivetrainMotorCommand *command)
{
	this->motor1->setDirection(command->is1Forward);
	this->motor1->setSpeed(command->speed1);

	this->motor2->setDirection(command->is2Forward);
	this->motor2->setSpeed(command->speed2);

	this->motor3->setDirection(command->is3Forward);
	this->motor3->setSpeed(command->speed3);

	// Failure states not yet implemented
	return RET_SET_COMMAND_SUCCESS;
	
}

/**
 * @brief Set vehicle to brake
 * 
 * @return RET_SET_COMMAND_SUCCESS if successful
 * 			RET_SET_COMMAND_FAILURE otherwise
 */
int Drivetrain::setBrake(void)
{
	this->motor1->setBrake();
	this->motor1->setSpeed(DRIVETRAIN_BRAKE_SPEED);

	this->motor2->setBrake();
	this->motor2->setSpeed(DRIVETRAIN_BRAKE_SPEED);

	this->motor3->setBrake();
	this->motor3->setSpeed(DRIVETRAIN_BRAKE_SPEED);

	// Failure states not yet implemented
	return RET_SET_COMMAND_SUCCESS;
}

/**
 * @brief Halt all motors
 * 
 * @return RET_HALT_COMMAND_SUCCESS if successful
 * 			RET_HALT_COMMAND_FAILURE otherwise
 */
int Drivetrain::halt(void)
{
	this->motor1->coast();
	this->motor1->setSpeed(0);

	this->motor2->coast();
	this->motor2->setSpeed(0);

	this->motor3->coast();
	this->motor3->setSpeed(0);

	// Failure states not yet implemented
	return RET_HALT_COMMAND_SUCCESS;
}
