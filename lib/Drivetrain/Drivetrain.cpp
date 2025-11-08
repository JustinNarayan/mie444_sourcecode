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
int Drivetrain::setTranslate(uint8_t speed, bool isForward)
{
	this->motor1->stop();

	this->motor2->setDirection(!isForward);
	this->motor2->setSpeed(speed);

	this->motor3->setDirection(isForward);
	this->motor3->setSpeed(speed);

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
int Drivetrain::setRotate(uint8_t speed, bool isLeft)
{
	this->motor1->setDirection(false == isLeft);
	this->motor1->setSpeed(speed);

	this->motor2->setDirection(false == isLeft);
	this->motor2->setSpeed(speed);

	this->motor3->setDirection(false == isLeft);
	this->motor3->setSpeed(speed);

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
	this->motor1->stop();
	this->motor2->stop();
	this->motor3->stop();

	// Failure states not yet implemented
	return RET_SET_COMMAND_FAILURE;
}
