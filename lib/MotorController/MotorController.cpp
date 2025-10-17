#include "MotorController.h"

/**
 * @brief Set pin modes for Digital in1/in2 pins and PWM enable pin.
 * 
 */
void MotorController::init()
{
	pinMode(_enablePin, OUTPUT);
	pinMode(_in1Pin, OUTPUT);
	pinMode(_in2Pin, OUTPUT);
	stop();
}

/**
 * @brief Set motor speed via PWM enable pin
 * 
 * @param speed From 0 to 255
 */
void MotorController::setSpeed(uint8_t speed)
{
	_speed = constrain(speed, SPEED_MIN, SPEED_MAX);
	analogWrite(_enablePin, _speed);
}

/**
 * @brief Set motor direction by arbitrary convention
 * 
 * @param isForward If forward in arbitrary convention
 */
void MotorController::setDirection(bool isForward)
{
	if (isForward)
	{
		digitalWrite(_in1Pin, HIGH);
		digitalWrite(_in2Pin, LOW);
	}
	else
	{
		digitalWrite(_in1Pin, LOW);
		digitalWrite(_in2Pin, HIGH);
	}
}

/**
 * @brief Emergency stop for motor
 * 
 */
void MotorController::stop()
{
	digitalWrite(_in1Pin, LOW);
	digitalWrite(_in2Pin, LOW);
	analogWrite(_enablePin, 0);
}
