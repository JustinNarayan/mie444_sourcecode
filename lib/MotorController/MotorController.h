#pragma once
#include "Types.h"

#define SPEED_MIN 0
#define SPEED_MAX 255

/**
 * Motor Controller driver using L298N
 */
class MotorController
{
private:
	uint8_t _enablePin;
	uint8_t _in1Pin;
	uint8_t _in2Pin;
	uint8_t _speed;

public:
	/**
	 * @brief Construct a new Motor Controller
	 * 
	 * @param enablePin PWM
	 * @param in1Pin Digital
	 * @param in2Pin Digital
	 */
	MotorController(uint8_t enablePin, uint8_t in1Pin, uint8_t in2Pin) 
		: _enablePin(enablePin), _in1Pin(in1Pin), _in2Pin(in2Pin), _speed(0)
	{
		pinMode(_enablePin, OUTPUT);
		pinMode(_in1Pin, OUTPUT);
		pinMode(_in2Pin, OUTPUT);
		this->stop();
	}

	/**
	 * @brief Set motor speed via PWM enable pin
	 * 
	 * @param speed From 0 to 255
	 */
	void setSpeed(uint8_t speed)
	{
		_speed = constrain(speed, SPEED_MIN, SPEED_MAX);
		analogWrite(_enablePin, _speed);
	}
	
	/**
	 * @brief Set motor direction by arbitrary convention
	 * 
	 * @param isForward If forward in arbitrary convention
	 */
	void setDirection(bool isForward)
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
	void stop()
	{
		digitalWrite(_in1Pin, LOW);
		digitalWrite(_in2Pin, LOW);
		analogWrite(_enablePin, 0);
	}
};
