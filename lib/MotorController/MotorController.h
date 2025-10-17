#pragma once
#include <Arduino.h>

#define SPEED_MIN 0
#define SPEED_MAX 255

/**
 * Motor Controller driver using L298N
 */
class MotorController
{
public:
	MotorController(uint8_t enablePin, uint8_t in1Pin, uint8_t in2Pin)
		: _enablePin(enablePin), _in1Pin(in1Pin), _in2Pin(in2Pin), _speed(0) {}

	void init(void);
	void setSpeed(uint8_t speed);
	void setDirection(bool isForward);
	void stop(void);

private:
	uint8_t _enablePin;
	uint8_t _in1Pin;
	uint8_t _in2Pin;
	uint8_t _speed;
};
