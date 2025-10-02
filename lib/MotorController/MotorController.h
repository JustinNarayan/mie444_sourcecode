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
	MotorController(uint8_t enablePin, uint8_t in1Pin, uint8_t in2Pin);

	void init();
	void setSpeed(uint8_t speed);
	void setDirection(bool isForward);
	void stop();

private:
	uint8_t _enablePin;
	uint8_t _in1Pin;
	uint8_t _in2Pin;
	uint8_t _speed;
};
