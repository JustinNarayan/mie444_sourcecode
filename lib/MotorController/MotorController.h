#pragma once
#include "Types.h"

#define MOTOR_SPEED_MIN 0
#define MOTOR_SPEED_MAX 255

/**
 * Motor Controller driver using L298N
 */
class MotorController
{
private:
	uint8_t _enablePin;
	uint8_t _in1Pin;
	uint8_t _in2Pin;
	motorSpeedApplied _speed;
    MotorDirection _in1State;
    MotorDirection _in2State;

public:
	/**
	 * @brief Construct a new Motor Controller
	 * 
	 * @param enablePin PWM
	 * @param in1Pin Digital
	 * @param in2Pin Digital
	 */
	MotorController(uint8_t enablePin, uint8_t in1Pin, uint8_t in2Pin) 
		: _enablePin(enablePin), _in1Pin(in1Pin), _in2Pin(in2Pin), _speed(0), _in1State(MotorDirection::Reverse), _in2State(MotorDirection::Reverse)
	{
		pinMode(_enablePin, OUTPUT);
		pinMode(_in1Pin, OUTPUT);
		pinMode(_in2Pin, OUTPUT);
		this->coast();
	}

	/**
	 * @brief Set motor speed via PWM enable pin
	 * 
	 * @param speed Must be constrained form 0 to 255
	 */
	void setSpeed(motorSpeedRaw rawSpeed)
	{
		this->_speed = (motorSpeedApplied)constrain(rawSpeed, MOTOR_SPEED_MIN, MOTOR_SPEED_MAX);
		analogWrite(_enablePin, this->_speed);
	}
	
	/**
	 * @brief Update direction pins
	 * 
	 * @param in1 Requested direciton of in1 pin
     * @param in2 Reqeusted direction of in2 pin
	 */
	void updateDirectionPins(MotorDirection in1, MotorDirection in2)
	{
        // Compare previous directions to current
        if ((this->_in1State == in1) && (this->_in2State == in2)) return; // Nothing required

        // Delay to prevent a brownout
        this->setSpeed(0);
        delayMicroseconds(MOTOR_BROWNOUT_DIRECTION_TIMEOUT_US);

        // Set motor direction
        this->_in1State = in1;
        digitalWrite(_in1Pin, (this->_in1State == MotorDirection::Forward) ? HIGH : LOW);
        this->_in2State = in2;
        digitalWrite(_in2Pin, (this->_in2State == MotorDirection::Forward) ? HIGH : LOW);
	}
	
	/**
	 * @brief Set motor direction by arbitrary convention
	 * 
	 * @param isForward If forward in arbitrary convention
	 */
	void setDirection(bool isForward)
	{
        this->updateDirectionPins(
            isForward ? MotorDirection::Forward : MotorDirection::Reverse, 
            isForward ? MotorDirection::Reverse : MotorDirection::Forward
        );
	}

    /**
	 * @brief Set motor to brake
	 * 
	 * @param isForward If forward in arbitrary convention
	 */
	void setBrake(void)
	{
        this->updateDirectionPins(MotorDirection::Forward, MotorDirection::Forward);
    }

	/**
	 * @brief Emergency stop for motor
	 * 
	 */
	void coast()
	{
        this->updateDirectionPins(MotorDirection::Reverse, MotorDirection::Reverse);
	}
};
