#pragma once
#include "Types.h"

/**
 * Servo controller
 */
class Servo
{
private:
	uint8_t pwmPin;
	servoPos pos;
	servoPos maxPos;
	servoPos minPos;

public:
	/**
	 * @brief Construct a new Servo
	 * 
	 * @param pwmPin PWM
	 */
	Servo(uint8_t pwmPin, servoPos startPos, servoPos bound1, servoPos bound2) : 
        pwmPin(pwmPin), pos(startPos)
	{
		pinMode(pwmPin, OUTPUT);
        this->minPos = min(bound1, bound2);
        this->maxPos = max(bound1, bound2);
        this->reposition(startPos);
	}

    /**
     * @brief Go to a new position, blocking.
     * 
     */
    void reposition(servoPos newPos)
    {
        // Ensure new position is in bounds
        servoPos targetPos = constrain(newPos, this->minPos, this->maxPos);
        while (this->pos != targetPos)
        {
            // Compute deta with signed value
            int16_t deltaPos = constrain(
                (int16_t)targetPos - (int16_t)this->pos,
                -SERVO_STEP_INCREMENT, 
                SERVO_STEP_INCREMENT
            );

            // Step towards target
            this->pos = (servoPos)((int16_t)this->pos + deltaPos);

            // Write to servo
            analogWrite(this->pwmPin, this->pos);
            delay(SERVO_STEP_DELAY);
        }
    }
};
