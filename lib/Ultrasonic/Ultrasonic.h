#pragma once
#include "UltrasonicDefs.h"

/**
 * Ultrasonic driver
 */
class Ultrasonic
{
private:
	uint8_t echoPin;
	uint8_t triggerPin;

public:
	/**
	 * @brief Construct a new Ultrasonic
	 * 
	 */
	Ultrasonic(void) {};

    /**
     * @brief Initialize pins
     * 
	 * @param echoPin Analog
	 * @param triggerPin Digital
     */
    void init(uint8_t echoPin, uint8_t triggerPin)
    {
        this->echoPin = echoPin;
        this->triggerPin = triggerPin;

		pinMode(this->echoPin, INPUT);
		pinMode(this->triggerPin, OUTPUT);
	}

    /**
     * @brief Request a nerw reading from the ultrasonic reading
     * 
     * @return whether request succesful
     */
    ultrasonicDistance_in ping(void)
    {        
        // Until ultrasonics are added, just return a value
        return 1;

        // Make sure trigger pin is low
        // if (digitalRead(this->triggerPin) == HIGH)
        // {
        //     digitalWrite(this->triggerPin, LOW);
        //     delay(ULTRASONIC_TIME_TRIGGER_RESET_MS);
        // }

        // // Raise trigger pin
        // digitalWrite(this->triggerPin, HIGH);
        // delayMicroseconds(ULTRASONIC_TIME_TRIGGER_HIGH_US);
        // digitalWrite(this->triggerPin, LOW);

        // // Receive echo
        // time_us pingTime = pulseIn(this->echoPin, HIGH);
        // return (ultrasonicDistance_in)ULTRASONIC_TIME_US_TO_INCH(pingTime);
    }
};
