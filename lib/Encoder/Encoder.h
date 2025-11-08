#pragma once
#include "Types.h"

/**
 * Encoder driver
 */
class Encoder
{
private:
	uint8_t aPin;
	uint8_t bPin;
	volatile long encoderCount;

public:
	/**
	 * @brief Construct a new Encoder object
	 * 
	 * @param aPin 
	 * @param bPin 
	 */
	Encoder(uint8_t aPin, uint8_t bPin) : aPin(aPin), bPin(bPin), encoderCount(0)
	{
		pinMode(aPin, INPUT);
		pinMode(bPin, INPUT);
	}

	/**
	 * @brief Update function passed into ISR, run on change of aPin
	 * 
	 */
	void updateEncoderISR(void)
	{
		bool aValue = (digitalRead(this->aPin) == HIGH);
		bool bValue = (digitalRead(this->bPin) == HIGH);
		if (aValue ^ bValue) this->encoderCount++;
		else this->encoderCount--;
	}

	/**
	 * @brief Get the current encoder count
	 * 
	 * @return volatile long
	 */
	volatile long getEncoderCount(void)
	{
		return this->encoderCount;
	}
};

