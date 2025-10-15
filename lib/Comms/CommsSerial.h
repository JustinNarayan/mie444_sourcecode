#pragma once
#include <Arduino.h>
#include "CommsInterface.h"

/**
 * Communication interface for Serial Rx/Tx
 */
class CommsSerial : public Comms
{
public:
	void init(unsigned long baud = 9600) override
	{
		Serial.begin(baud);
		while (!Serial)
		{
			;
		} // Allow time for Serial to connect
	}

	void send(char *buffer) override
	{
		Serial.println(buffer);
	}
};