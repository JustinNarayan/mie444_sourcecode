#pragma once
#include <Arduino.h>
#include "ExternalCommsInterface.h"

/**
 * Communication interface for Serial Rx/Tx
 */
class ExternalCommsSerial : public ExternalComms
{
public:
	void init(unsigned long baud = EXTERNAL) override
	{
		Serial.begin(baud);
		while (!Serial)
		{
			;
		} // Allow time for Serial to connect
	}

	void send(const char *buffer) override
	{
		Serial.println(buffer);
	}


	size_t get_num_unread_bytes() override
	{
		return (size_t)Serial.available();
	}

	char read_byte() override
	{
		return Serial.read();
	}
};