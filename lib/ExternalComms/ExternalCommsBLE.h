#pragma once
#include <Arduino.h>
#include <SoftwareSerial.h>
#include "ExternalCommsInterface.h"

/**
 * Global objects
 */
SoftwareSerial BTSerial(PIN_BLUETOOTH_RX, PIN_BLUETOOTH_TX); // RX | TX

/**
 * Communication interface for BLE Rx/Tx
 */
class ExternalCommsBLE : public ExternalComms
{
public:
	void init(unsigned long baud = EXTERNAL_COMMS_BAUD_RATE) override
	{
		BTSerial.begin(baud); 
		while (!BTSerial)
		{
			;
		} // Allow time for Serial to connect
	}

	void send(const char *buffer) override
	{
		BTSerial.write(buffer);
	}

	size_t get_num_unread_bytes() override
	{
		return (size_t)BTSerial.available();
	}

	char read_byte() override
	{
		return BTSerial.read();
	}
};