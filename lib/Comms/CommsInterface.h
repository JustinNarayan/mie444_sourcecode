#pragma once
#include <Arduino.h>
#include <RingBuffer.h>
#include "Errors.h"
#include "Settings.h"
#include "Comms.h"

class CommsInterface
{
private:
	/**
	 * Each Comms uses a single Serial connection
	 */
	Comms* comms;

	/**
	 * Each interface will manage all Comms streams via a Ring Buffer
	 */
	RingBuffer* ringBuffer;
public:
	/**
	 * @brief Constructor
	 * 
	 * @param port Must be a Hardware Serial not a Software Serial
	 */
	CommsInterface(void)  {
		comms = new Comms();
		ringBuffer = new RingBuffer();
	}

	void init(HardwareSerial* port, unsigned long baud = EXTERNAL_COMMS_BAUD_RATE);
	void receive(void);
	int popMessage(char* buffer);
	void sendMessage(const char* fmt, ...);
	void sendError(Error error);

	~CommsInterface() {
        delete ringBuffer;
        delete comms;
    }
};
