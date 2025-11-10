#pragma once
#include <CommsInterface.h>
#include "MemoryUtilities.h"

class CommsEnvoy
{
private:
	CommsInterface* toSend;

public:
	/**
	 * @brief Construct a new CommsEnvoy
	 * 
	 * @param toReceive Channel to send messages on
	 */
	CommsEnvoy(CommsInterface* toSend) : toSend(toSend) {};

	/**
	 * @brief Envoy message on comms interface
	 *
	 */
	void envoy(Message* message)
	{
		this->toSend->sendMessage(message);
	}
};