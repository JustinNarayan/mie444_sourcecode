#pragma once
#include <CommsInterface.h>

class CommsRepeater
{
private:
	CommsInterface* toReceive;
	CommsInterface* toSend;

public:
	/**
	 * @brief Construct a new CommsRepeater
	 * 
	 * @param toReceive Channel to listen for messages
	 * @param toSend Channel to repeat back those messages
	 */
	CommsRepeater(CommsInterface* toReceive, CommsInterface* toSend) :
		toReceive(toReceive), toSend(toSend) {};
	
	/**
	 * @brief Poll toReceive for messages and send anything received on toSend.
	 * 
	 */
	void process(void) const
	{
		toReceive->receive();
		Message message;
		if (toReceive->popMessage(&message))
		{
			toSend->sendMessage(&message);
		}
	}
};