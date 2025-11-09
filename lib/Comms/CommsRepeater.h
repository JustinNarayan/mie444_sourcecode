#pragma once
#include <CommsInterface.h>
#include <Taskmaster.h>

class CommsRepeater
{
private:
	CommsInterface* toReceive;
	CommsInterface* toSend;
	Taskmaster* taskmaster;

public:
	/**
	 * @brief Construct a new CommsRepeater
	 * 
	 * @param toReceive Channel to listen for messages
	 * @param toSend Channel to repeat back those messages
	 * @param taskmaster Taskmaster sharing the toSend interface
	 */
	CommsRepeater(CommsInterface* toReceive, CommsInterface* toSend, Taskmaster* taskmaster) :
		toReceive(toReceive), toSend(toSend), taskmaster(taskmaster) {};
	
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
			// If the taskmaster has requested priority, don't send the message
			// This will disappear forever
			if (false == taskmaster->hasPrioritizedSender())
				toSend->sendMessage(&message);
		}
	}
};