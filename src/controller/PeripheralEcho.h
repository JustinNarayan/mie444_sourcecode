#pragma once
#include <CommsInterface.h>
#include <Taskmaster.h>

class PeripheralEcho
{
private:
	CommsInterface* toReceive;
	CommsInterface* toSend;
	Taskmaster* taskmaster;

public:
	/**
	 * @brief Construct a new PeripheralEcho
	 * 
	 * @param toReceive Channel to listen for messages
	 * @param toSend Channel to repeat back those messages
	 * @param taskmaster Taskmaster sharing the toSend interface
	 */
	PeripheralEcho(CommsInterface* toReceive, CommsInterface* toSend, Taskmaster* taskmaster) :
		toReceive(toReceive), toSend(toSend), taskmaster(taskmaster) {};

    /**
     * @brief Determine if the message should be provided to the Taskmaster. Entirely based off the 
     * type of message and whether it may be useful.
     * 
     * @param message 
     * @return Whether to provide message
     */
    bool shouldProvideMessage(Message *message) const
    {
        if (!message) return false;
        
        // Encoder and command information is used for ultrasonic control
        MessageType type = message->getType();
        return (
            (type == MessageType::DrivetrainEncoderDistances) ||
            (type == MessageType::DrivetrainAutomatedResponse)
        );
    }
	
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
            // Provide the message to the Taskmaster, if it may be received
            if (this->shouldProvideMessage(&message))
                this->taskmaster->provideExternalMessage(&message);

			// If the taskmaster has requested priority, don't send the message
			// This will disappear forever
			if (false == taskmaster->hasPrioritizedSender())
				toSend->sendMessage(&message);
		}
	}
};