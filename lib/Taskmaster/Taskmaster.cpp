#include "Taskmaster.h"

/**
 * @brief Read CommsInterface for a Message
 * 
 * @tparam numControllers 
 * @param message Now populated
 * @return If a Message received
 */
bool Taskmaster::poll(Message* message)
{
	comms->receive();
	return comms->popMessage(message);
}

/**
 * @brief Deliver Message objects to Controller objects
 * 
 * @tparam numControllers 
 * @param message To deliver
 */
void Taskmaster::disseminate(Message* message)
{
	LOOP_CONTROLLER_IDX(controller_idx)
	{
		ControllerGeneric* controller = controllers[controller_idx];
		// Message is only delivered if Controller expects to receive this MessageType
		controller->deliver(message);
	}
}

/**
 * @brief Allow Controller objects to process
 * 
 * @tparam numControllers 
 */
void Taskmaster::process(void)
{
	LOOP_CONTROLLER_IDX(controller_idx)
	{
		ControllerGeneric* controller = controllers[controller_idx];
		controller->process();
	}
}

/**
 * @brief Read and send all Message objects the Controller objects have created to be sent
 * 
 * @tparam numControllers 
 */
void Taskmaster::preach(void)
{
	LOOP_CONTROLLER_IDX(controller_idx)
	{
		ControllerGeneric* controller = controllers[controller_idx];
		Message message;
		while(controller->pickup(&message) != ControllerMessageQueueOutput::DequeueQueueEmpty)
		{
			comms->sendMessage(&message);
		}
	}
}

/**
 * @brief Execute a full loop of the Taskmaster.
 * 
 */
void Taskmaster::execute(void)
{
	Message message;
	if(poll(&message))
	{
		disseminate(&message);
	}
	
	process();
	preach();
}