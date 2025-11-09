#include "Taskmaster.h"

/**
 * @brief Read CommsInterface for all bytes
 * 
 */
void Taskmaster::receive(void)
{
	while(comms->receive());
}

/**
 * @brief Try to pull a Message from the CommsInterface
 * 
 * @param message Now populated
 * @return If a Message received
 */
bool Taskmaster::poll(Message* message)
{
	return comms->popMessage(message);
}

/**
 * @brief Deliver Message objects to Controller objects
 * 
 * @param message To deliver
 */
void Taskmaster::dispatch(Message* message)
{
	LOOP_CONTROLLER_IDX(controller_idx)
	{
		ControllerGeneric* controller = controllers[controller_idx];
		// Message is only delivered if Controller expects to receive this MessageType
		// Force message to be delivered
		controller->deliver(message, true);
	}
}

/**
 * @brief Allow Controller objects to process
 * 
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
 */
void Taskmaster::collect(void)
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
	receive();

	Message message;
	while(poll(&message))
	{
		dispatch(&message);
	}
	
	process();
	collect();
}