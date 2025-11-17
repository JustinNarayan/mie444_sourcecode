#include "MemoryUtilities.h"
#include "Taskmaster.h"

/**
 * @brief Construct a Taskmaster
 *
 * @param comms
 * @param controllers
 * @param numControllers
 */
Taskmaster::Taskmaster(
    CommsInterface *comms,
    ControllerGeneric *controllers[],
    size_t numControllers) : comms(comms),
                             controllers(controllers),
                             numControllers(numControllers),
                             hasExternalMessage(false) {}; // externalMessage is uninitialized

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
 * @brief Check if any controllers have requested message sending priority
 * 
 */
void Taskmaster::monitorPrioritzedSenderRequests(void)
{
	// Check if current prioritzed sender should be cleared
	if (this->prioritizedSender != nullptr)
	{
		if (false == this->prioritizedSender->isRequestingPrioritizedSender())
		{
			this->prioritizedSender = nullptr;
		}
	}

	// Look for new prioritized senders
	if (this->prioritizedSender == nullptr)
	{
		LOOP_CONTROLLER_IDX(controller_idx)
		{
			ControllerGeneric* controller = controllers[controller_idx];
			if (controller->isRequestingPrioritizedSender())
			{
				this->prioritizedSender = controller;
				return;
			}
		}
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

		// Skip if a different controller is prioiritzed
		if (
			(this->prioritizedSender != nullptr) &&
			(this->prioritizedSender != controller)
		)
		{
			continue;
		}

		Message message;
		while(controller->pickup(&message) != ControllerMessageQueueOutput::DequeueQueueEmpty)
		{
			comms->sendMessage(&message);
		}
	}
}

/**
 * @brief Check if a controller is requesting send priority
 * 
 * @return true 
 * @return false 
 */
bool Taskmaster::hasPrioritizedSender(void)
{
	return (this->prioritizedSender != nullptr);
}

/**
 * @brief Allow another party to provide a message to be dispatched by Taskmaster
 *
 * @param message
 */
void Taskmaster::provideExternalMessage(Message *message)
{
    memoryCopy(&this->externalMessage, message, sizeof(Message));
    this->hasExternalMessage = true;
}

/**
 * @brief Execute a full loop of the Taskmaster.
 * 
 */
void Taskmaster::execute(void)
{
	receive();

    if (this->hasExternalMessage)
    {
        dispatch(&this->externalMessage);
        this->hasExternalMessage = false;
    }

	Message message;
	while(poll(&message))
	{
		dispatch(&message);
	}
	
	process();
	collect();
}