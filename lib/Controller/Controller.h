#pragma once
#include <Arduino.h>
#include "MessageQueueHub.h"

/*****************************************************
 *                   ENUM UTILITIES                  *
 *****************************************************/
enum class ControllerMessageQueueOutput {
	/* Enqueue */
	EnqueueNoQueue,
	EnqueueQueueFull,
	EnqueueDisallowedType,
	EnqueueSuccess,

	/* Dequeue */
	DequeueNoQueue,
	DequeueQueueEmpty,
	DequeueDisallowedType,
	DequeueSuccess
};

static inline ControllerMessageQueueOutput toControllerMessageQueueOutputEnqueue(
	int messageQueueOutput
)
{
	switch (messageQueueOutput)
	{
		case RET_ENQUEUE_NO_QUEUE: return ControllerMessageQueueOutput::EnqueueNoQueue;
		case RET_ENQUEUE_QUEUE_IS_FULL: return ControllerMessageQueueOutput::EnqueueQueueFull;
		case RET_ENQUEUE_DISALLOWED_TYPE: return ControllerMessageQueueOutput::EnqueueDisallowedType;
		case RET_ENQUEUE_SUCCESS:
		default: return ControllerMessageQueueOutput::EnqueueSuccess;
	}
}

static inline ControllerMessageQueueOutput toControllerMessageQueueOutputDequeue(
	int messageQueueOutput
)
{
	switch (messageQueueOutput)
	{
		case RET_DEQUEUE_NO_QUEUE: return ControllerMessageQueueOutput::DequeueNoQueue;
		case RET_DEQUEUE_QUEUE_IS_EMPTY: return ControllerMessageQueueOutput::DequeueQueueEmpty;
		case RET_DEQUEUE_DISALLOWED_TYPE: return ControllerMessageQueueOutput::DequeueDisallowedType;
		case RET_DEQUEUE_SUCCESS:
		default: return ControllerMessageQueueOutput::DequeueSuccess;
	}
}



/**
 * @brief A generic Controller-type object that real Controller objects inherit for 
 * polymorphism simplicity. This provides a general pointer to a non-templated Taskmaster
 * 
 */
class ControllerGeneric
{
private:
	virtual ControllerMessageQueueOutput read(const MessageType desiredType, Message* message) = 0;
	virtual ControllerMessageQueueOutput post(Message* message) = 0;
public:
	virtual ControllerMessageQueueOutput deliver(Message* message) = 0;
	virtual ControllerMessageQueueOutput pickup(Message* message) = 0;

    /**
     * @brief Core processing function
     */
    virtual void process() = 0;
};


/**
 * @brief A Controller will receive and send Message objects. The types must be pre-specified in a 
 * forward declaration.
 * 
 * @tparam messageTypesIn 
 * @tparam messageTypesOut 
 */
template <typename messageTypesIn, typename messageTypesOut>
class Controller;

/**
 * @brief A Controller will manage a certain subsystem:
 * - All communications into the Controller
 * - All communications out of the Controller
 * - All processes within the Controller
 * 
 * @tparam messageTypesIn MessageType list
 * @tparam messageTypesOut MessageType list
 */
template <MessageType... messageTypesIn, MessageType... messageTypesOut>
class Controller<
	MessageTypes<messageTypesIn...>, 
	MessageTypes<messageTypesOut...>
> : public ControllerGeneric
{
protected:
    MessageQueueHub<messageTypesIn...> messagesIn;
    MessageQueueHub<messageTypesOut...> messagesOut;

    /**
     * @brief Read a messages from the messagesIn queue
     * 
     * 
     * @param desiredType MessageType
     * @param message Message to output into
     * @return ControllerMessageQueueOutput Dequeue value
     */
    ControllerMessageQueueOutput read(const MessageType desiredType, Message* message)
    {
		return toControllerMessageQueueOutputDequeue(messagesIn.dequeue(desiredType, message));
	}

    /**
     * @brief Writes a messages to the messagesOut queue
     * 
     * 
     * @param desiredType MessageType
     * @param message Message to dequeue into
     * @return ControllerMessageQueueOutput Enqueue value
     */
    ControllerMessageQueueOutput post(Message* message)
    {
		return toControllerMessageQueueOutputEnqueue(messagesOut.enqueue(message));
	}	
public:
    Controller() = default;
    virtual ~Controller() = default;

	/**
	 * @brief Write a message to the messagesIn queue
	 * 
	 * @param message To enqueue
     * @return ControllerMessageQueueOutput Enqueue value
	 */
    ControllerMessageQueueOutput deliver(Message* message)
    {
		return toControllerMessageQueueOutputEnqueue(messagesIn.enqueue(message));
	}

	/**
	 * @brief Read a message from the messagesOut queue.
	 * 
	 * @param message Message to dequeue into
     * @return ControllerMessageQueueOutput Dequeue value
	 * If no messages left, to read, ControllerMessageQueueOutput::DequeueSuccess
	 */
    ControllerMessageQueueOutput pickup(Message* message)
    {

		// Loop over all allowed output MessageTypes
		constexpr MessageType types[] = { messageTypesOut... };
        for (size_t i = 0; i < sizeof...(messageTypesOut); ++i)
		{
			ControllerMessageQueueOutput ret = toControllerMessageQueueOutputDequeue(
				messagesOut.dequeue(types[i], message)
			);
			if (ret == ControllerMessageQueueOutput::DequeueSuccess)
				return ret;
		}
		return ControllerMessageQueueOutput::DequeueQueueEmpty;
	}

    /**
     * @brief Core processing function
     */
    virtual void process() = 0;
};
