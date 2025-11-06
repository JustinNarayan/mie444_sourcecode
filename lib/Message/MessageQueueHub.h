#pragma once
#include "MessageQueue.h"


/**
 * @brief A MessageQueueHub stores a list of MessageQueues for a given list of MessageType values.
 * A forward declaration implementing the MessageTypes template is required so that MessageQueueHub 
 * can then be directly initialzied from a list of MessageType values.
 * 
 * @tparam allowedTypes 
 */
template <MessageType... allowedTypes>
class MessageQueueHub
{
private:
	MessageQueueGeneric* queues[sizeof...(allowedTypes)];
    static constexpr MessageType types[sizeof...(allowedTypes)] = { allowedTypes... };

	/**
	 * @brief Recursively initialize all required MessageQueue objects at compile time
	 * 
	 */
	template <size_t index = 0, MessageType thisType, MessageType... nextTypes>
	void initQueue(void)
	{
		queues[index] = new MessageQueue<thisType>;
		initQueue<index + 1, nextTypes...>();
	}

	template <size_t index>
	void initQueue(void) {}; // ends recursive call

	/**
	 * @brief Get the pointer to the included MessageQueue of desired MessageType
	 * 
	 * @param desiredType MessageType of queue
	 * @return auto Pointer to MessageQueue, or NULL if none found.
	 */
    MessageQueueGeneric* locateQueueByType(MessageType desiredType)
    {
        for (size_t i = 0; i < sizeof...(allowedTypes); ++i)
            if (types[i] == desiredType)
                return queues[i];

		// Return a null pointer if not found
        return (MessageQueueGeneric*)nullptr;
    }
public:
	/**
	 * @brief Use templated helpers to construct all MessageQueue objects of right type at compile 
	 * time
	 * 
	 */
	MessageQueueHub(void)
	{
		this->initQueue<0, allowedTypes...>();
	}

	/**
	 * @brief Delete all generated MessageQueue objects at end of compile
	 * 
	 */
	~MessageQueueHub()
	{
		for (size_t i = 0; i < sizeof...(allowedTypes); ++i)
		{
			delete queues[i];
			queues[i] = nullptr; // optional, clears pointer
		}
	}

	/**
	 * @brief Check if any queue types are defined.
	 * 
	 * @return If this is an empty hub with no queues.
	 */
	static constexpr bool hasNoQueues(void) { return (sizeof...(allowedTypes) == 0); }

	int enqueue(Message* message)
	{
		// Verify queues exist
		if (hasNoQueues()) return RET_ENQUEUE_NO_QUEUE;

		// Verify queue of this type exists
		MessageType desiredType = message->getType();
		auto queue = locateQueueByType(desiredType);
		if (queue == nullptr) return RET_ENQUEUE_DISALLOWED_TYPE;

		// Enqueue
		return queue->enqueue(message);
	}

	int dequeue(const MessageType desiredType, Message* message)
	{
		if (hasNoQueues()) return RET_DEQUEUE_NO_QUEUE;

		// Verify queue of this type exists
		MessageQueueGeneric* queue = locateQueueByType(desiredType);
		if (queue == nullptr) return RET_DEQUEUE_DISALLOWED_TYPE;

		// Dequeue
		return queue->dequeue(message);
	}
};
