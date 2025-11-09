#pragma once
#include "MessageQueue.h"

/**
 * @brief Helper struct to hold a single MessageQueue
 * 
 * @tparam allowedType
 */
template <MessageType allowedType>
struct QueueHolder
{
    MessageQueue<allowedType> queue;
    MessageQueueGeneric* getGenericQueue() { return &queue; }
};

/**
 * @brief A MessageQueueHub stores a list of MessageQueues for a given list of MessageType values.
 * A forward declaration implementing the MessageTypes template is required so that MessageQueueHub 
 * can then be directly initialzied from a list of MessageType values.
 * 
 * @tparam allowedTypes 
 */
template <MessageType... allowedTypes>
class MessageQueueHub : private QueueHolder<allowedTypes>...
{
private:
    static constexpr MessageType types[sizeof...(allowedTypes)] = { allowedTypes... };

	/**
	 * @brief Get the pointer to the included MessageQueue of desired MessageType
	 * 
	 * @param desiredType MessageType of queue
	 * @return auto Pointer to MessageQueue, or NULL if none found.
	 */
    MessageQueueGeneric* locateQueueByType(MessageType desiredType)
    {
        // Expand over all allowed types and check each
        MessageQueueGeneric* result = nullptr;
        bool found = false;
        using expander = int[];
        (void)expander{0, (
			(allowedTypes == desiredType ? (
				result = QueueHolder<allowedTypes>::getGenericQueue(), found = true
			) : false), 
			0)...
		};
        return result;
    }
public:
    MessageQueueHub() = default;
    ~MessageQueueHub() = default;

	/**
	 * @brief Check if any queue types are defined.
	 * 
	 * @return If this is an empty hub with no queues.
	 */
	static constexpr bool hasNoQueues(void) { return (sizeof...(allowedTypes) == 0); }

	/**
	 * @brief Enqueue a message, if a valid queue for its MessageType exists
	 * 
	 * @param message 
	 * @return See MessageQueue.h
	 */
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

	/**
	 * @brief Dequeue a message, if a valid queue for its MessageType exists
	 * 
	 * @param desiredType 
	 * @param message 
	 * @return See MessageQueue.h
	 */
	int dequeue(const MessageType desiredType, Message* message)
	{
		if (hasNoQueues()) return RET_DEQUEUE_NO_QUEUE;

		// Verify queue of this type exists
		MessageQueueGeneric* queue = locateQueueByType(desiredType);
		if (queue == nullptr) return RET_DEQUEUE_DISALLOWED_TYPE;

		// Dequeue
		return queue->dequeue(message);
	}

	/**
	 * @brief Clear the whole queue, if a valid queue for this MessageType exists
	 * 
	 * @return See MessageQueue.h
	 */
	int clear(const MessageType desiredType)
	{
		if (hasNoQueues()) return RET_CLEAR_NO_QUEUE;
		
		// Verify queue of this type exists
		MessageQueueGeneric* queue = locateQueueByType(desiredType);
		if (queue == nullptr) return RET_CLEAR_DISALLOWED_TYPE;

		// Clear
		queue->clear();
		return RET_CLEAR_SUCCESS;
	}
};
