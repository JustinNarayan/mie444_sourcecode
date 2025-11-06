#pragma once
#include "Message.h"

/**
 * Return codes. Queue existence and type validation is performed by MessageQueueHub
 */
#define RET_ENQUEUE_NO_QUEUE (-3)
#define RET_ENQUEUE_QUEUE_IS_FULL (-2)
#define RET_ENQUEUE_DISALLOWED_TYPE (-1)
#define RET_ENQUEUE_SUCCESS (0)

#define RET_DEQUEUE_NO_QUEUE (-3)
#define RET_DEQUEUE_QUEUE_IS_EMPTY (-2)
#define RET_DEQUEUE_DISALLOWED_TYPE (-1)
#define RET_DEQUEUE_SUCCESS (0)


/**
 * @brief A generic MessageQueue-type object that real MessageQueue objects inherit for 
 * polymorphism simplicity. This provides a general pointer to a non-templated MessageQueue
 * 
 */
class MessageQueueGeneric
{
protected:
	/**
	 * Number of total messages that can be stored in the queue
	 */
	Message queue[MESSAGE_QUEUE_SIZE];
	size_t head;
	size_t tail;

public:
	MessageQueueGeneric(void) : head(0), tail(0) {};
    virtual ~MessageQueueGeneric() = default;
	
	/**
	 * @brief Check if messages stored in queue
	 * 
	 * @return If no messages in queue
	 */
	bool isEmpty(void) const
	{
		return head == tail;
	}
	
	/**
	 * @brief Check if any space left for more messages in queue
	 * 
	 * @return If queue is full with no space with for new messages
	 */
	bool isFull(void) const
	{
		return ((head + 1) % (size_t)MESSAGE_QUEUE_SIZE) == tail;
	}

    virtual int enqueue(Message* message) = 0;
    virtual int dequeue(Message* message) = 0;
};

/**
 * @brief A MessageQueue will store a buffer of Messages of one specific type. Typing is enforced 
 * on enqueue. This is treated as a memory dump, so messages are not initialized on enqueue or 
 * dequeue, but rather memoryCopy is used for the whole message. It is assumed the message is 
 * initialized on enqueue.
 * 
 * @tparam allowedType 
 */
template <MessageType allowedType>
class MessageQueue : public MessageQueueGeneric
{
public:
	/**
	 * @brief Enqueue a new message
	 * 
	 * @param message Pointer with data to enqueue
	 * @return RET_ENQUEUE_QUEUE_IS_FULL if no space in queue
	 * 		   RET_ENQUEUE_DISALLOWED_TYPE if provided message is incorrect type
	 *         RET_ENQUEUE_SUCCESS otherwise.
	 */
	int enqueue(Message* message)
	{
		if (this->isFull()) return RET_ENQUEUE_QUEUE_IS_FULL;
		// Enforce message is of correct type
		if (message->getType() != allowedType) return RET_ENQUEUE_DISALLOWED_TYPE;
		// Copy data into queue
		memoryCopy(&(queue[head]), message, sizeof(*message));
		// Move head
		head = (head + 1) % (size_t)MESSAGE_QUEUE_SIZE;
		return RET_ENQUEUE_SUCCESS;
	}

	/**
	 * @brief Dequeue a message
	 * 
	 * @param message Pointer to dequeue into
	 * @return RET_DEQUEUE_QUEUE_IS_EMPTY if no messages to dequeue
	 *         RET_DEQUEUE_SUCCESS otherwise.
	 */
	int dequeue(Message* message)
	{
		if (this->isEmpty()) return RET_DEQUEUE_QUEUE_IS_EMPTY;
		// Copy data into message
		memoryCopy(message, &(queue[tail]), sizeof(queue[tail]));
		// Move tail
		tail = (tail + 1) % (size_t)MESSAGE_QUEUE_SIZE;
		return RET_DEQUEUE_SUCCESS;
	}
};