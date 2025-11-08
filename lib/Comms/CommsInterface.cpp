#include "CommsInterface.h"
#include "MemoryUtilities.h"

/**
 * @brief Initialize Comms.
 * 
 * @param baud 
 */
void CommsInterface::init(HardwareSerial* port, unsigned long baud)
{
	comms->init(port, baud);
}

/**
 * @brief Receive available information and store in ring buffer.
 */
void CommsInterface::receive(void)
{
	// Construct a buffer to read into
	char buffer[STRING_LENGTH_MAX];

	// Receive info into buffer
	size_t numBytesRead = comms->receiveInfo(buffer, STRING_LENGTH_MAX);

	// Write into buffer, sealing if the end char is encountered
	ringBuffer->writeIntoBuffer(buffer, numBytesRead, true);
}

/**
 * @brief Pop first unread message from ring buffer
 * 
 * @param outMessage Pointer to an uninitialized message object to initialize
 */
int CommsInterface::popMessage(Message* outMessage)
{
	// Pop raw buffer contents
	char buffer[STRING_LENGTH_MAX];
	int ret = ringBuffer->popBuffer(buffer);

	// Instantiate raw buffer content as a message
	if (ret == RET_READ_BUFFER_SUCCESS)
	{
		outMessage->init(buffer);
		return true;
	}
	return false;
}

/**
 * @brief Send a message over the port
 * 
 * @param Message Pointer to an initialized message object
 */
void CommsInterface::sendMessage(Message* message)
{
	// Extract raw buffer contents
	char buffer[STRING_LENGTH_MAX];
	message->getRaw(buffer);

	// Compute message size
	size_t size = message->getRawSize();

	// Send
	comms->sendInfo(buffer, size);
}

/**
 * @brief Send a generated error over the communication interface
 * 
 * @param error Error defined in Errors.h
 */
void CommsInterface::sendError(Error error)
{
	Message message;
	char buffer[STRING_LENGTH_MAX];
	format(buffer, "Error [%d]: %s", error.code, error.message);
	message.init(buffer);

	this->sendMessage(&message);
}
