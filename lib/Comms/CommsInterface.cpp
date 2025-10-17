#include "CommsInterface.h"
#include "CommsUtilities.h"

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
	char buffer[MESSAGE_LENGTH_MAX];

	// Receive info into buffer
	size_t numBytesRead = comms->receiveInfo(buffer, MESSAGE_LENGTH_MAX);

	// Write into buffer, sealing if the end char is encountered
	ringBuffer->writeIntoBuffer(buffer, numBytesRead, true);
}

/**
 * @brief Pop first unread message from ring buffer
 * 
 * @param buffer To write into
 */
int CommsInterface::popMessage(char* buffer)
{
	// Pop buffer
	int ret = ringBuffer->popBuffer(buffer);

	return ret;
}

/**
 * @brief Send a format string over the port
 * 
 * @param fmt A format string
 * @param ... args
 */
void CommsInterface::sendMessage(const char* fmt, ...)
{
	// Construct formatted string
	char buffer[MESSAGE_LENGTH_MAX];
	va_list args;
	va_start(args, fmt);
	format(buffer, fmt, args);
	va_end(args);

	// Send over port
	comms->sendInfo(buffer);
}

/**
 * @brief Send a generated error over the communication interface
 * 
 * @param error Error defined in Errors.h
 */
void CommsInterface::sendError(Error error)
{
	this->sendMessage("Error [%d]: %s %c", error.code, error.message, MESSAGE_END_CHAR);
}
