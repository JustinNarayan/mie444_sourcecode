#pragma once
#include "Settings.h"
#include "MemoryUtilities.h"
#include "MessageType.h"

/**
 * @brief Generalized interface for all messages sent on comms. All messages ultimately have an
 * underlying string / byte representation, but they are wrapped in this form.
 * 
 */
class Message
{
private:
	/**
	 * Whether message has been populated yet
	 * 
	 */
	bool initialized;

	/**
	 * The underlying representation of the message with the Message type and content separated
	 */
	MessageType type;
	char msgContentBuffer[MESSAGE_CONTENT_LENGTH_MAX];

	/**
	 * The serialized representation of the message including the type char, content, end char, and 
	 * null-terminator
	 */
	char msgRawBuffer[STRING_LENGTH_MAX];

public:
	Message(void) : initialized(false) {
		memorySet(this->msgContentBuffer, '\0', sizeof(msgContentBuffer));
		memorySet(this->msgRawBuffer, '\0', sizeof(msgRawBuffer));
	};

	/**
	 * A message can either be initialized from:
	 * (1) a pure string representation, as when receiving from Serial
	 * (2) a type/content representation, as when sending on Serial
	 */
	void init(const char* rawBuffer);
	void init(MessageType type, const char* contentBuffer);

	MessageType getType(void);
	void getContent(char* outBuffer);
	void getRaw(char* outBuffer);
};
