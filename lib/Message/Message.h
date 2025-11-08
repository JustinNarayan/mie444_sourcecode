#pragma once
#include "Settings.h"
#include "MemoryUtilities.h"
#include "MessageType.h"

#define MESSAGE_CONTENT_SIZE_AUTOMATIC (0xFF) // message content is null-terminated stirng

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
	size_t msgContentSize;

	/**
	 * The serialized representation of the message including the type char, content, end char, and 
	 * null-terminator
	 */
	char msgRawBuffer[STRING_LENGTH_MAX];

public:
	Message(void) : initialized(false), msgContentSize(MESSAGE_CONTENT_SIZE_AUTOMATIC) {
		memorySet(this->msgContentBuffer, '\0', sizeof(msgContentBuffer));
		memorySet(this->msgRawBuffer, '\0', sizeof(msgRawBuffer));
	};

	/**
	 * A message can either be initialized from:
	 * (1) a pure string representation, as when receiving from Serial
	 * (2) a type/size/content representation, as when sending on Serial
	 * 
	 * A size needs not be specified unless content contains \0s in addition to null-terminator.
	 * This may be the case if the values being sent are serialized structs rather than enums.
	 * 
	 */
	void init(
		const char* rawBuffer
	);
	void init(
		MessageType type,
		const size_t contentSize,
		const char* contentBuffer
	);

	MessageType getType(void);
	size_t getContentSize(void);
	void getContent(char* outBuffer);
	void getRaw(char* outBuffer);
	size_t getRawSize(void);
};
