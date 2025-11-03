#include "Message.h"

/**
 * @brief Initialize Message by decoding its raw string representation into its type and 
 * content
 * 
 * @param rawBuffer Raw string information
 */
void Message::init(const char* rawBuffer)
{
	// Decode information if raw message contains is well-formed
	size_t rawLength = stringLength(rawBuffer);
	if (
		(rawBuffer != NULL) &&
		(rawLength >= MESSAGE_ENCODING_LENGTH)
	)
	{
		memoryCopy(this->msgRawBuffer, rawBuffer, rawLength);
		this->msgRawBuffer[rawLength] = '\0';

		size_t contentLength = rawLength - MESSAGE_ENCODING_LENGTH;
		memoryCopy(this->msgContentBuffer, &(rawBuffer[1]), contentLength); // exclude initial type char
		this->msgContentBuffer[contentLength] = '\0';
		
		this->type = static_cast<MessageType>(this->msgRawBuffer[0]);

		// Properly initialized
		this->initialized = true;
	}
}


/**
 * @brief Initialize Message by encoding its type and content representation into a raw string
 * 
 * @param type Message type
 * @param contentBuffer Unencoded content
 */
void Message::init(MessageType type, const char* contentBuffer)
{	
	// Encode information
	if (contentBuffer != NULL)
	{
		this->type = type;

		size_t contentLength = min(stringLength(contentBuffer), MESSAGE_CONTENT_LENGTH_MAX);
		memoryCopy(this->msgContentBuffer, contentBuffer, contentLength);
		this->msgContentBuffer[contentLength] = '\0';

		format(this->msgRawBuffer, 
			"%c%s%c\0", 
			static_cast<char>(this->type), 
			this->msgContentBuffer, 
			MESSAGE_END_CHAR
		);
		
		// Properly initialized
		this->initialized = true;
	}
	
}

/**
 * @brief Get message type
 * 
 * @return MessageType 
 */
MessageType Message::getType(void)
{
	return this->type;
}

/**
 * @brief Get message content
 * 
 * @param outBuffer Contains message content after call, must be of minimum size 
 * MESSAGE_CONTENT_LENGTH_MAX
 */
void Message::getContent(char* outBuffer)
{
	if (outBuffer != NULL)
	{
		size_t contentLength = stringLength(this->msgContentBuffer);
		memoryCopy(outBuffer, this->msgContentBuffer, contentLength);
		outBuffer[contentLength] = '\0';
	}
}

/**
 * @brief Get message raw string representation
 * 
 * @param outBuffer Contains message content after call, must be of minimum size 
 * STRING_LENGTH_MAX
 */
void Message::getRaw(char* outBuffer)
{
	if (outBuffer != NULL)
	{
		size_t rawLength = stringLength(this->msgRawBuffer);
		memoryCopy(outBuffer, this->msgRawBuffer, rawLength);
		outBuffer[rawLength] = '\0';
	}
}
