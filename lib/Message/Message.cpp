#include "Message.h"
#include "Types.h"

/**
 * @brief Initialize Message by decoding its raw string representation into its type and 
 * content
 * 
 * @param rawBuffer Raw string information
 */
void Message::init(const char* rawBuffer)
{
	// Decode information
	if (rawBuffer != NULL)
	{
		// Cast the first char to a MessageType
		this->type = static_cast<MessageType>(rawBuffer[0]);

		// Cast the second char to a size_t
		this->msgContentSize = (size_t)(rawBuffer[1]);

		// Encode raw information
		size_t rawLength = this->msgContentSize + MESSAGE_ENCODING_LENGTH;
		memoryCopy(this->msgRawBuffer, rawBuffer, rawLength);
		this->msgRawBuffer[rawLength] = '\0';

		// Encode raw infromation without MessageType char, size char, or end char
		size_t contentLength = rawLength - MESSAGE_ENCODING_LENGTH;
		memoryCopy(this->msgContentBuffer, &(rawBuffer[MESSAGE_PRE_ENCODE_LENGTH]), contentLength);
		this->msgContentBuffer[contentLength] = '\0';

		// Properly initialized
		this->initialized = true;
	}
}


/**
 * @brief Initialize Message by encoding its type and content representation into a raw string
 * 
 * @param type Message type
 * @param contentSize If not MESSAGE_CONTENT_SIZE_AUTOMATIC, the number of content bytes
 * @param contentBuffer Unencoded content
 */
void Message::init(MessageType type, const size_t contentSize, const char* contentBuffer)
{	
	// Encode information
	if (contentBuffer != NULL)
	{
		// Store MessageType
		this->type = type;

		// Store content size
		// Must be less than MESSAGE_CONTENT_LENGTH_MAX to be properly transmitted
		this->msgContentSize = (contentSize == MESSAGE_CONTENT_SIZE_AUTOMATIC) ?
			stringLength(contentBuffer, MESSAGE_CONTENT_LENGTH_MAX) :
			contentSize;

		// Store unencoded content
		memoryCopy(this->msgContentBuffer, contentBuffer, this->msgContentSize);
		this->msgContentBuffer[this->msgContentSize] = '\0';

		// Encode raw information with MessageType char and end char
		this->msgRawBuffer[0] = static_cast<char>(this->type);
		this->msgRawBuffer[1] = static_cast<char>(this->msgContentSize);
		memoryCopy(
			&(this->msgRawBuffer[MESSAGE_PRE_ENCODE_LENGTH]), 
			this->msgContentBuffer, 
			this->msgContentSize
		);
		this->msgRawBuffer[this->msgContentSize + MESSAGE_PRE_ENCODE_LENGTH] = MESSAGE_END_CHAR;
		this->msgRawBuffer[this->msgContentSize + MESSAGE_ENCODING_LENGTH] = '\0';
		
		// Properly initialized
		this->initialized = true;
	}
	
}

/**
 * @brief Get message content size
 * 
 * @return MessageType 
 */
size_t Message::getContentSize(void)
{
	return this->msgContentSize;
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
		memoryCopy(outBuffer, this->msgContentBuffer, this->msgContentSize);
		outBuffer[this->msgContentSize] = '\0';
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
		size_t rawLength = this->msgContentSize + MESSAGE_ENCODING_LENGTH;
		memoryCopy(outBuffer, this->msgRawBuffer, rawLength);
		outBuffer[rawLength] = '\0';
	}
}

/**
 * @brief Get message raw size
 * 
 * @return MessageType 
 */
size_t Message::getRawSize(void)
{
	return this->msgContentSize + MESSAGE_ENCODING_LENGTH;
}
