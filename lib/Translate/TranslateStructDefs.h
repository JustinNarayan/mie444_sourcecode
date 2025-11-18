#pragma once
#include "Types.h"
#include <Message.h>

/*****************************************************
 *                 COMPILER UTILITIES                *
 *****************************************************/
#define COMPILE_TIME_ENFORCE_STRUCT_SIZE(struct) \
    static_assert( \
        sizeof(struct) <= MESSAGE_CONTENT_LENGTH_MAX, \
        #struct " size exceeds MESSAGE_CONTENT_LENGTH_MAX" \
    )

#define STRUCT_MESSAGE_MAP_TRANSLATION(s) \
    static StructMessageMap<s> s##Translation(MessageType::s); \
    /* Declare the unique deserialization function */ \
    template<> \
    void StructMessageMap<s>::strToStruct(s*, const char*) const;

/* 
 * Define a StructMessageMap to connect an struct to a given MessageType.
 *
 */
template <typename S>
class StructMessageMap
{
private:
	MessageType type;
	size_t size;

	/**
	 * @brief Given a generic value, get the char reprsentation (i.e. as bytes).
	 * 
	 * @param s A pointer struct of type S
	 * @param out A buffer of chars, of known final size based on sizeof(S).
	 */
	void structToStr(const S *s, char* out) const {
		memoryCopy(out, s, this->size);
    }

	/**
	 * @brief Given a string (i.e. as bytes), get the struct representation.
	 * 
	 * @param s A pointer to struct of type S
	 * @param buffer A string, of known initial size based on sizeof(S).
     * 
     * Must be instantiated by each individual class. See Translate.h
	 */
	void strToStruct(S *s, const char* buffer) const = delete;

public:
	StructMessageMap(MessageType type) : type(type), size(sizeof(S)) {};

	/**
	 * @brief Provided a pointer to a struct, initialize and output a corresponding message.
	 * 
	 * @param s A pointer to a struct to translate
	 * @param outMessage Message with appropriate type and content, now initialized
	 */
	void asMessage(const S *s, Message* outMessage)
	{
		char buffer[MESSAGE_CONTENT_LENGTH_MAX];
		structToStr(s, buffer);
		outMessage->init(type, this->size, buffer);
	}

	/**
	 * @brief Provided a message and pointer to a struct, populate the struct with appropriate 
	 * information.
	 * 
	 * @param message A message to translate
	 * @param s pointer to a struct
	 */
	void asStruct(Message* message, S *s)
	{
		char buffer[MESSAGE_CONTENT_LENGTH_MAX];
		message->getContent(buffer);
		strToStruct(s, buffer);
	}
};
