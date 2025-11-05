#pragma once
#include <Arduino.h>
#include <Message.h>

#define ENUM_MAP_ENTRY(e, s) { static_cast<uint8_t>(e), s }
#define MAP_NUM_ELEMENTS(m) (size_t)(sizeof(m)/sizeof(m[0]))
#define COMPILE_TIME_ENFORCE_ENUM_MAP_COUNT(map, enum) \
	static_assert( \
		(sizeof(map)/sizeof(map[0])) == (uint8_t)(enum::Count), \
		#map " entries are not correctly mapped to all enums in " #enum \
	);

#define ENUM_MESSAGE_MAP_TRANSLATION(e) static EnumMessageMap<e, MAP_NUM_ELEMENTS(e##Map)> \
	e##Translation(MessageType::e, e##Map);

/**
 * Define an EnumStringMap to decode a string into an appropriate enum, given the type of 
 * MessageType.
 */
struct EnumStringMap {
	uint8_t enumValue;
	const char* strRep;
};

/* 
 * Define an EnumMessageMap to connect an EnumStringMap to a given MessageType.
 *
 */
template <typename T, size_t numElements>
class EnumMessageMap {
private:
	MessageType type;
	const EnumStringMap (&enumStringMap)[numElements];

	/**
	 * @brief Given a generic enum, get the string literal representation.
	 * 
	 * @param e An enum
	 * @return constexpr const char* String literal representation
	 */
	const char* enumToStr(T e) const {
		for (size_t i = 0; i < numElements; ++i)
			if (enumStringMap[i].enumValue == static_cast<uint8_t>(e))
				return enumStringMap[i].strRep;
		return "";
    }

	/**
	 * @brief Given a string, get the generic enum representation.
	 * 
	 * @param buffer A string
	 * @return constexpr T Generic enum representation
	 */
	const T strToEnum(const char* buffer) const {
    	if (!buffer || buffer[0] == '\0') return T::NoReceived;
        for (size_t i = 0; i < numElements; ++i)
            if (stringsEqual(enumStringMap[i].strRep, buffer))
                return static_cast<T>(enumStringMap[i].enumValue);
        return T::Invalid;
    }

public:
	EnumMessageMap(MessageType type, const EnumStringMap (&map)[numElements]) : type(type), enumStringMap(map) {};

	/**
	 * @brief Provided an enum, initialize and output a corresponding message.
	 * 
	 * @param e An enum to translate
	 * @param outMessage Message with appropriate type and content, now initialized
	 */
	void asMessage(const T e, Message* outMessage)
	{
		outMessage->init(type, enumToStr(e));
	}

	/**
	 * @brief Provided a message, return a corresponding enum.
	 * 
	 * @param message A message to translate
	 * @return T An appropriate enum
	 */
	T asEnum(Message* message)
	{
		char buffer[MESSAGE_CONTENT_LENGTH_MAX];
		message->getContent(buffer);
		return strToEnum(buffer);
	}
};
