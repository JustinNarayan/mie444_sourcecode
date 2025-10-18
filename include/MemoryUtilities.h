#pragma once
#include <Arduino.h>

/**
 * @brief Set everything in element to value
 * 
 * @param element
 * @param value
 * @param size 
 * @return void* Updated element
 */
static inline void* memorySet(void* ptr, const uint8_t value, const size_t size)
{
	if (ptr == NULL)
		return ptr;

	uint8_t* ptrAsByte = (uint8_t*)ptr;
	for (size_t i = 0; i < size; i++)
	{
		*(ptrAsByte + i) = value;
	}
	return ptr;
}

/**
 * @brief Copy all elements in source into destination
 * 
 * @param dest 
 * @param src 
 * @param size 
 * @return void* Updated destination
 */
static inline void* memoryCopy(void* dest, const void* src, const size_t size)
{
	if ((dest == NULL) || (src == NULL))
		return dest;

	uint8_t* destAsByte = (uint8_t*)dest;
	uint8_t* srcAsByte = (uint8_t*)src;
	for(size_t i = 0; i < size; i++)
	{
		*(destAsByte + i) = *(srcAsByte + i);
	}
	return dest;
}

/**
 * @brief Get string length. Ensure strings are null-terminated before use.
 * 
 * @param item 
 * @return size_t 
 */
static inline size_t stringLength(const void* item)
{
	if (item == NULL)
		return (size_t)0;

	for (size_t i = 0; i < MESSAGE_LENGTH_MAX; i++)
	{
		char here = *((uint8_t *)(item) + i);
		if (here == '\0')
			return i;
	}
	
	return (size_t)-1;
}

/**
 * @brief Compare the elements of two strings and return True if they are the same. Ensure strings 
 * are null-terminated before use.
 * 
 * @param item1 
 * @param item2 
 * @return bool
 */
static inline bool stringCompare(const void* item1, const void* item2)
{
	if (item1 == NULL || item2 == NULL)
        return false;

	const char *s1 = (const char *)item1;
    const char *s2 = (const char *)item2;

	// Continue until either null-terminator
	while (*s1 != '\0' && *s2 != '\0')
	{
		// Fail if not the same
		if (*s1 != *s2)
            return false;

		// Increment
		s1++;
        s2++;
	}
	
	// Only the same if both strings are now at null-terminated
	return *s1 == *s2;
}
