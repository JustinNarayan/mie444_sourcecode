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
	uint8_t* destAsByte = (uint8_t*)dest;
	uint8_t* srcAsByte = (uint8_t*)src;
	for(size_t i = 0; i < size; i++)
	{
		*(destAsByte + i) = *(srcAsByte + i);
	}
	return dest;
}
