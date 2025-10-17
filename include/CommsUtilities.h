#pragma once
#include <Arduino.h>

/**
 * @brief Format a string with custom arguments.
 *
 * @param buffer Buffer of size `size`
 * @param size Size of string buffer to create.
 * @param fmt Format string similar to printf (i.e. %d, %s).
 * @param args Arguments similar to printf
 *
 * @note If a truncation was detected in the string, the first 3 characters will be replaced with a "~".
 */
void format(char *buffer, const char *fmt, va_list args)
{
	int bytesWritten = vsnprintf(buffer, (size_t)MESSAGE_LENGTH_MAX, fmt, args);

	if ((size_t)bytesWritten >= MESSAGE_LENGTH_MAX)
	{
		buffer[0] = '~'; // Indicates truncation
	}
}