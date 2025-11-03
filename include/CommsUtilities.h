#pragma once
#include <Arduino.h>

/**
 * @brief Format a string with custom arguments already in va_list form.
 *
 * @param buffer Buffer of size `size`
 * @param size Size of string buffer to create.
 * @param fmt Format string similar to printf (i.e. %d, %s).
 * @param args Arguments similar to printf in va_list form
 *
 * @note If a truncation was detected in the string, the first 3 characters will be replaced with a "~".
 */
void format(char *buffer, const char *fmt, va_list args)
{
	int bytesWritten = vsnprintf(buffer, (size_t)STRING_LENGTH_MAX, fmt, args);

	if ((size_t)bytesWritten >= STRING_LENGTH_MAX)
	{
		buffer[0] = '~'; // Indicates truncation
	}
}

/**
 * @brief Format a string with custom arguments not in va_list form
 *
 * @param buffer Buffer of size `size`
 * @param size Size of string buffer to create.
 * @param fmt Format string similar to printf (i.e. %d, %s).
 * @param ... Arguments similar to printf
 *
 * @note If a truncation was detected in the string, the first 3 characters will be replaced with a "~".
 */
void format(char *buffer, const char *fmt, ...)
{
	va_list args;
	va_start(args, fmt);
	int bytesWritten = vsnprintf(buffer, (size_t)STRING_LENGTH_MAX, fmt, args);
	va_end(args);

	if ((size_t)bytesWritten >= STRING_LENGTH_MAX)
	{
		buffer[0] = '~'; // Indicates truncation
	}
}