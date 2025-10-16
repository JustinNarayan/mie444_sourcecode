#pragma once
#include <stdarg.h>
#include "Settings.h"
#include "Errors.h"
#include "CommsUtilities.h"

/**
 * Abstracted communications interface. Specified to be BLE or Serial via CommsConfig.
 */
class ExternalComms
{
public:
	virtual void init(unsigned long baud = EXTERNAL_COMMS_BAUD_RATE) = 0;
	virtual void send(const char *buffer) = 0;
	virtual size_t get_num_unread_bytes(void) = 0;
	virtual char read_byte(void) = 0;

	/**
	 * @brief Send a general buffer of information over the communication interface
	 * 
	 * @param fmt A format string
	 * @param ... Parameters for format string
	 */
	void sendInfo(const char *fmt, ...)
	{
		char buffer[StrLen::Default];
		va_list args;
		va_start(args, fmt);
		format(buffer, StrLen::Default, fmt, args);
		va_end(args);

		// Transmit
		this->send(buffer);
	}

	/**
	 * @brief Send a generated error over the communication interface
	 * 
	 * @param error Error defined in Errors.h
	 */
	void sendError(Error error)
	{
		this->sendInfo("Error [%d]: %s", error.code, error.message);
	}

	/**
	 * @brief Receive information from the communication interface
	 * 
	 * @param buffer Buffer to store received information in
	 * @param length Length of buffer, including null-terminator
	 * 
	 * @return Number of received bytes
	 */
	size_t receiveInfo(char* buffer, StrLen length)
	{
		size_t max_bytes = length - 1; // space null_terminator
		size_t available_bytes = this->get_num_unread_bytes();
		size_t bytes_to_read = min(max_bytes, available_bytes);

		// Loop through available bytes
		size_t i;
		for(i = 0; i < bytes_to_read; i++)
		{
			buffer[i] = this->read_byte();
		}

		// Add null terminator
		buffer[i] = '\0';

		return i;
	}

	virtual ~ExternalComms() = default;
};
