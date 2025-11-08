#include "Settings.h"
#include "Comms.h"

/**
 * @brief Initialize interface with Serial port.
 * 
 * @param baud 
 */
void Comms::init(HardwareSerial* port, unsigned long baud)
{
	this->port = port;
	port->begin(baud);
	while (!(*port))
	{
		;
	} // Allow time for Serial to connect
}

/**
 * @brief Send a general buffer of information over the communication interface
 * 
 * @param buffer A buffer
 */
void Comms::sendInfo(const char* buffer, size_t size)
{
	// Transmit
	this->send(buffer, size);
}

/**
 * @brief Receive information from the communication interface
 * 
 * @param buffer Buffer to store received information in
 * @param length Length of buffer, including null-terminator
 * 
 * @return Number of received bytes
 */
size_t Comms::receiveInfo(char* buffer, size_t length)
{
	size_t maxBytes = length - 1; // space for null_terminator
	size_t availableBytes = this->get_num_unread_bytes();
	size_t bytesToRead = min(maxBytes, availableBytes);

	// Loop through available bytes
	size_t i;
	for(i = 0; i < bytesToRead; i++)
	{
		buffer[i] = this->read_byte();
	}

	// Add null terminator
	buffer[i] = '\0';

	return i;
}
