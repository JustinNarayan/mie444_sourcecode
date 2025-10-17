#pragma once
#include <Arduino.h>

/**
 * Abstracted communications interface. Specified to be BLE or Serial via CommsConfig.
 */
class Comms
{
private:
	/**
	 * Each Comms uses a single Serial connection
	 */
	HardwareSerial *port;
	
	/**
	 * @brief Write to the Serial port
	 * 
	 * @param buffer 
	 */
	void send(const char *buffer)
	{
		port->write(buffer);
	}
	
	/**
	 * @brief Get the number of available bytes on the Serial connection
	 * 
	 * @return Number of bytes
	 */
	size_t get_num_unread_bytes()
	{
		return (size_t)(port->available());
	}
	
	/**
	 * @brief Read the first available byte from the port
	 * 
	 * @return First available byte
	 */
	char read_byte()
	{
		return port->read();
	}

public:
	void init(HardwareSerial* port, unsigned long baud);
	void sendInfo(const char *buffer);
	size_t receiveInfo(char* buffer, size_t length);
};
