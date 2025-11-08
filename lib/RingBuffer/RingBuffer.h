#pragma once
#include "Types.h"
#include "Settings.h"
#include "MemoryUtilities.h"

/**
 * Return codes
 */
#define RET_READ_BUFFER_NONE_TO_READ (-1)
#define RET_READ_BUFFER_SUCCESS (0)

#define RET_WRITE_BUFFER_SUCCESS (0)

#define RET_SEAL_BUFFER_EMPTY (-1)
#define RET_SEAL_BUFFER_SUCCESS (0)

class RingBuffer
{
private:
	/**
	 * Number of total ring buffers and size of each specified in "Settings.h"
	 */
	char buffers[MESSAGE_NUM_BUFFERS][STRING_LENGTH_MAX];

	/**
	 * currentBuffer stores the current buffer available to write to read, not
	 * ready to read.
	 * 
	 * numOccupiedBuffers stores the number of buffers ready to read. All of these
	 * buffers will precede currentBuffer, with the furthest back being the first
	 * available message to read.
	 */
	uint8_t currentBuffer;
	uint8_t numOccupiedBuffers;
	size_t numOccupiedBytesInCurrentBuffer;
	
	int sealBuffer(void);
	uint8_t getNumOccupiedBuffers(void);
public:
	RingBuffer() : currentBuffer(0), numOccupiedBuffers(0), numOccupiedBytesInCurrentBuffer(0)
	{
		memorySet(buffers, 0, sizeof(buffers));
	}

	bool isFull(void);
	int popBuffer(char* read_into);
	int writeIntoBuffer(const char* write_from, const size_t size, bool sealOnEndChar = true);
};