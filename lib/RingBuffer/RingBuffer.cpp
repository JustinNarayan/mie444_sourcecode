#include "RingBuffer.h"
#include "MemoryUtilities.h"
/**
 * @brief Check if all message buffers are full.
 */
bool RingBuffer::isFull(void)
{
    return numOccupiedBuffers >= MESSAGE_NUM_BUFFERS;
}

/**
 * @brief Get number of buffers ready to read.
 */
uint8_t RingBuffer::getNumOccupiedBuffers(void)
{
    return numOccupiedBuffers;
}

/**
 * @brief Pop (read) the oldest complete message from the ring buffer.
 *
 * @param read_into Pointer to external buffer where message will be copied.
 * @return RET_READ_BUFFER_NONE_TO_READ if no buffers are ready,
 *         RET_READ_BUFFER_SUCCESS otherwise.
 */
int RingBuffer::popBuffer(char* read_into)
{
    if (numOccupiedBuffers == 0)
        return RET_READ_BUFFER_NONE_TO_READ;

    // Calculate the index of the oldest occupied buffer
    uint8_t oldestBuffer = (MESSAGE_NUM_BUFFERS + currentBuffer - numOccupiedBuffers) % MESSAGE_NUM_BUFFERS;

    // Copy message to the provided output buffer
    memoryCopy(read_into, buffers[oldestBuffer], MESSAGE_LENGTH_MAX);

	// Clear buffer
	memorySet(buffers[oldestBuffer], 0, MESSAGE_LENGTH_MAX);

    // Decrement the number of occupied buffers
    numOccupiedBuffers--;

    return RET_READ_BUFFER_SUCCESS;
}

/**
 * @brief Write data into the current active buffer (not yet sealed).
 *
 * @param write_from Null-terminated string to copy into the current buffer.
 * @param size Number of bytes to write
 * @param sealOnEndChar Whether to seal whenever an end char is reached
 * @return RET_WRITE_BUFFER_SUCCESS otherwise.
 */
int RingBuffer::writeIntoBuffer(
	const char* write_from, 
	const size_t size, 
	bool sealOnEndChar
)
{
	size_t remainingBytes = size;

	// Read into buffers until all bytes written
	while(remainingBytes > 0)
	{
		// Check if this is an end char
		bool encounteredEndChar = (*write_from) == MESSAGE_END_CHAR;

		// Copy char in and update indices
		buffers[currentBuffer][numOccupiedBytesInCurrentBuffer] = *write_from;
		numOccupiedBytesInCurrentBuffer++;
		write_from++;
		remainingBytes--;
		
		// Determine if should seal
		if(
			(encounteredEndChar && sealOnEndChar) || 
			(numOccupiedBytesInCurrentBuffer == MESSAGE_LENGTH_MAX - 1)
		)
		{
			sealBuffer();
			if (isFull())
				return remainingBytes;
		}
	}

    return RET_SEAL_BUFFER_SUCCESS;
}

/**
 * @brief Mark the current buffer as "ready to read" and advance to the next buffer.
 *
 * @return RET_SEAL_BUFFER_EMPTY if the buffer was empty (nothing written),
 *         RET_SEAL_BUFFER_SUCCESS otherwise.
 */
int RingBuffer::sealBuffer(void)
{
	// Don't seal if empty
    if (numOccupiedBytesInCurrentBuffer == 0)
	{
        return RET_SEAL_BUFFER_EMPTY;
	}

	// Add null terminator
	size_t lastIndex = (numOccupiedBytesInCurrentBuffer < MESSAGE_LENGTH_MAX - 1)
        ? numOccupiedBytesInCurrentBuffer
        : MESSAGE_LENGTH_MAX - 1;
	buffers[currentBuffer][lastIndex] = '\0';

    // Advance to next buffer index (wrap around)
    currentBuffer = (currentBuffer + 1) % MESSAGE_NUM_BUFFERS;
	numOccupiedBytesInCurrentBuffer = 0;

    if (numOccupiedBuffers < MESSAGE_NUM_BUFFERS)
	{
        numOccupiedBuffers++;
	}

    return RET_SEAL_BUFFER_SUCCESS;
}
