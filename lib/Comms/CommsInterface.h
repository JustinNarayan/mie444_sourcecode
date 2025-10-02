#pragma once
#include <stdarg.h>
#include "Errors.h"

enum StrLen {
    Default = 128,
    Long = 256
};

/**
 * Abstracted communications interface. Specified to be BLE or Serial via CommsConfig.
 */
class Comms {
public:
    virtual void init(unsigned long baud = 9600) = 0;
    virtual void send(char* buffer) = 0;

    void send_info(char* fmt, ...) {
        char buffer[StrLen::Default];
        va_list args;
        va_start(args, fmt);
        format(buffer, StrLen::Default, fmt, args);
        va_end(args);

        // Transmit
        this->send(buffer); 
    }

    void send_error(Error error) {
        this->send_info("Error [%d]: %s", error.code, error.message);
    }

    virtual ~Comms() = default;
};

/***************************************************
 *         H E L P E R   F U N C T I O N S         *
 ***************************************************/

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
void format(char* buffer, StrLen size, char* fmt, va_list args)
{
    int bytesWritten = vsnprintf(buffer, (size_t)size, fmt, args);

    if ((size_t)bytesWritten >= size)
    {
        buffer[0] = '~'; // Indicates truncation
    }
}