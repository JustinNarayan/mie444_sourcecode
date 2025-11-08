#pragma once

/**
 * Define custom integer codes for all errors. This greatly simplifies debugging by
 * providing as much easy-to-encode specificity in failure states
 */
enum class ErrorCode {
    PinModeMissassigned,    // PinMode is not assigned as expected, something has gone wrong with Wiring.
    StrTooLong,             // Requested a string of excessive length for allocated space.
    InvalidParameter,       // Parameter for function is invalid.
    Unknwown                // Catch-all.
};

/**
 * Wrap error codes with additional debugging information.
 */
struct Error {
    ErrorCode code;
    const char* message;

    // Constructor
    Error(ErrorCode code, const char* message) : code(code), message(message) {}
};