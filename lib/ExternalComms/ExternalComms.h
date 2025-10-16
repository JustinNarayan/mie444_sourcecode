#pragma once

/**
 * Select one of the following two communications protocols based on current setup.
 */
// #define USE_SERIAL
#define USE_BLE

/**
 * Determine active communications protocol
 */
#ifdef USE_SERIAL
#include "ExternalCommsSerial.h"
using ExternalCommsActive = ExternalCommsSerial;
#elif defined(USE_BLE)
#include "ExternalCommsBLE.h"
using ExternalCommsActive = ExternalCommsBLE;
#else
#error "No communication protocol defined."
#endif