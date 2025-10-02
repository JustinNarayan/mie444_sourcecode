#pragma once
#include "Comms.h"

/**
 * Select one of the following two communications protocols based on current setup.
 */
#define USE_SERIAL
// #define USE_BLE

/**
 * Determine active communications protocol
 */
#ifdef USE_SERIAL
#include "CommsSerial.h"
using CommsActive = CommsSerial;
#elif defined(USE_BLE)
#include "CommsBLE.h"
using CommsActive = CommsBLE;
#else
#error "No communication protocol defined."
#endif