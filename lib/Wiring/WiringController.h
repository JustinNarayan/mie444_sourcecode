#pragma once
#include <Arduino.h>
#include "Wiring.h"

/**
 * Define all used pins on Arduino Mega 2560
 */
#define PIN_BLINKER         LED_BUILTIN

/* Ultrasonic Sensors */
#define PIN_US_TEST_TRIGGER  PB7
#define PIN_US_TEST_ECHO     PB6

/**
 * @brief Initialize all wiring and pin modes for the main board.
 */
void Wiring_Init()
{
    // Blinker
    pinMode(PIN_BLINKER, OUTPUT);

    // Ultrasonic Sensors
    pinMode(PIN_US_TEST_TRIGGER, OUTPUT);
    pinMode(PIN_US_TEST_ECHO, INPUT);
}
