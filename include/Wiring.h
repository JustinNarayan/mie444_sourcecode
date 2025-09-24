#pragma once
#include <Arduino.h>

/**
 * Define all used pins on Arduino Mega 2560
 */
#define PIN_BLINKER         LED_BUILTIN

/* Ultrasonic Sensors */
#define PIN_US_TEST_TRIGGER  PB7
#define PIN_US_TEST_ECHO     PB6

/**
 * @brief Initialize all wiring and pin modes for the board.
 */
void Wiring_Init()
{
    // Blinker
    pinMode(PIN_BLINKER, OUTPUT);

    // Ultrasonic Sensors
    pinMode(PIN_US_TEST_TRIGGER, OUTPUT);
    pinMode(PIN_US_TEST_ECHO, INPUT);
}

/**
 * @brief Confirm pin is set to expected mode.
 * 
 * @param pin Arduino pin to check (i.e. PB7)
 * @param mode Expected mode (i.e. OUTPUT)
 * @return Whether observed mode for pin matches expected mode
 */
bool Wiring_ConfirmPinMode(uint8_t pin, uint8_t mode)
{
    uint8_t observedMode;

    uint8_t bit = digitalPinToBitMask(pin);
    uint8_t port = digitalPinToPort(pin);

    // Check behaviour-determining registers for bit on port
    volatile uint8_t *reg = portModeRegister(port);
    if (*reg & bit) 
    {
        observedMode = OUTPUT;
    }
    else
    {
        volatile uint8_t *out = portOutputRegister(port);
        observedMode = ((*out & bit) ? INPUT_PULLUP : INPUT);
    }
    return observedMode == mode;
}
