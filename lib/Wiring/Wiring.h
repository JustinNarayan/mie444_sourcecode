#pragma once
#include <Arduino.h>

/**
 * Include appropriate wiring file based on board
 */
#if defined(BOARD_CONTROLLER)
    #include "WiringController.h"
#elif defined(BOARD_DRIVETRAIN)
    #include "WiringDrivetrain.h"
#else
    #error "Unsupported board! Please defined BOARD_xxx in platformio.ini"
#endif

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
