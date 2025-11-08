#pragma once
#include "Settings.h"
#include "Types.h"
#include <CommsInterface.h>
#include "Wiring.h"

/*****************************************************
 *                  PIN SELECTIONS                   *
 *****************************************************/

#define PIN_BLINKER LED_BUILTIN

/* UART */
#define UART_INTERNAL_TO_CONTROLLER (&Serial)

/* Encoders */
#define PIN_ENCODER_1_A 19
#define PIN_ENCODER_1_B 18
#define PIN_ENCODER_2_A 16
#define PIN_ENCODER_2_B 2
#define PIN_ENCODER_3_A 17
#define PIN_ENCODER_3_B 3
#define IS_INTERRUPT(p) ((p == 2) || (p == 3))

/*****************************************************
 *               PIN CHANGE INTERRUPTS               *
 *****************************************************/

#define PCINT_INVALID_PIN 0xFF

/* Valid Groups */
#define PCINT_GROUP_0 0 // Port B
#define PCINT_GROUP_1 1 // Port C
#define PCINT_GROUP_2 2 // Port D

/* Interrupt Base Offsets */
#define PCINT_BASE_B 0
#define PCINT_BASE_C 8
#define PCINT_BASE_D 16

/* Interrupt Vector Widths */
#define PCINT_WIDTH_B 8
#define PCINT_WIDTH_C 7
#define PCINT_WIDTH_D 8

/* Map pin to PCINT group */
#define PCINT_GROUP_FROM_PIN(pin) ( \
	((pin) >= 8 	&& (pin) <= 13) ? PCINT_GROUP_0 : \
	((pin) >= A0 	&& (pin) <= A5) ? PCINT_GROUP_1 : \
	((pin) <= 7) 					? PCINT_GROUP_2 : \
	PCINT_INVALID_PIN \
)

/* Map pin to PCINT index (0 - 23) */
#define PCINT_PIN(pin) ( \
    (PCINT_GROUP_FROM_PIN(pin) == PCINT_GROUP_0) ? ((pin) - 8) : \
    (PCINT_GROUP_FROM_PIN(pin) == PCINT_GROUP_1) ? ((pin) - A0 + 8) : \
    (PCINT_GROUP_FROM_PIN(pin) == PCINT_GROUP_2) ? ((pin) + 16) : \
    PCINT_INVALID_PIN \
)

/* Map pin to mask register pointer */
#define PCINT_MASK_REGISTER(pin) ( \
    (PCINT_GROUP_FROM_PIN(pin) == PCINT_GROUP_0) ? &PCMSK0 : \
    (PCINT_GROUP_FROM_PIN(pin) == PCINT_GROUP_1) ? &PCMSK1 : \
    (PCINT_GROUP_FROM_PIN(pin) == PCINT_GROUP_2) ? &PCMSK2 : \
    (uint8_t*)0 \
)

/* Declare ISR, required to call here for any port in use */
#define DECLARE_PCINT_ISR(vec, pinRegister, lastValue, offset, width) \
ISR(vec) { \
    uint8_t now = (pinRegister); \
    uint8_t changed = now ^ lastValue; \
    lastValue = now; \
    for (uint8_t i = 0; i < width; i++) { \
        if (changed & (1 << i)) { \
            void (*cb)(void) = pcintCallbacks[offset + i]; \
            if (cb) cb(); \
        } \
    } \
}

/**
 * @brief Declare all pin change interrupts for use throughout the Arduino UNO
 * 
 */
static void (*pcintCallbacks[24])(void) = {nullptr};
static volatile uint8_t lastValueB, lastValueC, lastValueD;
DECLARE_PCINT_ISR(PCINT0_vect, PINB, lastValueB, PCINT_BASE_B, PCINT_WIDTH_B)
DECLARE_PCINT_ISR(PCINT1_vect, PINC, lastValueC, PCINT_BASE_C, PCINT_WIDTH_C)
DECLARE_PCINT_ISR(PCINT2_vect, PIND, lastValueD, PCINT_BASE_D, PCINT_WIDTH_D)

/**
 * @brief Generate an Interrupt Service Routine on the UNO
 * In general, the board may not have enough dedicated Interrupt pins for all
 * interrupt needs. If the requested pin is a valid Interrupt pin, a normal
 * ISR is generated. If it is not an Interrupt pin, a Pin Change Interrupt
 * is instead used.
 * 
 */
bool Wiring_GenerateInterrutServiceRoutine(uint8_t pin, void (*isr)(void))
{
	// Verify requested pin is an input
	if (Wiring_ConfirmPinMode(pin, INPUT) == false)
		return false;
	
	// Use dedicated external interrupt if supported
	if (IS_INTERRUPT(pin))
	{
		uint8_t interruptPin = digitalPinToInterrupt(pin);
		if (interruptPin != NOT_AN_INTERRUPT)
		{
			attachInterrupt(interruptPin, isr, CHANGE);
			return true;
		}
		return false;
	}

	// Otherwise, use pin change interrupt
	uint8_t pcintPin = PCINT_PIN(pin);
	if (pcintPin == PCINT_INVALID_PIN) return false;
	uint8_t group = PCINT_GROUP_FROM_PIN(pin);
	
	// Enable PCINT group and pin bit within group mask
	PCICR |= _BV(group);
	volatile uint8_t *maskReg = PCINT_MASK_REGISTER(pin);
	*maskReg |= digitalPinToBitMask(pin);

	// Store callback ISR function
	pcintCallbacks[pcintPin] = isr;
	return true;
}


/*****************************************************
 *                   INITIALIZERS                    *
 *****************************************************/

/**
 * @brief Initialize all wiring and pin modes for the drivetrain board.
 */
void Wiring_InitPins()
{
	// Blinker
	pinMode(PIN_BLINKER, OUTPUT);
}

/**
 * @brief Initialize communications protocols
 * 
 */
void Wiring_InitComms(CommsInterface *controllerComms)
{
	// Internal communications setup
	controllerComms->init(UART_INTERNAL_TO_CONTROLLER, INTERNAL_COMMS_BAUD_RATE);
}
