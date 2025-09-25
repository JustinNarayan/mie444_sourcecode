/**
 * Includes
 */
#include <Arduino.h>

#include "Wiring.h"
#include "Errors.h"
#include "CommsConfig.h" // either USE_SERIAL or USE_BLE

/**
 * Global objects
 */
CommsActive comms;

/**
 * @brief Global setup functions for board
 */
void setup() {
  Wiring_Init();
  comms.init(9600);
}

/**
 * @brief Runtime loop for board
 */
void loop() {
  delay(1000);
}