# MIE444 Project Source Code
## Group
Justin Narayan, Alice Liu, Trent Rossos, Claire Zhang

## Setup
This is written using Platform.io for an Arduino Mega 2560 and Arduino UNO.

## Getting Started
Everything should be run from the root of the folder `mie444_sourcecode`.

Update the `upload_port` in `platformio.ini` when flashing the board to reflect the COM port used by your computer.

`pio run` => Compile project
`pio run --target upload -e <env>` or `pio run -t upload -e <env>` => Compile project and upload firmware to board
`pio run --target clean -e <env>` or `pio run -t clean -e <env>` => Remove firmware from board

### Example
`pio run -t upload -e controller` => Upload code to MEGA

## File Structure
### Communications
Based on the current communications setup (Serial or Bluetooth), modify the #define in lib/Wiring/WiringController.h