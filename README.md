# MIE444 Project Source Code
## Group
Justin Narayan, Alice Liu, Trent Rossos, Claire Zhang

## Setup
This is written using Platform.io for an Arduino Mega 2560.

## Getting Started
Everything should be run from the root of the folder `mie444_sourcecode`.

Update the `upload_port` in `platformio.ini` when flashing the board to reflect the COM port used by your computer.

`pio run` => Compile project
`pio run --target upload` or `pio run -t upload` => Compile project and upload firmware to board
`pio run --target clean` or `pio run -t clean` => Remove firmware from board