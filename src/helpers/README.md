# Southern Arm Control Derivers > src > helpers

This folder contains the helper include files for the nodes.

## Files
config.h
* This file contains the globally applicable #defines for the drivers. 

display.*
* This library holds the functions for initializing communicating with the display.

menuItem.h
* This library holds the structs necessary to hold the items for the menu.

## Folders
helpers/
* This folder contains the source files for the nodes.

## Notes
* The dual use of the *_driver.cpp files may be modified in the future to make either different source or .h files for each arm.
* Much of the code for the display library comes from http://wiringpi.com/examples/adafruit-rgb-lcd-plate-and-wiringpi/ and the Wiring Pi library.
