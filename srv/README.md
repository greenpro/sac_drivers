# Southern Arm Control Derivers > srv

This folder contains the source files for the service messages.

## Files
encoder.srv
* Contains the message definition for the encoder service in the USB Driver node.

motorPosition.srv
* Contains the message definition for the motor position service in the USB Driver node.

motorSpeed.srv
* Contains the message definition for the motor speed service in the USB Driver node.

motorsOffset.srv
* Contains the message definition for the motors offset service in the USB Driver node.

motorsComplete.srv
* Contains the message definition for the motors complete service in the USB Driver node.

## Folders
helpers/
* This folder contains the source files for the nodes.

## Notes
* The dual use of the *_driver.cpp files may be modified in the future to make either different source or .h files for each arm.
* Much of the code for the display driver comes from http://wiringpi.com/examples/adafruit-rgb-lcd-plate-and-wiringpi/ and the Wiring Pi library.
