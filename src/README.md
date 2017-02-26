# Southern Arm Control Derivers > src

This folder contains the source files for the nodes.

## Files
alpha_driver.cpp
* The alpha driver will control the first joint of the andreas arm and the base joint of the scorbot arm.
* This node will be compiled to alpha_driver for the andreas arm and base_joint_driver for the scorbot.

beta_driver.cpp
* The beta driver will control the second joint of the andreas arm and the shoulder joint of the scorbot arm.
* This node will be compiled to beta_driver for the andreas arm and shoulder_joint_driver for the scorbot.

gamma_driver.cpp
* The gamma driver will control the third joint of the andreas arm and the elbow joint of the scorbot arm.
* This node will be compiled to gamma_driver for the andreas arm and elbow_joint_driver for the scorbot.

delta_driver.cpp
* The delta driver will control the fourth joint of the andreas arm and the wrist elevation joint of the scorbot arm.
* This node will be compiled to delta_driver for the andreas arm and wrist_elevation_joint_driver for the scorbot.

epsilon_driver.cpp
* The epsilon driver will control the fifth joint of the andreas arm and the wrist roll joint of the scorbot arm.
* This node will be compiled to epsilon_driver for the andreas arm and wrist_roll_joint_driver for the scorbot.

zeta_driver.cpp
* The zeta driver will control the sixth joint of the andreas arm.
* This node will be compiled to zeta_driver for the andreas arm and.

display_driver.cpp
* The display driver will control the menu for the system and the writing to the display.
* The code for this comes from the Wiring Pi library.

serial_driver.cpp
* The serial driver will communicate to the serial interface of the scorbot.
* This driver is necessary to allow all of the nodes to communicate with the robot without conflicting. If they do not go through this node the writes to the usb file will conflict and possibly cause the nodes to crash.

## Folders
helpers/
* This folder contains the source files for the nodes.

## Notes
* The dual use of the *_driver.cpp files may be modified in the future to make either different source or .h files for each arm.
* Much of the code for the display driver comes from http://wiringpi.com/examples/adafruit-rgb-lcd-plate-and-wiringpi/ and the Wiring Pi library.
