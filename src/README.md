# Southern Arm Control Derivers > src

This folder contains the source files for the nodes.

## Files
### alpha_driver.cpp
* The alpha driver will control the first joint of the Andreas Arm.

### beta_driver.cpp
* The beta driver will control the second joint of the Andreas Arm.

### gamma_driver.cpp
* The gamma driver will control the third joint of the Andreas Arm.

### delta_driver.cpp
* The delta driver will control the fourth joint of the Andreas Arm.

### epsilon_driver.cpp
* The epsilon driver will control the fifth joint of the Andreas Arm.

### zeta_driver.cpp
* The zeta driver will control the sixth joint of the Andreas Arm.

### base_driver.cpp
* The base driver will control the first joint of the scorbot.

### shoulder_driver.cpp
* The shoulder driver will control the second joint of the scorbot.

### elbow_driver.cpp
* The elbow driver will control the third joint of the scorbot.

### wrist_pitch_driver.cpp
* The wrist pitch driver will control the fourth joint of the scorbot.

### wrist_roll_driver.cpp
* The wrist roll driver will control the fifth joint of the scorbot.

### hand_driver.cpp
* The hand driver will control the motion of the gripper in both the andreas and scorbot arms.

### display_driver.cpp (not in use)
* The display driver will control the menu for the system and the writing to the display.
* The code for this is modified from the Wiring Pi library.
* Much of the code for the display driver comes from http://wiringpi.com/examples/adafruit-rgb-lcd-plate-and-wiringpi/ and the Wiring Pi library.

### usb_driver.cpp
* The serial driver will communicate to the serial USB interface of the scorbot.
* This driver is necessary to allow all of the nodes to communicate with the robot without conflicting. If they do not go through this node the writes to the usb file will conflict and possibly cause the nodes to crash. This node does not follow the ROS convention of doing the smallest necessary because of this convention.

## Folders
### helpers/
* This folder contains the source files for the nodes.

## Notes
* The encoders will be checked in their respective joint drivers.
