## Purpose
This package implements the drivers for the system to communicate with the hardware. Some of 
these nodes may communicate with the microcontroller driver to send messages to allow for
serial communication.

## Files
**Alpha Driver**
        - File Name: alpha_driver.cpp
        - Node Name: alpha_driver
        - Purpose: This driver should take in the alpha motor messages, run validity checks, and send them to the microcontroller.
        - Input Messages:
            - "alphaDriver"
                - Type: std_msgs/UInt64MultiArray.msg
                - Notes: The first element of the array is the motor angle in radians, the second element is the time.
        - Output Messages:
            - "serialDriver"
                - Type: std_msgs/UInt64MultiArray.msg
                - Notes: The first element of the array is the motor number (0), the second element is the angle, and the third element is the time.
            - "

**Beta Driver**
        - File Name: beta_driver.cpp
        - Node Name: beta_driver
        - Purpose: This driver should take in the alpha motor messages, run validity checks, and send them to the microcontroller.
        - Input Messages:
            - BetaMotor
                - Type: std_msgs/UInt64MultiArray.msg
                - Notes: The first element of the array is the motor angle in radians, the second element is the time.
        - Output Messages:
            - Microcontroller
                - Type: std_msgs/UInt64MultiArray.msg
                - Notes: The first element of the array is the motor number (0), the second element is the angle, and the third element is the time.

**Gamma Driver**
        - File
            - gamma_driver.cpp
        - Node Name
            - gamma_driver
        - Input Messages
            - GammaMotor
                - Type
                    - std_msgs/UInt64MultiArray.msg
                - Notes
                    - The first element of the array is the motor angle in radians, the second element is the time.
        - Output Messages
            - Microcontroller
                - Type
                    - std_msgs/UInt64MultiArray.msg
                - Notes
                    - The first element of the array is the motor number (0), the second element is the angle, and the third element is the time.
        - Purpose
            - This driver should take in the alpha motor messages, run validity checks, and send them to the microcontroller.

**Delta Driver**
        - File
            - delta_driver.cpp
        - Node Name
            - delta_driver
        - Input Messages
            - DeltaMotor
                - Type
                    - std_msgs/UInt64MultiArray.msg
                - Notes
                    - The first element of the array is the motor angle in radians, the second element is the time.
        - Output Messages
            - Microcontroller
                - Type
                    - std_msgs/UInt64MultiArray.msg
                - Notes
                    - The first element of the array is the motor number (0), the second element is the angle, and the third element is the time.
        - Purpose
            - This driver should take in the alpha motor messages, run validity checks, and send them to the microcontroller.

**Epsilon Driver**
        - File
            - epsilon_driver.cpp
        - Node Name
            - epsilon_driver
        - Input Messages
            - EpsilonMotor
                - Type
                    - std_msgs/UInt64MultiArray.msg
                - Notes
                    - The first element of the array is the motor angle in radians, the second element is the time.
        - Output Messages
            - Microcontroller
                - Type
                    - std_msgs/UInt64MultiArray.msg
                - Notes
                    - The first element of the array is the motor number (0), the second element is the angle, and the third element is the time.
        - Purpose
            - This driver should take in the alpha motor messages, run validity checks, and send them to the microcontroller.

**Zeta Driver**
        - File
            - zeta_driver.cpp
        - Node Name
            - zeta_driver
        - Input Messages
            - ZetaMotor
                - Type
                    - std_msgs/UInt64MultiArray.msg
                - Notes
                    - The first element of the array is the motor angle in radians, the second element is the time.
        - Output Messages
            - Microcontroller
                - Type
                    - std_msgs/UInt64MultiArray.msg
                - Notes
                    - The first element of the array is the motor number (0), the second element is the angle, and the third element is the time.
        - Purpose
            - This driver should take in the alpha motor messages, run validity checks, and send them to the microcontroller.

## Reference
    - This package implements the drivers shown in the RosTopology.vxdx
    - The reference fot the message types can be found at http://wiki.ros.org/std_msgs
