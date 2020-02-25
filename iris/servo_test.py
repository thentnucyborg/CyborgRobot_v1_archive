#!/usr/bin/env python2
# Import the dynamixel library
import dynamixel
# Import the time library for it's sleep functionality
import time

# Open a connection to the USB2Dynamixel device
serial = dynamixel.SerialStream(port="/dev/ttyUSB0",
                                baudrate=1000000,
                                timeout=1)
# Configure a DynamixelNetwork
net = dynamixel.DynamixelNetwork(serial)

# Configure the motor servo actuator
actuator = dynamixel.Dynamixel(1, net)
net._dynamixel_map[1] = actuator

# Configure the actuator such that it is in free running mode, and at a standstill
# The speed of the motor is configured in steps
# The values 0-1023 sets the motor to rotate clockwise at the given speed (1023 max)
# The values 1024-2047 sets the motor to rotate counterclockwise at the given speed - 1024
# The value 1024 represents stopped
actuator.moving_speed = 1024
# Enable torque for the motor
actuator.torque_enable = True
# Set the maximum torque to maximum
actuator.torque_limit = 1023
actuator.max_torque = 1023
# Set the goal position to 512 to enable free running mode
actuator.goal_position = 512

# Synchronize the settings
net.synchronize()


while True:
    # Start opening the IRIS
    actuator.moving_speed = 1000
    net.synchronize()
    time.sleep(0.355)

    actuator.moving_speed = 1024
    net.synchronize()

    # The IRIS is now open
    time.sleep(4)

    # Start closing the IRIS
    actuator.moving_speed = 1024+1000
    net.synchronize()
    time.sleep(0.355)

    actuator.moving_speed = 1024
    net.synchronize()

    # The IRIS is now closed
    time.sleep(4)
