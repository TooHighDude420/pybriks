#!/usr/bin/env pybricks-micropython

"""
Example LEGO® MINDSTORMS® EV3 Robot Educator Color Sensor Down Program
----------------------------------------------------------------------

This program requires LEGO® EV3 MicroPython v2.0.
Download: https://education.lego.com/en-us/support/mindstorms-ev3/python-for-ev3

Building instructions can be found at:
https://education.lego.com/en-us/support/mindstorms-ev3/building-instructions#robot
"""

from pybricks.ev3devices import Motor, ColorSensor
from pybricks.parameters import Port
from pybricks.tools import wait
from pybricks.robotics import DriveBase

def sayVals(name, val):
    print(name + val)
    
def switcher(value):
    if (value > -1):
        wait(10)
        print("ik zie wit")
    elif(value < -31 and value > -39):
        wait(10)
        print("links")
    elif(value < -41):
        wait(10)
        print("ik zie zwart")
        
def printRGB(obj):
    return obj.rgb()

# Initialize the motors.
left_motor = Motor(Port.B)
right_motor = Motor(Port.C)

# Initialize the color sensor.
line_sensor = ColorSensor(Port.S3)

# Initialize the drive base.
robot = DriveBase(left_motor, right_motor, wheel_diameter=55.5, axle_track=104)

# Calculate the light threshold. Choose values based on your measurements.
BLACK = 8
WHITE = 9
threshold = (BLACK + WHITE) / 2

# Set the drive speed at 100 millimeters per second.
DRIVE_SPEED = 100
# TURN_SPEED = 30

# Set the gain of the proportional line controller. This means that for every
# percentage point of light deviating from the threshold, we set the turn
# rate of the drivebase to 1.2 degrees per second.

# For example, if the light value deviates from the threshold by 10, the robot
# steers at 10*1.2 = 12 degrees per second.
PROPORTIONAL_GAIN = 0.75

# Start following the line endlessly.
while True:
    # Calculate the deviation from the threshold.
    print(line_sensor.reflection())
    deviation = line_sensor.reflection() - threshold
    # print(printRGB(line_sensor))
    # Calculate the turn rate.
    turn_rate = PROPORTIONAL_GAIN * deviation
    # Set the drive base speed and turn rate.
    robot.drive(DRIVE_SPEED, turn_rate)

    # You can wait for a short time or do other things in this loop.
    wait(10)   