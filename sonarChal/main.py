#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile


# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.

# Initialize the motors.
left_motor = Motor(Port.B)
right_motor = Motor(Port.C)

# Initialize the sensors.
# sonic_sensor = UltrasonicSensor(Port.S1)
# gyro = GyroSensor(Port.S2)

# Initialize the drive base.
# robot = DriveBase(left_motor, right_motor, wheel_diameter=55.5, axle_track=104)

# while True:
#     dis = sonic_sensor.distance()
#     print(dis)