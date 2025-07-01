#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
    
def save_obj(distance, angle):
    if (farthest_distance == 0 or distance > farthest_distance):
        farthest_distance = distance
        obj_list_distance.insert(0, distance)
        obj_list_angle.insert(0, angle)
    else:
        obj_list_distance.append(distance)
        obj_list_angle.append(angle)
 
for i in range(len(obj_list_distance)):
    print(obj_list_distance)
    
# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.

# Initialize the motors.
arm = Motor(Port.A, Direction.CLOCKWISE, [12, 36])
left_motor = Motor(Port.B)
right_motor = Motor(Port.C)

# setting variables
DRIVE_SPEED = 100
CLOSE_RANGE = 120
FAR = 600
loop = True

# Initialize the sensors.
sonic_sensor = UltrasonicSensor(Port.S1)
gyro = GyroSensor(Port.S2) 

# Initialize the drive base.
robot = DriveBase(left_motor, right_motor, wheel_diameter=55.5, axle_track=104)

# Rest van code gaat hier

# turning 90 dergrees

while arm.angle() > 80:
    arm.drive(100.0)
 
# while sonic_sensor.distance() > 30:
#     robot.drive(250, 0)
 
# robot.stop()
# left_motor.brake()
# right_motor.brake()
# arm.reset_angle()

arm.dc(-50)

# while loop:
#     dis = sonic_sensor.distance()
#     if (dis < CLOSE_RANGE):
#         print("iets dichtbij me!")
#         print(dis)
        
#         if (dis <= 75):
#             robot.stop()
#             left_motor.brake()
#             right_motor.brake()
#         else:
#             robot.drive(100, 0)
                

#     elif (dis > CLOSE_RANGE and dis < FAR):
#         print("Er staat iets in midrange")
#         print(dis)
#         robot.drive(100, 0)

#     elif (dis > FAR):
#         print("geen object of te ver!")
#         print(dis)
#         gyro.reset_angle(0)

#         rotate = True
        
#         while rotate:
#             turneddis = sonic_sensor.distance()
            
#             angle = gyro.angle()
    
#             if (angle > 90 or turneddis < FAR):
#                 rotate = False
        
#             print(gyro.angle())
#             robot.drive(20, 50)