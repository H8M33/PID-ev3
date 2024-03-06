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


# Initialize the EV3 Brick.
ev3 = EV3Brick()

# Initialize the motors.
left_motor = Motor(Port.A)
right_motor = Motor(Port.B)

# Initialize the drive base.
robot = DriveBase(left_motor, right_motor, wheel_diameter=56, axle_track=134)

sensor = ColorSensor(Port.S4)
# Go forward and backwards for one meter.

WHITE = 30
BLACK = 0
DRIVE_SPEED = -60
set_point = (WHITE + BLACK) / 2
KP = 1
KD = 20
KI = 0.005
prev_err = 0
i = 0
to_del = []
per = 1
sum_int = 0
while True:
    sum_int += err
    to_del.append(err)
    i +=1
    if i == 1001:
        sum_int -=to_del[0]
        to_del.pop(0)
        i -=1
    if err * per < 0:
        sum_int = 0
        to_del = []
        per *= -1
        i = 0
    u = KP * err + KD * (err - prev_err) + KI*sum_int
    prev_err = err
    robot.drive(DRIVE_SPEED, u)
