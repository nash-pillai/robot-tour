#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import GyroSensor
from pybricks.parameters import Port, Direction

import time

ev3 = EV3Brick()

gyro = GyroSensor(Port.S1, Direction.COUNTERCLOCKWISE)
gyro.reset_angle(0)

while True:
  ev3.screen.draw_text(0, 50, "Angle: " + str(gyro.angle()))
  time.sleep(.1)
  ev3.screen.clear()
