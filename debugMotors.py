#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor
from pybricks.parameters import Port

import time

ev3 = EV3Brick()

right_motor = Motor(Port.D)
left_motor = Motor(Port.A)

right_motor.run(1000)
left_motor.run(1000)

while True:
  ev3.screen.draw_text(0, 50, "Left Motor Angle:" + str(left_motor.angle()))
  ev3.screen.draw_text(0, 150, "Right Motor Angle:" + str(left_motor.angle()))
  time.sleep(.1)
  ev3.screen.clear()
