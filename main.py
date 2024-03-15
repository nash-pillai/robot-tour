#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor
from pybricks.ev3devices import GyroSensor
from pybricks.parameters import Port, Direction

import math

# import numpy as np

import time

ev3 = EV3Brick()

right_motor = Motor(Port.D)
left_motor = Motor(Port.A)
motors = [left_motor, right_motor]

gyro = GyroSensor(Port.S4, Direction.COUNTERCLOCKWISE)
gyro.reset_angle(0)

heading_error = 0
last_heading_error = 0
heading_pid= {
	"kp": 20,
	"ki": 0,
	"kd": 0
}

start_time = time.time()
target_time = 52

current_pos = (0.5, 0)

# mm/s
slow_speed = 600
fast_speed = 600

fast_turn_speed = 300
slow_turn_speed = 100

remaining_distance = 0

degrees_per_tile = 360 * 4.1

def main():
	move_to(current_pos[0], current_pos[1] + 0.5)
	move_to(3.5, 1.5)
	move_to(0.5, 2.5)
	move_to(3.5, 3.5)
	move_to(0.5, 0.5)

	# plan_path([{"x": current_pos[0], "y": current_pos[1]}, # Only 90s
	# 					{"x": 3.5, "y":2.5},
	# 					{"x": 2.5, "y":1.5, "run_backwards": True},
	# ])

	print("Done!\n Time taken:", time.time() - start_time, "seconds (target:", target_time, "seconds)")

def approx_equal(a, b, tol): return abs(a - b) < tol

def time_elapsed(): return time.time() - start_time

def clamp(minimum, n, maximum): return max(minimum, min(n, maximum))

def required_angular_speed(log=False):
	if log: print("Calculating speed with", round(remaining_distance, 3), "tiles and", round(target_time - time_elapsed(), 3), "seconds left")
	return clamp(slow_speed, remaining_distance * degrees_per_tile / max(target_time - time_elapsed(), 1), fast_speed)

def angle_closest_dir(initial, target):
	diff = target - initial
	return sorted([diff, diff + math.copysign(360, -diff)], key=abs)[0]

def get_angle(): return gyro.angle() % 360


def turn_to(target_angle, speed, tol):
	target_angle %= 360
	turning_dir = math.copysign(1, angle_closest_dir(get_angle(), target_angle))

	left_motor.run(speed * turning_dir)
	right_motor.run(speed * -turning_dir)
	
	print("Currently:", str(get_angle()), "deg and turning to:", str(target_angle), "deg")
	while not approx_equal(get_angle(), target_angle, tol): 
		pass
		# print("Currently: " + str(get_angle()) + " deg and Turning to: " + str(target_angle) + " deg\r", end="")

	left_motor.hold()
	right_motor.hold()

def drive_straight(distance, speed):
	global heading_error, last_heading_error, heading_pid
	start_angle = get_angle()

	right_motor.reset_angle(0)
	left_motor.reset_angle(0)

	while abs(left_motor.angle()) < abs(distance * degrees_per_tile): 
		heading_error = angle_closest_dir(get_angle(), start_angle)
		turning_speed = heading_pid["kp"] * heading_error + heading_pid["ki"] * (heading_error + last_heading_error) + heading_pid["kd"] * (heading_error - last_heading_error)
		turning_speed = clamp(-500, turning_speed, 500)

		left_motor.run(-speed + turning_speed)
		right_motor.run(-speed - turning_speed)

		last_heading_error = heading_error

	left_motor.hold()
	right_motor.hold()

def move_to(x: int, y: int, run_backwards=False, update_pos=True, angle_offset=0, distance_offset=0):
	global current_pos, remaining_distance

	if (current_pos == (x, y)): return
	
	target_angle = math.atan2(y - current_pos[1], x - current_pos[0]) * 360 / (2 * math.pi) - 90 + angle_offset
	if run_backwards: target_angle += 180

	if time_elapsed() > target_time * 0.85: turn_to(target_angle, fast_turn_speed, 15)
	time.sleep(0.2)
	turn_to(target_angle, slow_turn_speed, 3)

	distance = math.sqrt((x - current_pos[0])**2 + (y - current_pos[1])**2) + distance_offset
	if not run_backwards: distance *= -1
	print("At", current_pos, "moving", round(distance, 3), "to", (x, y), "at", round(required_angular_speed(log=True), 3), "deg/s\n")

	drive_straight(distance, required_angular_speed())

	if update_pos: 
		current_pos = (x, y)
		remaining_distance -= abs(distance - distance_offset)

def plan_path(segments: list[dict]):
	global current_pos, remaining_distance

	remaining_distance = 0
	for i in range(1, len(segments)):
		remaining_distance += math.sqrt((segments[i]["x"] - segments[i - 1]["x"])**2 + (segments[i]["y"] - segments[i - 1]["y"])**2)
	
	print("Starting at", current_pos, "with", remaining_distance, "tiles to go")
	current_pos = (segments[0]["x"], segments[0]["y"])
	for s in segments[1:]: move_to(**s)

if __name__ == '__main__': main()
