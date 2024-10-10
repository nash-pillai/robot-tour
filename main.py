from hub import light_matrix
from hub import motion_sensor
from hub import port
import runloop
import motor


import math

import time

# import sys
# sys.stdout = open('logs.txt','at')
# print()

right_motor = port.B
left_motor = port.A
motors = [left_motor, right_motor]

gyro = motion_sensor
gyro.reset_yaw(gyro.tilt_angles()[0])

heading_error = 0
last_heading_error = 0
heading_pid = {
	"kp": 20,
	"ki": 0,
	"kd": 0
}

turning_error = 0
last_turning_error = 0
turning_pid = {
	"kp": 20,
	"ki": 0,
	"kd": 0
}

start_time = time.time()
target_time = 59 + 1

# mm/s
slow_speed = 100
fast_speed = 1200

turn_speed = 1

remaining_distance = 0
remaining_turns = 0

degrees_per_tile = 360 * 2.85

current_pos = (0.5, 0)

async def main():
	print(0, 0, "Starting!")

	plan_path([{"x": current_pos[0], "y": current_pos[1]},
		{"x": 0.5, "y": 0.5}, # Move onto Field
		{"x": 2.5, "y": 0.5},
		{"x": 2.5, "y": 1.3}, # Gate B
		{"x": 2.5, "y": 0.5, "run_backwards": True},
		{"x": 3.5, "y": 0.5}, # Gate D
		{"x": 3.5, "y": 2.5},
		{"x": 2.7, "y": 2.5}, # Gate C
		{"x": 2.0, "y": 3.5},
		{"x": 0.5, "y": 3.5},
		{"x": 0.5, "y": 2.5},
		{"x": 1.3, "y": 2.5}, # Gate A
		{"x": 0.5, "y": 2.5, "run_backwards": True},
		{"x": 0.5, "y": 3.5}, # End
	])

	print(0, 50, "Done! -- " + str(time_elapsed()))
	print("Done!\n Time taken:", time.time() - start_time, "seconds (target:", target_time, "seconds)")
	time.sleep(10)

def approx_equal(a, b, tol):
	return abs(a - b) < tol

def time_elapsed():
	return time.time() - start_time

def clamp(minimum, n, maximum):
	return max(minimum, min(n, maximum))

def required_angular_speed(log=False):
	if log: print("Calculating speed with", round(remaining_distance, 3), "tiles and", round(target_time - time_elapsed(), 3), "seconds and", remaining_turns, "turns left")
	return clamp(slow_speed, (remaining_distance + 0.1 * remaining_turns) * degrees_per_tile / max(target_time - time_elapsed(), 1), fast_speed)

def angle_closest_dir(initial, target):
	diff = target - initial
	return sorted([diff, diff + math.copysign(360, -diff)], key=abs)[0]

def get_angle(): return gyro.tilt_angles()[0] % 360


def turn_to(target_angle, speed, tol):
	global turning_error, last_turning_error, turning_pid, remaining_turns
	target_angle %= 360

	print("Currently:", str(get_angle()), "deg and turning to:", str(target_angle), "deg")

	while not abs(angle_closest_dir(get_angle(), target_angle)) < tol:
		turning_error = angle_closest_dir(get_angle(), target_angle)
		turning_speed = turning_pid["kp"] * turning_error + turning_pid["ki"] * (turning_error + last_turning_error) + turning_pid["kd"] * (turning_error - last_turning_error)
		turning_speed = clamp(-400, turning_speed, 400)

		motor.run(left_motor, turning_speed * speed)
		motor.run(right_motor, -turning_speed * speed)

		last_turning_error = turning_error

	motor.stop(left_motor, stop=motor.HOLD)
	motor.stop(right_motor, stop=motor.HOLD)
	remaining_turns -= 1
	print("Finished Turning, final angle is:", str(get_angle()), "deg")


def drive_straight(distance, speed, target_angle=None):
	global heading_error, last_heading_error, heading_pid
	target_angle = target_angle if (target_angle is not None) else get_angle()

	motor.reset_relative_position(right_motor, 0)
	motor.reset_relative_position(left_motor, 0)

	while abs(motor.relative_position(left_motor)) < abs(distance * degrees_per_tile):
		heading_error = angle_closest_dir(get_angle(), target_angle)
		turning_speed = heading_pid["kp"] * heading_error + heading_pid["ki"] * (heading_error + last_heading_error) + heading_pid["kd"] * (heading_error - last_heading_error)
		turning_speed = clamp(-500, turning_speed, 500)

		motor.run(left_motor, -speed * (1 if distance < 0 else -1) + turning_speed)
		motor.run(right_motor, -speed * (1 if distance < 0 else -1) - turning_speed)

		last_heading_error = heading_error

	motor.stop(left_motor, stop=motor.HOLD)
	motor.stop(right_motor, stop=motor.HOLD)

def move_to(x: int, y: int, run_backwards=False, update_pos=True, angle_offset=0, distance_offset=0):
	global current_pos, remaining_distance

	if (current_pos == (x, y)):
		return

	target_angle = math.atan2(y - current_pos[1], x - current_pos[0]) * 360 / (2 * math.pi) - 90 + angle_offset
	if run_backwards:
		target_angle += 180

	turn_to(target_angle, turn_speed, 3)

	distance = math.sqrt((x - current_pos[0])**2 + (y - current_pos[1])**2) + distance_offset
	if not run_backwards: distance *= -1
	print("At", current_pos, "moving", round(distance, 3), "to", (x, y), "at", round(required_angular_speed(log=True), 3), "deg/s\n")

	drive_straight(distance, required_angular_speed(), target_angle)

	if update_pos:
		current_pos = (x, y)
		remaining_distance -= abs(distance - distance_offset)

def plan_path(segments: list[dict]):
	global current_pos, remaining_distance, remaining_turns

	remaining_distance = 0
	for i in range(1, len(segments)):
		remaining_distance += math.sqrt((segments[i]["x"] - segments[i - 1]["x"])**2 + (segments[i]["y"] - segments[i - 1]["y"])**2)

	# remaining_turns = len(list(filter(lambda a: a.get("run_backwards") != True, segments[1:])))
	remaining_turns = len(segments) - 1
	print("Starting at", current_pos, "with", remaining_distance, "tiles to go")
	current_pos = (segments[0]["x"], segments[0]["y"])
	for s in segments[1:]: move_to(**s)


runloop.run(main())