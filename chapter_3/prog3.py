#!/usr/bin/env python


robot_x = 0.1
robot_y = 0.1

while (robot_x < 2 and robot_y < 2):
	robot_x += 0.1
	robot_y += 0.1

	print ("Current Position ",robot_x,robot_y)


print ("Reached destination")
