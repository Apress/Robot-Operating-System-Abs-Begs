#!/usr/bin/env python


robot_command = raw_input("Enter the command:>  ")

if(robot_command == "move_left"):
	print "Robot is moving Left"
elif(robot_command == "move_right"):
	print "Robot is moving right"
elif(robot_command == "move_forward"):
	print "Robot is moving forward"
elif(robot_command == "move_backward"):
	print "Robot is moving backward"
else:
	print "Invalid command"




