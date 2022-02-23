#!/usr/bin/env python


robot_x = 0.1
robot_y = 0.1


for i in range(0,100):
        robot_x += 0.1
        robot_y += 0.1

        print ("Current Position ",robot_x,robot_y)

        if(robot_x > 2 and robot_y > 2):
      	        print ("Reached destination")
      	        break
