#!/usr/bin/env python
"""
   dead_reckoning - Subscribe navigation goal from Rviz and move robot to the pose   
   
    Copyright (C) 2017 Lentin Joseph. 
     
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
"""
import rospy

import sys

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist 
import tf

from math import radians, degrees
import math



import signal
import sys


global robot_pose_x 
global robot_pose_y 
global robot_yaw 


global goal_x 
global goal_y 
global goal_yaw 

global turn_vel
global linear_vel

global stop_vel

robot_pose_x = 0.0
robot_pose_y = 0.0
robot_yaw = 0.0


goal_x = 0.0
goal_y = 0.0

goal_yaw = 0.0

turn_vel = 13
linear_vel = 0.8

stop_vel = 0


##########################################################################################

#Key board handler

def quit_code(signum, frame):

	global pub_twist

	global stop_vel
	# To reset the robot
	rospy.loginfo("Quitting code")

	for i in range(0, 100):

		send_turn_cmd(stop_vel)
		rospy.sleep(0.02)

	sys.exit(1)

signal.signal(signal.SIGINT, quit_code)

##########################################################################################




def send_turn_cmd(speed):

	global pub_twist

	twist = Twist()

        twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0

        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = speed

        pub_twist.publish(twist)




def send_linear_cmd(speed):

	global pub_twist

	twist = Twist()
        twist.linear.x = speed; twist.linear.y = 0; twist.linear.z = 0

        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0


        pub_twist.publish(twist)


def do_dead_recoking(goal_x,goal_y,yaw_degree):


	global turn_vel
	global linear_vel

	rospy.loginfo("Goal Position: [ %f, %f, %f ]"%(goal_x, goal_y, goal_yaw))

	rospy.loginfo("Robot Position: [ %f, %f, %f ]"%(robot_pose_x , robot_pose_y , robot_yaw))


	difference_angle = goal_yaw - robot_yaw

	if(difference_angle > 180):
		difference_angle  = 360 + difference_angle

		turn_vel = -turn_vel

	print difference_angle

	rospy.loginfo("Robot rotating along in the goal axis")


	while(abs(difference_angle) > 20):

		try:

			difference_angle = goal_yaw - robot_yaw

			distance = math.hypot(goal_x - robot_pose_x, goal_y - robot_pose_y)

			rospy.loginfo("Difference angle [%f], Distance [%f]" %(difference_angle, distance))

			
			send_turn_cmd(turn_vel)


			rospy.sleep(0.03)

		except:
			send_turn_cmd(stop_vel)
			rospy.sleep(2)
			sys.exit(0)	



	send_turn_cmd(stop_vel)
	rospy.sleep(2)


	rospy.loginfo("Robot moving to the goal point")

	distance = math.hypot(goal_x - robot_pose_x, goal_y - robot_pose_y)

	rospy.loginfo("Robot moving to the goal point [%f]" %(distance))


	while(distance > 0.08 and distance < 2):

		try:

			distance = math.hypot(goal_x - robot_pose_x, goal_y - robot_pose_y)

			difference_angle = goal_yaw - robot_yaw

			rospy.loginfo("Difference angle [%f], Distance [%f]" %(difference_angle, distance))

	
			send_linear_cmd(linear_vel)

			rospy.sleep(0.03)

		except:
			send_linear_cmd(stop_vel)
			rospy.sleep(2)
			sys.exit(0)	
			


	send_linear_cmd(stop_vel)
	rospy.sleep(2)

	rospy.loginfo("Robot reached the destination")




def odomCallback(msg):


    global robot_pose_x 
    global robot_pose_y 
    global robot_yaw 

    position = msg.pose.pose.position
    orientation = msg.pose.pose.orientation
    
    robot_pose_x = position.x
    robot_pose_y = position.y

    roll, pitch, yaw = tf.transformations.euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])

    yaw_degree = degrees(yaw)

    if(yaw_degree < 0):
	yaw_degree = 360 + yaw_degree

    robot_yaw = yaw_degree


#Callback to accept the goal pose
def poseCallback(msg):
	#print msg


	global goal_x 
	global goal_y 
	global goal_yaw 


    	position = msg.pose.position
    	quat = msg.pose.orientation

    	#rospy.loginfo("Point Position: [ %f, %f, %f ]"%(position.x, position.y, position.z))
    	#rospy.loginfo("Quat Orientation: [ %f, %f, %f, %f]"%(quat.x, quat.y, quat.z, quat.w))


	#Pose of robot

	goal_x = position.x

	goal_y = position.y


    	# Also print Roll, Pitch, Yaw
    	roll, pitch, yaw = tf.transformations.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])

	goal_yaw = degrees(yaw)

	if(goal_yaw < 0):
		goal_yaw = 360 + goal_yaw

    	#rospy.loginfo("Goal Yaw angle: %s"%str(yaw_degree))  

	#Do dead recokning
	do_dead_recoking(goal_x,goal_y,goal_yaw)


if __name__ == "__main__":
	rospy.init_node("Dead_reckoning_node")

	rospy.loginfo("Starting Dead recokning node")

	global pub_twist

	pub_twist = rospy.Publisher('/cmd_vel',Twist ,queue_size=1)


        rospy.Subscriber('/move_base_simple/goal', PoseStamped, poseCallback)
    
        rospy.Subscriber('/odom', Odometry, odomCallback)


	rospy.spin()
	'''
	while(True):
	    try:
		rospy.sleep(0.2)
		#pass

	    except:
		#pass
		sys.exit(0)
		
	'''

