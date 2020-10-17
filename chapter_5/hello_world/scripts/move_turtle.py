#!/usr/bin/env python

import rospy

from geometry_msgs.msg import Twist

import sys

def move_turtle(lin_vel,ang_vel):

    rospy.init_node('move_turtle', anonymous=True)
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10) # 10hz
 
    vel = Twist()
    while not rospy.is_shutdown():
        
	vel.linear.x = lin_vel
	vel.linear.y = 0
	vel.linear.z = 0

	vel.angular.x = 0
	vel.angular.y = 0
	vel.angular.z = ang_vel



        rospy.loginfo("Linear Vel = %f: Angular Vel = %f",lin_vel,ang_vel)

        pub.publish(vel)
    # to stop the turtle
    vel.linear.x = 0 
    vel.angular.z = 0 
    pub.publish(vel) 
    rospy.spin()

if __name__ == '__main__':
    try:
        move_turtle(float(sys.argv[1]),float(sys.argv[2]))
    except rospy.ROSInterruptException:
        pass
