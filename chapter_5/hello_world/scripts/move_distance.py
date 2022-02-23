#!/usr/bin/env python3

import rospy

from geometry_msgs.msg import Twist

from turtlesim.msg import Pose

import sys

robot_x = 0


def pose_callback(pose):
	global robot_x

	rospy.loginfo("Robot X = %f\n",pose.x)

	robot_x = pose.x


def move_turtle(lin_vel,ang_vel,distance):

    global robot_x

    rospy.init_node('move_turtle', anonymous=False)
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    
    rospy.Subscriber('/turtle1/pose',Pose, pose_callback)

    rate = rospy.Rate(10) # 10hz
 
    vel = Twist()
    while not rospy.is_shutdown():
        
        vel.linear.x = lin_vel
        vel.linear.y = 0
        vel.linear.z = 0

        vel.angular.x = 0
        vel.angular.y = 0
        vel.angular.z = ang_vel

        #rospy.loginfo("Linear Vel = %f: Angular Vel = %f",lin_vel,ang_vel)

        if(robot_x >= distance):
	        rospy.loginfo("Robot Reached destination")
	        rospy.logwarn("Stopping robot")

	        break

        pub.publish(vel)

        rate.sleep()

if __name__ == '__main__':
    try:
        move_turtle(float(sys.argv[1]),float(sys.argv[2]),float(sys.argv[3]))
    except rospy.ROSInterruptException:
        pass

