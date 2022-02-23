#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import sys
robot_x = 0
def pose_callback(msg):
    global robot_x
    #Reading x position from the Odometry message
    robot_x = msg.pose.pose.position.x
    rospy.loginfo("Robot X = %f\n",robot_x)
def move_turtle(lin_vel,ang_vel,distance):
    global robot_x
    rospy.init_node('move_turtlebot', anonymous=False)
    #The Twist topic is /cmd_vel_muc/input/teleop
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    #Position topic is /odom
    rospy.Subscriber('/odom',Odometry, pose_callback)
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
                vel.linear.x = 0
                vel.linear.z = 0
                break
        pub.publish(vel)
        rate.sleep()
if __name__ == '__main__':
    try:
        move_turtle(float(sys.argv[1]),float(sys.argv[2]),float(sys.argv[3]))
    except rospy.ROSInterruptException:
        pass

