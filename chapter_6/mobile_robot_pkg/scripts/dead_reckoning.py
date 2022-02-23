#!/usr/bin/env python
"""
   dead_reckoning - Subscribe navigation goal from Rviz and move robot to the pose   
   
    Copyright (C) 2021 Lentin Joseph. 
     
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

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
import tf

from math import radians, degrees
import math

import signal
import sys

class DeadReckoning:

    def __init__(self):

        self.goal_x = 0.0
        self.goal_y = 0.0
        self.goal_yaw = 0.0

        self.robot_pose_x = 0.0
        self.robot_pose_y = 0.0
        self.robot_yaw = 0.0

        self.stop_vel = 0
        self.delta_angle = 45

        self.turn_vel = 15
        self.linear_vel = 1

        self.delta_distance = 0.4
        self.obstacle_distance = 50

        self.obstacle_distance_upper_limit = 40
        self.obstacle_distance_lower_limit = 5

        #Publish command velocity
        self.pub_twist = rospy.Publisher('/cmd_vel', Twist, queue_size=1)        
        #Subscriber of Move base goal command
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.poseCallback)
        #Odom callback
        rospy.Subscriber('/odom', Odometry, self.odomCallback)
        #Obstacle callback
        rospy.Subscriber('/obstacle_distance', Float32, self.obstacleCallback)

        signal.signal(signal.SIGINT, self.shutdown)
        pass

    def odomCallback(self,msg):

        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation

        self.robot_pose_x = position.x
        self.robot_pose_y = position.y

        roll, pitch, yaw = tf.transformations.euler_from_quaternion(
            [orientation.x, orientation.y, orientation.z, orientation.w])

        yaw_degree = degrees(yaw)

        if(yaw_degree < 0):
            yaw_degree = 360 + yaw_degree

        self.robot_yaw = yaw_degree

        rospy.loginfo(msg)


    def poseCallback(self,msg):

        position = msg.pose.position
        quat = msg.pose.orientation

        rospy.loginfo("Point Position: [ %f, %f, %f ]"%(position.x, position.y, position.z))
        rospy.loginfo("Quat Orientation: [ %f, %f, %f, %f]"%(quat.x, quat.y, quat.z, quat.w))

        # Pose of robot
        self.goal_x = position.x
        self.goal_y = position.y

        # Also print Roll, Pitch, Yaw
        roll, pitch, yaw = tf.transformations.euler_from_quaternion(
            [quat.x, quat.y, quat.z, quat.w])

        self.goal_yaw = degrees(yaw)

        if(self.goal_yaw < 0):
            self.goal_yaw = 360 + self.goal_yaw


        self.do_dead_reckoning(self.goal_x,self.goal_y,self.goal_yaw)    


    def do_dead_reckoning(self,goal_x, goal_y, yaw_degree):
        
        rospy.loginfo("Goal Position: [ %f, %f, %f ]" % (goal_x, goal_y, yaw_degree))
        rospy.loginfo("Robot Position: [ %f, %f, %f ]" %
                  (self.robot_pose_x, self.robot_pose_y, self.robot_yaw))

        difference_angle = yaw_degree - self.robot_yaw
        rospy.loginfo("Difference angle %d",difference_angle)

        if(difference_angle > 180):
            difference_angle = 360 + difference_angle
            self.turn_vel = -self.turn_vel

        rospy.loginfo("Robot rotating along in the goal axis")

#######################################################################################
        #Robot is rotating to align to the goal position
        while(abs(difference_angle) > self.delta_angle):

            try:
                difference_angle = self.goal_yaw - self.robot_yaw
                rospy.loginfo("Difference angle [%f]" % (
                    difference_angle))
                difference_angle = abs(difference_angle)
                self.send_turn_cmd(1)
                rospy.sleep(0.01)
            except:
                rospy.logwarn("Exception at rotation")
                self.send_turn_cmd(self.stop_vel)
                rospy.sleep(2)
                sys.exit(0)

        self.send_turn_cmd(self.stop_vel)
        rospy.sleep(2)

############################################################################################
        #Robot moving to goal point
        rospy.loginfo("Robot moving to the goal point")

        distance = math.hypot(goal_x - self.robot_pose_x, self.goal_y - self.robot_pose_y)
        rospy.loginfo("Robot moving to the goal point [%f]" % (distance))


        while(distance > self.delta_distance):

            try:

                distance = math.hypot(goal_x - self.robot_pose_x, goal_y - self.robot_pose_y)
                rospy.loginfo("Distance [%f]" % (distance))
                self.send_linear_cmd(self.current_linear_velocity)
                rospy.sleep(0.01)

                if(distance > 1.5):
                    rospy.logwarn("Robot went outside the goal")
                    break

                if(self.obstacle_distance <= self.obstacle_distance_upper_limit and self.obstacle_distance >= self.obstacle_distance_lower_limit):
                    rospy.logwarn(
                        "Obstacle detected, stopping robot and moving away from it")
                    self.send_linear_cmd(self.stop_vel)
                    rospy.sleep(1)
                    self.send_linear_cmd(-self.current_linear_velocity)
                    rospy.sleep(0.4)
                    self.send_linear_cmd(self.stop_vel)
                    break

            except:

                rospy.logwarn("Exception at translation")
                self.send_linear_cmd(self.stop_vel)
                rospy.sleep(2)
                sys.exit(0)

        self.send_linear_cmd(self.stop_vel)
        rospy.sleep(2)


    def send_linear_cmd(self,speed):

        twist = Twist()
        twist.linear.x = speed
        twist.linear.y = 0
        twist.linear.z = 0

        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0

        self.pub_twist.publish(twist)


    def send_turn_cmd(self,speed):

        twist = Twist()

        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0

        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = speed

        self.pub_twist.publish(twist)

    def obstacleCallback(self,msg):

        self.obstacle_distance = msg.data
        rospy.loginfo(msg)

    def shutdown(self,signum,frame):
        rospy.loginfo("Shutting down node")
        sys.exit(0)
        pass



if __name__ == "__main__":
    rospy.init_node("Dead_reckoning_node")
    rospy.loginfo("Starting Dead reckoning node")

    obj = DeadReckoning()


    rospy.spin()
