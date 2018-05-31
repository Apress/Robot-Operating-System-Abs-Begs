#!/usr/bin/env python
"""
   twist_to_motors - converts a twist message to motor commands.  Needed for navigation stack
   
   
    Copyright (C) 2012 Jon Stephan. 
     
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
import roslib
from std_msgs.msg import Float32
from std_msgs.msg import Int32MultiArray
from geometry_msgs.msg import Twist 

#Mapping -1 - 1 to -255 to 255
from numpy import interp
#############################################################
#############################################################
class TwistToMotors():
#############################################################
#############################################################

    #############################################################
    def __init__(self):
    #############################################################
        rospy.init_node("twist_to_motors")
        nodename = rospy.get_name()
        rospy.loginfo("%s started" % nodename)
    
        self.w = rospy.get_param("~base_width", 0.125)
    
        #self.pub_lmotor = rospy.Publisher('lwheel_vtarget', Float32,queue_size=10)
        #self.pub_rmotor = rospy.Publisher('rwheel_vtarget', Float32,queue_size=10)

	self.pub_speed = rospy.Publisher('/set_speed',Int32MultiArray,queue_size=10)


        rospy.Subscriber('/cmd_vel', Twist, self.twistCallback)
    
    
        self.rate = rospy.get_param("~rate", 50)
        self.timeout_ticks = rospy.get_param("~timeout_ticks", 2)
        self.left = 0
        self.right = 0
        
    #############################################################
    def spin(self):
    #############################################################
    
        r = rospy.Rate(self.rate)
        idle = rospy.Rate(10)
        then = rospy.Time.now()
        self.ticks_since_target = self.timeout_ticks
    
        ###### main loop  ######
        while not rospy.is_shutdown():
        
            while not rospy.is_shutdown() and self.ticks_since_target < self.timeout_ticks:
                self.spinOnce()
                r.sleep()
            idle.sleep()
                
    #############################################################
    def spinOnce(self):
    #############################################################
    
        # dx = (l + r) / 2
        # dr = (r - l) / w
            
        self.right = 1.0 * self.dx + self.dr * self.w / 2 
        self.left = 1.0 * self.dx - self.dr * self.w / 2
        #rospy.loginfo("publishing: (%f, %f)", self.left, self.right) 
 
               
	self.speed_data = Int32MultiArray()


	self.right_mapped = interp(self.right, [-1,1], [-60,60])
	self.left_mapped = interp(self.left, [-1,1], [-60,60])

	if(self.left_mapped < 0 and self.right_mapped > 0):

		self.speed_data.data = [10,self.right_mapped]

	elif(self.left_mapped > 0 and self.right_mapped < 0):

		self.speed_data.data = [self.left_mapped,10]
	else:
		self.speed_data.data = [self.left_mapped,self.right_mapped]


	self.pub_speed.publish(self.speed_data)

        #self.pub_lmotor.publish(self.left)
        #self.pub_rmotor.publish(self.right)
            
        self.ticks_since_target += 1

    #############################################################
    def twistCallback(self,msg):
    #############################################################
        # rospy.loginfo("-D- twistCallback: %s" % str(msg))
        self.ticks_since_target = 0
        self.dx = msg.linear.x
        self.dr = msg.angular.z
        self.dy = msg.linear.y
    
#############################################################
#############################################################
if __name__ == '__main__':
    """ main """
    twistToMotors = TwistToMotors()
    twistToMotors.spin()
