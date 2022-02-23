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
from std_msgs.msg import Float32, Int32, Bool
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

        rospy.on_shutdown(self.shutdown_cb)


        self.sign = lambda a: (a>0) - (a<0)

        self.left_speed = Int32()
        self.right_speed = Int32()

        self.w = rospy.get_param("~base_width", 0.125)
        self.fixed_speed = rospy.get_param("~fixed_speed", 180)
    
        self.pub_lmotor = rospy.Publisher('set_left_speed', Int32,queue_size=1)
        self.pub_rmotor = rospy.Publisher('set_right_speed', Int32,queue_size=1)

        self.reset = rospy.Publisher('reset', Bool,queue_size=1)

        rospy.Subscriber('/cmd_vel', Twist, self.twistCallback)
    
    
        self.rate = rospy.get_param("~rate", 50)
        self.timeout_ticks = rospy.get_param("~timeout_ticks", 2)
        self.left = 0
        self.right = 0
    
    def shutdown_cb(self):
        rospy.logwarn("Resetting board")
        self.pub_lmotor.publish(0)
        self.pub_rmotor.publish(0)

        self.reset.publish(0)


        pass

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
               
        self.left_mapped = self.sign(self.left)*self.fixed_speed
        self.right_mapped = self.sign(self.right)*self.fixed_speed

        self.left_speed.data = int(self.left_mapped)
        self.right_speed.data =int(self.right_mapped)

        rospy.loginfo(self.left_speed)
        rospy.loginfo(self.right_speed)

        self.pub_lmotor.publish(self.left_speed)
        self.pub_rmotor.publish(self.right_speed)

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
