#!/usr/bin/env python

import rospy

import random

from std_srvs.srv import Empty


def change_color():

    rospy.init_node('change_color', anonymous=False)

    rospy.set_param('/background_b',random.randint(0,255))
    rospy.set_param('/background_g',random.randint(0,255))
    rospy.set_param('/background_r',random.randint(0,255))

    rospy.wait_for_service('/reset')

    try:

    	serv = rospy.ServiceProxy('/reset',Empty)
	resp = serv()
	rospy.loginfo("Executed service")

    except rospy.ServiceException, e:
	rospy.loginfo("Service call failed: %s" %e)


    rospy.spin()

if __name__ == '__main__':
    try:
        change_color()
    except rospy.ROSInterruptException:
        pass
