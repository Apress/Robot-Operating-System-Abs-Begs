#!/usr/bin/env python

'''
File name:robot_bt_driver.py

This is ROS driver node for mobile robot using bluetooth, it is using python bluetooth module for sending and receiving data.

This will publish robot IMU, speed and encoder data, also send speed and reset command to robot

Author: Lentin Joseph

'''


#ROS Modules
import rospy
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float32
from std_msgs.msg import Int32
from std_msgs.msg import Int32MultiArray

import serial
import string
import time
import bluetooth

import signal
import sys

##########################################################################################
#Bluetooth parameters

bluetooth_mac = '20:16:04:18:61:60'



rospy.init_node('ros_bluetooth_driver',anonymous=True)
orient = Vector3()
rospy.loginfo("Staring ROS-Bluetooth Driver node")

##########################################################################################
#Speed handler, this will send speed to BL
def speed_send(data):

    	global bluetooth_serial_handle	

	#Note, here we may need change in one motor speed, now taking both speed as data

	left_speed = data.data
	right_speed = data.data

	print 'Setting left speed = %d : right speed=%d' %(left_speed,right_speed)

	#send_data = 's'+' '+str(left_speed)+' '+str(right_speed)+'\r'

	send_data = 's %d %d\r' %(int(left_speed),int(right_speed))

	#print send_data

	try:
		bluetooth_serial_handle.send(str(send_data))
	except:
		print "Unable to send BL data"
		pass

##########################################################################################
def reset_robot(data):
    	global bluetooth_serial_handle	

	send_data = 'r\r'

	try:
		bluetooth_serial_handle.send(str(send_data))
	except:
		print "Unable to send BL data"
		pass


##########################################################################################

#Publisher and subscriber list

left_speed_handle = rospy.Publisher('left_speed', Int32, queue_size=1)
right_speed_handle = rospy.Publisher('right_speed', Int32, queue_size=1)

left_encoder_handle = rospy.Publisher('left_ticks', Int32, queue_size=1)
right_encoder_handle = rospy.Publisher('right_ticks', Int32, queue_size=1)

imu_yaw_handle = rospy.Publisher('yaw', Float32, queue_size=1)
imu_pitch_handle = rospy.Publisher('pitch', Float32, queue_size=1)
imu_roll_handle = rospy.Publisher('roll', Float32, queue_size=1)

imu_data_handle = rospy.Publisher('imu_data', Vector3, queue_size=1)

#Subscribers

rospy.Subscriber('/set_speed', Int32, speed_send)
rospy.Subscriber('/reset', Int32, reset_robot)




##########################################################################################

#Key board handler

def quit_code(signum, frame):
	sys.exit(1)

signal.signal(signal.SIGINT, quit_code)

##########################################################################################




#Function to publish topics to ROS
def publish_topics(string_list):
	global orient

	global left_speed_handle
	global right_speed_handle

	global left_encoder_handle
	global right_encoder_handle


	global imu_yaw_handle
	global imu_pitch_handle
	global imu_roll_handle

	global imu_data_handle


	

	#Reading each line in list and spliting it
	for line in string_list:
		
		real_data = line.split(" ")
		#print real_data

		if(real_data[0] == 's'):
			left_speed_handle.publish(float(real_data[1]))
			right_speed_handle.publish(float(real_data[2]))

		if(real_data[0] == 'e'):
			left_encoder_handle.publish(float(real_data[1]))
			right_encoder_handle.publish(float(real_data[2]))

		if(real_data[0] == 'i'):
			imu_yaw_handle.publish(float(real_data[1]))
			imu_pitch_handle.publish(float(real_data[2]))
			imu_roll_handle.publish(float(real_data[3]))


			orient.x = float(real_data[1])
			orient.y = float(real_data[2])
			orient.z = float(real_data[3])

			imu_data_handle.publish(orient)

##########################################################################################
#Function to remove spaces from sentence 



def remove_space(sentence):
	org_sentence = []
	for char in sentence:
		if(char != ""):
			org_sentence.append(char)

	#print org_sentence
	publish_topics(org_sentence)
	
##########################################################################################

def decode_string(string):
	'''This function will decode the buffer data and
	convert into each string which contain ypr values
	param: string, string buffer
	'''

	start_flag = 0

	final_string = ""

	string_list = []

	decode_list = list(string)
	string_len = len(decode_list)

	#IMU value extract
        
	for i in range(0,string_len):
		#Start reading when y is found
		if(decode_list[i] == 'i' and (string_len - i) > 10):
			start_flag = 1

		#Read until a carriage return occurs
		if(start_flag == 1):
			if(decode_list[i] != '\n'):
				final_string = final_string + decode_list[i]
			else:
				start_flag = 0
		#If it is a new line, append to a list
		if(decode_list[i] == "\n"):
			string_list.append(final_string)

			final_string = ''

	final_string = remove_space(string_list)


	string_list = []
	final_string  = ""
        #Encoder value extract
	for i in range(0,string_len):
		if(decode_list[i] == 'e' and (string_len - i) > 6):
			start_flag = 1

		if(start_flag == 1):
			if(decode_list[i] != '\n'):
				final_string = final_string + decode_list[i]
			else:
				start_flag = 0
		#If it is a new line, append to a list
		if(decode_list[i] == "\n"):
			string_list.append(final_string)

			final_string = ''
	
	final_string = remove_space(string_list)



	string_list = []
	final_string  = ""

        #Motor speed value extract
	for i in range(0,string_len):
		#Start reading when y is found
		if(decode_list[i] == 's' and (string_len - i) > 8):
			start_flag = 1

		if(start_flag == 1):
			if(decode_list[i] != '\n'):
				final_string = final_string + decode_list[i]
			else:
				start_flag = 0
		#If it is a new line, append to a list
		if(decode_list[i] == "\n"):
			string_list.append(final_string)

			final_string = ''	

	final_string = remove_space(string_list)

##########################################################################################
#Function to connect to BL robot
	
def connect():
    global bluetooth_mac
    global bluetooth_serial_handle	
    while(True):    
        try:
            bluetooth_serial_handle = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
            bluetooth_serial_handle.connect((bluetooth_mac, 1))
            break;
        except bluetooth.btcommon.BluetoothError as error:
            bluetooth_serial_handle.close()
	    rospy.logwarn("Unable to connect, Retrying in 10s...")
            #print "Could not connect: ", error, "; Retrying in 10s..."
            time.sleep(10)
    return bluetooth_serial_handle;

bluetooth_serial_handle = connect()


##########################################################################################

#Main code

if __name__ == '__main__':

	while(True):
	    try:
		
		receive_data = bluetooth_serial_handle.recv(300)
		list_rec = receive_data.split("\n")
		list_rec = list(receive_data)
		#Continously receiving data from robot and decode it
		decode_string(list_rec)

		rospy.sleep(0.05)

	    except bluetooth.btcommon.BluetoothError as error:
		#print "Caught BluetoothError: ", error
		rospy.logerr("Caught Bluetooth Error; error")
		time.sleep(5)
		bluetooth_serial_handle = connect()
		pass


	bluetooth_serial_handle.close()
