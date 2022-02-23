#!/usr/bin/env python
class Robot:
	def __init__(self):
		print ("Started robot")
	def move_forward(self,distance):
		print ("Robot moving forward: "+str(distance)+"m")
	def move_backward(self,distance):
		print ("Robot moving backward: "+str(distance)+"m")
	def move_left(self,distance):
		print ("Robot moving left: "+str(distance)+"m")
	def move_right(self,distance):
		print ("Robot moving right: "+str(distance)+"m")
	def __del__(self):
		print ("Robot stopped")
def main():
	obj = Robot()
	obj.move_forward(2)
	obj.move_backward(2)
	obj.move_left(2)
	obj.move_right(2)
if __name__ == "__main__":
	
	main()
