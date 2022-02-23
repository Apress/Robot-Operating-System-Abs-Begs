#!/usr/bin/env python

def divide(a, b):
	try:
		result = a// b
		print("Your answer is :", result)
	except ZeroDivisionError:
		print("You are dividing by zero")
		
# Look at the parameters and note the working of Program
divide(3, 0)

