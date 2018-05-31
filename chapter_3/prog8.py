#!/usr/bin/env python


text = raw_input("Enter the text:> ")
file_obj = open("test.txt","w+")
file_obj.write(text)
file_obj.close()

file_obj = open("test.txt","r")
text = file_obj.readline()
print "Read text: ",text



