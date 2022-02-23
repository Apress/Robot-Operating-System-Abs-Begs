cmake_minimum_required(VERSION 3.0)

set(CMAKE_BUILD_TYPE Release)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")

project(main)

add_executable(
	main 
	add.cpp 
	main.cpp
)
	
