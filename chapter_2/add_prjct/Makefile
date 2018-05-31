#Makefile case 4
CC = g++
CFLAGS = -c
SOURCES =  main.cpp add.cpp
OBJECTS = $(SOURCES:.cpp=.o)
EXECUTABLE = main

all: $(OBJECTS) $(EXECUTABLE)

$(EXECUTABLE) : $(OBJECTS)
		$(CC) $(OBJECTS) -o $@  

.cpp.o: *.h
	$(CC) $(CFLAGS) $< -o $@

clean :
	-rm -f $(OBJECTS) $(EXECUTABLE)

.PHONY: all clean

