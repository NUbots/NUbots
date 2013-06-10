###############################################################
#
# Purpose: Makefile for "firmware_installer"
# Author.: robotis
# Version: 0.1
# License: GPL
#
###############################################################

TARGET = firmware_installer

CXX = g++
INCLUDE_DIRS = -I../../include -I../../../Framework/include
CXXFLAGS +=	-O2 -DLINUX -g -Wall -fmessage-length=0 $(INCLUDE_DIRS)
LIBS += -lpthread -lrt

OBJS =	./hex2bin.o \
		./main.o


all: $(TARGET)

darwin.a:
	make -C ../../build

$(TARGET): $(OBJS) darwin.a
	$(CXX) -o $(TARGET) $(OBJS) ../../lib/darwin.a $(LIBS)
	
clean:
	rm -f $(OBJS) $(TARGET)

libclean:
	make -C ../../build clean
