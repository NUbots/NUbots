###############################################################
#
# Purpose: Makefile for "roboplus"
# Author.: robotis
# Version: 0.1
# License: GPL
#
###############################################################

TARGET = roboplus

CXX = g++
INCLUDE_DIRS = -I../../include -I../../../Framework/include
CXXFLAGS +=	-O2 -DLINUX -g -Wall -fmessage-length=0 $(INCLUDE_DIRS)
LIBS += -lpthread -lncurses -lrt

OBJS =	./main.o


all: darwin.a $(TARGET)

clean:
	rm -f *.a *.o $(TARGET) core *~ *.so *.lo

libclean:
	make -C ../../build clean

distclean: clean libclean

darwin.a:
	make -C ../../build

$(TARGET): $(OBJS) ../../lib/darwin.a
	$(CXX) -o $(TARGET) $(OBJS) ../../lib/darwin.a $(LIBS)
	
# useful to make a backup "make tgz"
tgz: clean
	mkdir -p backups
	tar czvf ./backups/roboplus_`date +"%Y_%m_%d_%H.%M.%S"`.tgz --exclude backups *





