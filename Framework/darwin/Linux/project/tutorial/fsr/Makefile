###############################################################
#
# Purpose: Makefile for "fsr"
# Author.: robotis
# Version: 0.1
# License: GPL
#
###############################################################

TARGET = fsr

CXX = g++
INCLUDE_DIRS = -I../../../include -I../../../../Framework/include
CXXFLAGS +=	-O2 -DLINUX -g -Wall -fmessage-length=0 $(INCLUDE_DIRS)
LIBS += -lpthread -lrt -ljpeg

OBJS =	main.o


all: darwin.a $(TARGET)

darwin.a:
	make -C ../../../build

$(TARGET): $(OBJS) ../../../lib/darwin.a
	$(CXX) -o $(TARGET) $(OBJS) ../../../lib/darwin.a $(LIBS)
	
clean:
	rm -f $(OBJS) $(TARGET)

libclean:
	make -C ../../../build clean

distclean: clean libclean

# useful to make a backup "make tgz"
tgz: clean
	mkdir -p backups
	tar czvf ./backups/tutorial_fsr_`date +"%Y_%m_%d_%H.%M.%S"`.tgz --exclude backups *

