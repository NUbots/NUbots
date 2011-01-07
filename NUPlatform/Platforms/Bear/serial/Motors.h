/*! @file Motors.h
    @brief Declaration of serial communication with the servo motors in the Bear

    @class Motors
    @brief A serial communication class with the servo motors in the bear
 
    The serial communication for channel 0 and 1 is done in parallel, and much of the serial
    packet contents are cached. 
 
    Serial writes are straight forward all of the data is added to two binary buffers and then written to each channel.
    Only a single function call is required 'write()' which copies everything stored in MotorControls and MotorPunches 
    to the servo motors.
 
    Serial reads require two steps. The first step is to request the new sensor data with 'request()', the second step
    is to copy it from the usb buffer to the global sensor arrays with 'read()'. The request() is a little tricky
    because it relys on staggering the return delays of blocks of motors so that multiple motors can reply at once.
    Thus, call request() then sometime later data will be available to read()
 
    MOTORS_NUM_MOTORS specifies the number of degrees of freedom of the robot.
    MOTORS_NUM_LOWER_MOTORS specifies the number of degrees of freedom on channel 0
    MOTORS_NUM_UPPER_MOTORS specifies the number of degrees of freedom on channel 1
 
    This class is very old, and not very flexible. The following areas could be improved:
        - It should be updated to use stl vectors so that the number of motors doesn't need to be hard coded. 
        - It should also be updated to use the new libftdi (instead of libftd2xx). 
        - It should be updated to not use global variables to store the sensor data.
        - The threading does need to have quite so much copypasta

    @author Jason Kulk
 
 Copyright (c) 2008, 2009, 2010 Jason Kulk
 
    This file is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This file is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with NUbot.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef MOTORS_H
#define MOTORS_H

class DXSerialThread;
class ConditionalThread;

#include "ftd2xx.h"
#include <vector>
#include <iostream>
#include <fstream>
#include <string>
using namespace std;
 
#define MOTORS_BAUD_RATE                  1000000

#define MOTORS_NUM_MOTORS                 21
#define MOTORS_NUM_LOWER_MOTORS           21
#define MOTORS_NUM_UPPER_MOTORS           0
#define MOTORS_MIN_ID                     2
#define MOTORS_MAX_ID                     22

#define MOTORS_NUM_MOTORS_PER_BLOCK       6                       // the number of motors in a bulk read_data message
#define MOTORS_NUM_LOWER_REQUEST_BLOCKS   (MOTORS_NUM_LOWER_MOTORS+(MOTORS_NUM_MOTORS_PER_BLOCK-1))/MOTORS_NUM_MOTORS_PER_BLOCK        // the number of bulk read_data messages required to poll all of the motors in the lower body
#define MOTORS_NUM_UPPER_REQUEST_BLOCKS   (MOTORS_NUM_UPPER_MOTORS+(MOTORS_NUM_MOTORS_PER_BLOCK-1))/MOTORS_NUM_MOTORS_PER_BLOCK

#define MOTORS_UPPER_BODY                 0
#define MOTORS_LOWER_BODY                 1

#define MOTORS_NUM_CONTROLS               1+4            // the number of bytes used to control the motors (this includes the write address(es))
#define MOTORS_NUM_PUNCHES                1+2            // the number of bytes used to change the 'punch' of the motors (this includes the write address)
#define MAX_MESSAGE_LENGTH                100

// Globally availiable feedback arrays (be aware that accessing them is inherently not thread-safe, but I do make an effort to not update them until control has finished)
extern long double JointTime;
extern long double PreviousJointTime;
extern unsigned short JointPositions[MOTORS_NUM_MOTORS];
extern unsigned short JointSpeeds[MOTORS_NUM_MOTORS];
extern unsigned short JointLoads[MOTORS_NUM_MOTORS];

// This structure's sole purpose is to store data required for a threaded write
struct threaddata_t
{
   FT_HANDLE Handle;                // the handle to buffer will be written to
   unsigned char* Buffer;           // pointer to the buffer containing the data
   unsigned short BufferLength;     // the number of bytes in the buffer
};

class Motors
{
   public:
      static Motors* getInstance();
      void setSensorThread(ConditionalThread* thread);
      void updateControl(unsigned char motorid, unsigned short position, unsigned short speed, unsigned short punch);
      void updateControls(unsigned char motorid[], unsigned char nummotors, unsigned short positions[], unsigned short speeds[], unsigned short punches[]);
      void updateControls(unsigned short positions[MOTORS_NUM_MOTORS], unsigned short speeds[MOTORS_NUM_MOTORS], unsigned short punches[MOTORS_NUM_MOTORS]);           // update the control variables (they will be sent to the motors on the next cycle)
      bool write();                                                                       // write motor control commands
      bool request();                                                                     // request for feedback data
      bool read();                                                                        // read feedback data and put it into global feedback arrays
    
      void getTargets(vector<float>& targets);
    
      void torqueEnable();                                                                // turn on all motors
      void emergencyOff();                                                                // emergency off (fast, collapse in heap)
      void torqueOn(unsigned char motorid[], unsigned char nummotors);                    // turn the motors on (and start sending motor controls)
      void torqueOff(unsigned char motorid[], unsigned char nummotors);                   // turn the motors off (and stop sending motor controls)
      
   private:
      // Singleton class has private constructor, copy and assignment
      Motors();
      ~Motors();
      Motors(const Motors&);
      Motors& operator=(const Motors&);
      // Initialisation
      void initSelf();
      void initRequestMessages();
      void initSerial();
      void initReturnDelays();
      void initControlTables();
      void initSlopes();
      void initMargins();
      void initPunches();
      void initFiles();
      void initHistory();
   
      void closeSerial();
   
      // Serial Writing
      bool write(unsigned char motorid, unsigned char command, unsigned char data[], unsigned char datalength);
      bool write(unsigned char motorid[], unsigned char nummotors, unsigned char command, unsigned char* data[], unsigned char datalength);
      bool write(unsigned char command, unsigned char data[MOTORS_NUM_MOTORS][MOTORS_NUM_CONTROLS]);
      bool broadcast(unsigned char command, unsigned char data[], unsigned short datalength);
   
      void appendPacketToBuffer(unsigned char motorid, unsigned char command, unsigned char data[], unsigned char datalength, unsigned char messagebuffer[], unsigned short* currentbufferindex);
      void appendPacketsToBuffer(unsigned char motorid[], unsigned char nummotors, unsigned char command, unsigned char* data[], unsigned char datalength, unsigned char lowermessagebuffer[], unsigned char uppermessagebuffer[], unsigned short* lowerindex, unsigned short* upperindex);
      void appendPacketsToBuffer(unsigned char command, unsigned char data[MOTORS_NUM_MOTORS][MOTORS_NUM_CONTROLS], unsigned char lowermessagebuffer[], unsigned char uppermessagebuffer[], unsigned short* lowerindex, unsigned short* upperindex);
      void appendPacketsToBuffer(unsigned char command, unsigned char data[MOTORS_NUM_MOTORS][MOTORS_NUM_PUNCHES], unsigned char lowermessagebuffer[], unsigned char uppermessagebuffer[], unsigned short* lowerindex, unsigned short* upperindex);
      void appendControlPacketsToBuffer(unsigned char lowermessagebuffer[], unsigned char uppermessagebuffer[], unsigned short* lowerindex, unsigned short* upperindex);
   
      // Serial Reading
      unsigned short getNumBytesInQueue(FT_HANDLE fthandle);
      bool read(FT_HANDLE fthandle, unsigned char data[], unsigned short numbytestoread);
      unsigned short readQueue(FT_HANDLE fthandle, unsigned char data[], unsigned short maxdatalength);
      unsigned char updateFeedbackData(unsigned char readdata[], unsigned short numbytes);
      bool findHeader(unsigned char readdata[], unsigned short numbytes, unsigned short* index);
   
   public:
      static unsigned char MotorIDToLowerBody[];
      static unsigned char LowerBodyIndexToMotorID[];
      static unsigned char UpperBodyIndexToMotorID[];
      static unsigned char IndexToMotorID[];
      static unsigned char MotorIDToIndex[];  
      
      static char MotorSigns[];
      static unsigned short DefaultPositions[];
      static unsigned short DefaultSpeeds[];
   
      static unsigned char DefaultSlopes[];
      static unsigned char DefaultMargins[];
      static unsigned short DefaultPunches[];
   
   private:
      FT_HANDLE upperHandle;              // handle of the d2xx channel for the upper body (device 1)
      FT_HANDLE lowerHandle;              // handle of the d2xx channel for the lower body (device 0)
   
      // Control packet data
      unsigned char MotorControls[MOTORS_NUM_MOTORS][MOTORS_NUM_CONTROLS];
      unsigned char MotorPunches[MOTORS_NUM_MOTORS][MOTORS_NUM_PUNCHES];
      
      // Pre-generated request packets
      unsigned char MotorRequestsLower[MOTORS_NUM_LOWER_REQUEST_BLOCKS][MOTORS_NUM_LOWER_MOTORS*MAX_MESSAGE_LENGTH];
      unsigned short MotorRequestsLowerLength[MOTORS_NUM_LOWER_REQUEST_BLOCKS];
      unsigned char MotorRequestsUpper[MOTORS_NUM_UPPER_REQUEST_BLOCKS][MOTORS_NUM_UPPER_MOTORS*MAX_MESSAGE_LENGTH];
      unsigned short MotorRequestsUpperLength[MOTORS_NUM_UPPER_REQUEST_BLOCKS];
   
      // Threaded write data
      threaddata_t ThreadLowerWrite, ThreadUpperWrite;
   
      // Software motor on/off control
      bool MotorTorqueOn[MOTORS_NUM_MOTORS];
   
      long double StartTime;
    
      DXSerialThread* m_thread;
};

void* runThreadedWrite(void *arg);

#endif
