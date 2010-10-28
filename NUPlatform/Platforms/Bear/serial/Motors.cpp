/* Motor serial
 
 by Jason Kulk (jason.555 at gmail dot com)
 
 Copyright (c) 2009 Jason Kulk 
 
 motors.h is part of Jason's Cycloid Code.
 
 Jason's Cycloid Code is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.
 
 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.
 
 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "dx117.h"
#include "Motors.h"
#include "DXSerialThread.h"

#include "debug.h"
#include "debugverbositynuplatform.h"

unsigned char Motors::MotorIDToLowerBody[MOTORS_MAX_ID+1] = {MOTORS_LOWER_BODY, MOTORS_LOWER_BODY, MOTORS_LOWER_BODY, MOTORS_LOWER_BODY, MOTORS_LOWER_BODY, MOTORS_LOWER_BODY, MOTORS_LOWER_BODY, MOTORS_LOWER_BODY, MOTORS_LOWER_BODY, MOTORS_LOWER_BODY, MOTORS_LOWER_BODY, MOTORS_LOWER_BODY, MOTORS_LOWER_BODY, MOTORS_LOWER_BODY, MOTORS_LOWER_BODY, MOTORS_LOWER_BODY, MOTORS_LOWER_BODY, MOTORS_LOWER_BODY, MOTORS_LOWER_BODY, MOTORS_LOWER_BODY, MOTORS_LOWER_BODY, MOTORS_LOWER_BODY, MOTORS_LOWER_BODY};
unsigned char Motors::IndexToMotorID[MOTORS_NUM_MOTORS] = {22, 21, 20, 6, 4, 8, 7, 5, 9, 3, 2, 12, 10, 14, 18, 16, 13, 11, 15, 19, 17};
unsigned char Motors::MotorIDToIndex[MOTORS_MAX_ID+1] = {-1, -1, 10, 9, 4, 7, 3, 6, 5, 8, 12, 17, 11, 16, 13, 18, 15, 20, 14, 19, 2, 1, 0};
unsigned char Motors::LowerBodyIndexToMotorID[MOTORS_NUM_LOWER_MOTORS] = {2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22};
unsigned char Motors::UpperBodyIndexToMotorID[MOTORS_NUM_UPPER_MOTORS] = {};

//                                                               HP    HY    TP   LSR   LSP   LEP   RSR   RSP   REP    TR    TY   LHR   LHP    LK   LAR   LAP   RHR   RHP    RK   RAR   RAP
//                                                                0    1     2     3     4     5     6     7     8    9     10    11    12    13    14    15    16    17    18    19    20  
char Motors::MotorSigns[MOTORS_NUM_MOTORS] =                   { -1,    1,    1,   -1,    1,    1,   -1,   -1,   -1,   -1,    1,   -1,    1,    1,    1,    1,   -1,   -1,   -1,    1,   -1}; 
unsigned short Motors::DefaultPositions[MOTORS_NUM_MOTORS] =   {499,  481,  374,  709,  526,  612,  318,  500,  412,  525,  528,  657,  372,  531,  773,  508,  563,  642,  514,  500,  519}; 
unsigned short Motors::DefaultSpeeds[MOTORS_NUM_MOTORS] =      {100,  100,  100,  100,  100,  100,  100,  100,  100,  100,  100,  100,  100,  100,  100,  100,  100,  100,  100,  100,  100}; 
// Compliance Settings. Remember the slope has to be a power of 2, and cannot be changed online without producing jerk
unsigned char Motors::DefaultSlopes[MOTORS_NUM_MOTORS] =       {006,  006,  005,  005,  004,  006,  005,  004,  006,  006,  006,  005,  005,  006,  007,  006,  005,  005,  006,  007,  006}; 
//unsigned char Motors::DefaultSlopes[MOTORS_NUM_MOTORS] =       {004,  004,  004,  004,  004,  004,  004,  004,  004,  004,  004,  004,  004,  004,  004,  004,  004,  004,  004,  004,  004}; 
unsigned char Motors::DefaultMargins[MOTORS_NUM_MOTORS] =      {000,  000,  000,  000,  000,  000,  000,  000,  000,  000,  000,  000,  000,  000,  000,  000,  000,  000,  000,  000,  000}; 
unsigned short Motors::DefaultPunches[MOTORS_NUM_MOTORS] =     {000,  000,  000,  000,  000,  000,  000,  000,  000,  000,  000,  000,  000,  000,  000,  000,  000,  000,  000,  000,  000}; 

// externally available feedback arrays
long double JointTime = 0.0;
long double PreviousJointTime = 0.0;
unsigned short JointPositions[MOTORS_NUM_MOTORS] =             {000,  000,  000,  000,  000,  000,  000,  000,  000,  000,  000,  000,  000,  000,  000,  000,  000,  000,  000,  000,  000}; 
unsigned short JointSpeeds[MOTORS_NUM_MOTORS] =                {000,  000,  000,  000,  000,  000,  000,  000,  000,  000,  000,  000,  000,  000,  000,  000,  000,  000,  000,  000,  000}; 
unsigned short JointLoads[MOTORS_NUM_MOTORS] =                 {000,  000,  000,  000,  000,  000,  000,  000,  000,  000,  000,  000,  000,  000,  000,  000,  000,  000,  000,  000,  000}; 

/*! Create a serial link with the motors
 */
Motors::Motors()
{  
    initSelf();
    initRequestMessages();
    initSerial();
    initReturnDelays();
    initControlTables();  //<-- I don't need to change this, however, occasionally the motor does resort back to 57k baud rate
    initSlopes();
    initMargins();
    // initPunches(); the punch is now used as a control variable
    struct timespec timenow;
    clock_gettime(CLOCK_REALTIME, &timenow);
    StartTime = timenow.tv_nsec/1e6 + timenow.tv_sec*1e3;
    
    m_thread = new DXSerialThread(this, 20);
}

/*! Destroy the serial link with the motors
 */
Motors::~Motors()
{
   delete m_thread;
   closeSerial();
}

/*! @brief Get the Motors instance
    @return returns a pointer to the single Motors instance
 */
Motors* Motors::getInstance()
{
    static Motors instance;
    return &instance;
}

/*! @brief Sets the external motion/sensor thread, so that its startLoop function is called everytime there is new sensor data
    @param thread a pointer to the sensemovethread
 */
void Motors::setSensorThread(ConditionalThread* thread)
{
    m_thread->setSensorThread(thread);
}

/* Initialise self
 MotorTorqueOn[] is initialised such that every motor is off
 MotorControls[][] is initialised such that the positions, and speeds are at their default settings (ie. the robot will stand up in its reference position)
 */
void Motors::initSelf()
{
   #if DEBUG_NUPLATFORM_VERBOSITY > 0
      debug << "MOTORS: Initialising self." << endl;
   #endif
   // Initialise motor torques to off
   for (unsigned char i=0; i<MOTORS_NUM_MOTORS; i++)
   {
      MotorTorqueOn[i] = false;
   }
   
   // Initialise the controls to their default values
   for (unsigned char i=0; i<MOTORS_NUM_MOTORS; i++)
   {
      MotorControls[i][0] = P_GOAL_POSITION_L;
      MotorPunches[i][0] = P_PUNCH_L;
   }
   updateControls(Motors::DefaultPositions, Motors::DefaultSpeeds, Motors::DefaultPunches);
   return;
}

/* Initialise feedback data request messages (I precalculate all of these messages to save time, they are always the same)
 The following class members are used to store the messages:
    MotorRequestsLower[MOTORS_NUM_LOWER_REQUEST_BLOCKS][MOTORS_NUM_LOWER_MOTORS*MAX_MESSAGE_LENGTH];     <--- The messages for the lower body motors
    MotorRequestsLowerLength[MOTORS_NUM_LOWER_REQUEST_BLOCKS];                                           <--- The length of each lower body message
    MotorRequestsUpper[MOTORS_NUM_UPPER_REQUEST_BLOCKS][MOTORS_NUM_UPPER_MOTORS*MAX_MESSAGE_LENGTH];     <--- The messages for the upper body motors
    MotorRequestsUpperLength[MOTORS_NUM_UPPER_REQUEST_BLOCKS];                                           <--- The length of each upper body message
 
 The request messages are organised as follows:
    The lower and upper body motors are on separate channels, so the messages for each are separate
    Each channel has the motors separated into blocks of 6 or less, each block of motors is sent requests with the same call to FT_Write, and the reply delays are staggered such that they reply at different times
 
 Essentially, to use the messages
   FT_Write(lowerbodymessage[0])
   FT_Write(upperbodymessage[0])
   wait for reply
   FT_Write(lowerbodymessage[1])
   FT_Write(upperbodymessage[1])
   wait for reply
   etc...
 */
void Motors::initRequestMessages()
{
   #if DEBUG_NUPLATFORM_VERBOSITY > 0
      debug << "MOTORS: Initialising Request Messages. Number of lower blocks: " << MOTORS_NUM_LOWER_REQUEST_BLOCKS << " Number of upper blocks: " << MOTORS_NUM_UPPER_REQUEST_BLOCKS << endl;
   #endif
   // The 'data' for the request packet for each motor is always the same
   unsigned char data[] = {P_PRESENT_POSITION_L, NUM_FEEDBACK_MOTOR};   // [start address, length]
   unsigned char* requestdata[MOTORS_NUM_MOTORS_PER_BLOCK];
   for (unsigned char i=0; i<MOTORS_NUM_MOTORS_PER_BLOCK; i++)
   {
      requestdata[i] = data;
   }
   
   // Now I only need to get the motorids and the number in this block
   // I get the motorids by offseting the *BodyIndexToMotorID by the number of previous indices used
   // I get the number of motors by comparing the max number of motors in a block to the number that haven't been sent already, its the max or the number left
   
   // Do the lower body first
   unsigned char nummotorsinblock = MOTORS_NUM_MOTORS_PER_BLOCK;
   for (unsigned char i=0; i< MOTORS_NUM_LOWER_REQUEST_BLOCKS; i++)
   {
      // determine the number of motors in this block
      if (MOTORS_NUM_LOWER_MOTORS - i*MOTORS_NUM_MOTORS_PER_BLOCK < MOTORS_NUM_MOTORS_PER_BLOCK)
         nummotorsinblock = MOTORS_NUM_LOWER_MOTORS - i*MOTORS_NUM_MOTORS_PER_BLOCK;
      else
         nummotorsinblock = MOTORS_NUM_MOTORS_PER_BLOCK;
      appendPacketsToBuffer(&Motors::LowerBodyIndexToMotorID[i*MOTORS_NUM_MOTORS_PER_BLOCK], nummotorsinblock, DX117_READ, requestdata, 2, MotorRequestsLower[i], NULL, &MotorRequestsLowerLength[i], NULL);
   }

   for (unsigned char i=0; i< MOTORS_NUM_UPPER_REQUEST_BLOCKS; i++)
   {
      // determine the number of motors in this block
      if (MOTORS_NUM_UPPER_MOTORS - i*MOTORS_NUM_MOTORS_PER_BLOCK < MOTORS_NUM_MOTORS_PER_BLOCK)
         nummotorsinblock = MOTORS_NUM_UPPER_MOTORS - i*MOTORS_NUM_MOTORS_PER_BLOCK;
      else
         nummotorsinblock = MOTORS_NUM_MOTORS_PER_BLOCK;
      appendPacketsToBuffer(&Motors::UpperBodyIndexToMotorID[i*MOTORS_NUM_MOTORS_PER_BLOCK], nummotorsinblock, DX117_READ, requestdata, 2, NULL, MotorRequestsUpper[i], NULL, &MotorRequestsUpperLength[i]);
   }

#if DEBUG_NUPLATFORM_VERBOSITY > 2
   debug << "MOTORS: Initialised request messages:" << MotorRequestsLowerLength[0] << " upper length: " << MotorRequestsUpperLength[0] << endl;
#endif
}


/* Initialise the actual serial connection with the motors
 The connection to the lower body and upper body channels is opened
 The baud rate and data characteristics are set (1Mbp, 8bit no parity, 1 stop bit)
 The read and write timeouts are set to 1ms
 The latency timer is set to 2ms (This is the latency between the chip receiving data, and the PC getting it)
 */
void Motors::initSerial()
{
   // open, set baud rate, set data characteristics, set timeouts
   #if DEBUG_NUPLATFORM_VERBOSITY > 0
      debug << "MOTORS: Initialising RS-485 dual channel comms to DX-117s." << endl;
   #endif
   
   // Open
   FT_STATUS status;
   status = FT_Open(0, &lowerHandle);
   if (status != FT_OK)
      debug << "MOTORS: Unable to open lower body serial connection" << endl;

   status = FT_Open(1, &upperHandle);
   if (status != FT_OK)
      debug << "MOTORS: Unable to open upper body serial connection" << endl;
   
   // Set Baud Rate
   status = FT_SetBaudRate(lowerHandle, MOTORS_BAUD_RATE);
   if (status != FT_OK)
      debug << "MOTORS: Unable to set lower body serial baud rate" << endl;
   
   status = FT_SetBaudRate(upperHandle, MOTORS_BAUD_RATE);
   if (status != FT_OK)
      debug << "MOTORS: Unable to set upper body serial baud rate" << endl;
   
   // Set Data characteristics (8 bit bytes, no parity, 1 stop bit)
   status = FT_SetDataCharacteristics(lowerHandle, FT_BITS_8, FT_STOP_BITS_1, FT_PARITY_NONE);
   if (status != FT_OK)
      debug << "MOTORS: Unable to set lower body serial data characteristics" << endl;
   
   status = FT_SetDataCharacteristics(upperHandle, FT_BITS_8, FT_STOP_BITS_1, FT_PARITY_NONE);
   if (status != FT_OK)
      debug << "MOTORS: Unable to set upper body serial data characteristics" << endl;
   
   // Set timeouts
   status = FT_SetTimeouts(lowerHandle, 1, 1);
   if (status != FT_OK)
      debug << "MOTORS: Unable to set lower body serial timeouts" << endl;
   
   status = FT_SetTimeouts(upperHandle, 1, 1);
   if (status != FT_OK)
      debug << "MOTORS: Unable to set upper body serial timeouts" << endl;
   
   status = FT_SetLatencyTimer(lowerHandle, 2);
   if (status != FT_OK)
      debug << "MOTORS: Unable to set lower latency timer" << endl;
   status = FT_SetLatencyTimer(upperHandle, 2);
   if (status != FT_OK)
      debug << "MOTORS: Unable to set upper latency timer" << endl;
}

/* Initialise the motor return delays (ie. the delay between receiving a request for data, and the motor replying)
 We can use this delay to ensure motors don't reply at the same time.
 
 Through experiment, I have determined delays that allow 6 motors to have requests sent at the same time, but replys staggered so that each are received in-turn without error.
 These staggered delays have resulted in the motors being segmented into block of 6 or less. The segmentation is done automatically, based on lower/upper body index
 
 The delays are sent to the motors in this function.
 */
void Motors::initReturnDelays()
{
   #if DEBUG_NUPLATFORM_VERBOSITY > 0
      debug << "MOTORS: Initialising DX-117 return delays." << endl;
   #endif
   // The idea here is to set the return delays to be different so that the motors do not reply to bulk read_data commands all at once.
   unsigned char data[] = {P_RETURN_DELAY_TIME, 0};         // write a single byte to that address, first byte is the address, second byte is the new return delay
   unsigned char initialdelay = 50;                         // the initial delay (the delay before the first packet can be sent; sending it too early will result in a reply while the rest of the bulk read_data message is being sent)
   unsigned char replydelay = 40;                           // the delay required between consecutive replys (the actual delay is 2*value microseconds) 
   unsigned char delays[MOTORS_NUM_MOTORS_PER_BLOCK];
   for (unsigned char i=0; i<MOTORS_NUM_MOTORS_PER_BLOCK; i++)
   {
      delays[i] = initialdelay + replydelay*i;
   }
   
   for (unsigned char i=0; i<MOTORS_NUM_LOWER_MOTORS; i++)
   {
      data[1] = delays[i%MOTORS_NUM_MOTORS_PER_BLOCK];
      write(Motors::LowerBodyIndexToMotorID[i], DX117_WRITE, data, 2);
   }
   for (unsigned char i=0; i<MOTORS_NUM_UPPER_MOTORS; i++)
   {
      data[1] = delays[i%MOTORS_NUM_MOTORS_PER_BLOCK];
      write(Motors::UpperBodyIndexToMotorID[i], DX117_WRITE, data, 2);
   }
}

void Motors::initControlTables()
{
   // firstly, because there is an echo on the serial link I need to empty the read buffer before doing this
   unsigned char readdata[4096];
   unsigned short numbytes = 0;
   sleep(1);
   readQueue(lowerHandle, readdata, 4096);
   
   // for each motor read the control table
   unsigned char data[] = {P_ID, 16};           // read 16 bytes starting from P_ID
   for (unsigned char i=0; i<MOTORS_NUM_MOTORS; i++)
   {
      write(Motors::IndexToMotorID[i], DX117_READ, data, 2);
   }
   // now I need to wait and then read the buffer
   sleep(1);
   numbytes = readQueue(lowerHandle, readdata, 4096);
   
   /* DX117 Reply Packet Format:
    0xFF, 0xFF, ID, length, error, para1, para2, ..., para(length-2), checksum
    */
   unsigned short index = 0;
   unsigned char id, length, error, checksum, calculatedchecksum;
   unsigned char packetdata[MAX_MESSAGE_LENGTH];                          // [parm0, parm1, ... , parmN];
   unsigned short numupdates = 0;
   // scan through and find the first header
   while (index < numbytes)
   {
      if(findHeader(readdata, numbytes, &index))
      {
         // debug << "MOTORS: init: header end at index: " << index << endl;
         if (index + 2 < numbytes)
         {
            index++;
            id = readdata[index];
            index++;
            length = readdata[index];
         }
         // debug << "MOTORS: scanData: id: " << (int)id << " length: " << (int)length << endl;
         if (length < MAX_MESSAGE_LENGTH)
         {
            if (index + length < numbytes)
            {
               index++;
               error = readdata[index];
               index++;
               calculatedchecksum = id + length + error;
               for (unsigned char i=0; i<length-2; i++)
               {
                  packetdata[i] = readdata[index + i];
                  calculatedchecksum += packetdata[i];
               }
               index += length - 2;
               checksum = readdata[index];
               calculatedchecksum = ~calculatedchecksum;
               
               if ((calculatedchecksum == checksum) && ((length - 2) == 16) && (id < MOTORS_NUM_MOTORS))
               {
                  if (error > 0)
                     debug << "MOTORS: Motor " << (int)id << " has error " << (int)error << ", you should look into it ;)" << endl;
                  for (unsigned char i=0; i<length-2; i++)
                  {
                     debug << (int)packetdata[i] << ", ";
                  }
                  debug << "!:! " << (int)numupdates << endl;
                  numupdates++;
               }
            }
         }
      }
   }
}

/* Initialise motor compliance slopes
 
 DefaultSlopes[] is used to set a symmetric compliance slope for each motor. Remember slopes are now levels (1 to 7) where 1 is the steepest slope
 */
void Motors::initSlopes()
{
   #if DEBUG_NUPLATFORM_VERBOSITY > 0
      debug << "MOTORS: Initialising DX-117 compliance slopes." << endl;
   #endif
   unsigned char data[] = {P_CW_COMPLIANCE_SLOPE, 0, 0};      // address, cw_slope, ccw_slope
   
   // I know from previous work that the slope has to be a power of 2, so I have chosen to label the slopes (1 to 7) where 1 is the steepest (being consistent with the RX series manuals).
   for (unsigned char i=0; i<MOTORS_NUM_MOTORS; i++)
   {
      switch (Motors::DefaultSlopes[i])
      {
      case 1:
         data[1] = DX117_SLOPE_1;
         data[2] = DX117_SLOPE_1;
         break;
      case 2:
         data[1] = DX117_SLOPE_2;
         data[2] = DX117_SLOPE_2;
         break;
      case 3:
         data[1] = DX117_SLOPE_3;
         data[2] = DX117_SLOPE_3;
         break;
      case 4:
         data[1] = DX117_SLOPE_4;
         data[2] = DX117_SLOPE_4;
         break;
      case 5:
         data[1] = DX117_SLOPE_5;
         data[2] = DX117_SLOPE_5;
         break;
      case 6:
         data[1] = DX117_SLOPE_6;
         data[2] = DX117_SLOPE_6;
         break;
      case 7:
         data[1] = DX117_SLOPE_7;
         data[2] = DX117_SLOPE_7;
         break;
      default:
         data[1] = DX117_SLOPE_4;
         data[2] = DX117_SLOPE_4;
         break;
      };
      write(Motors::IndexToMotorID[i], DX117_WRITE, data, 3);
   }
   return;
}

/* Initialse motor compliance margins
 The margins are symmetric
 */
void Motors::initMargins()
{
   unsigned char data[] = {P_CW_COMPLIANCE_MARGIN, 0, 0};      // address, cw_margin, ccw_margin

   for (unsigned char i=0; i<MOTORS_NUM_MOTORS; i++)
   {
      data[1] = DefaultMargins[i];
      data[2] = DefaultMargins[i];
      write(Motors::IndexToMotorID[i], DX117_WRITE, data, 3);
   }
   return;
}

/* Initialise the motor punch settings
 */
void Motors::initPunches()
{
   unsigned char data[] = {P_PUNCH_L, 0, 0};      // address, low byte, high byte
   
   for (unsigned char i=0; i<MOTORS_NUM_MOTORS; i++)
   {
      data[1] = DefaultPunches[i] & 0xFF;
      data[2] = (DefaultPunches[i] >> 8) & 0xFF;
      write(Motors::IndexToMotorID[i], DX117_WRITE, data, 3);
   }
   return;
}

/* Close the serial connection
 Both the lower and upper body connections are closed
 */
void Motors::closeSerial()
{
   FT_STATUS status;
   status = FT_Close(lowerHandle);
   if (status != FT_OK)
      debug << "MOTORS: Failed to close lower body serial connection" << endl;
   
   status = FT_Close(upperHandle);
   if (status != FT_OK)
      debug << "MOTORS: Failed to close lower body serial connection" << endl;
}

/*! Enable the torque on each motor (this will not start sending control commands, use torqueOn to do that)
 Thus, this commands will make the robot 'rigid' in its current position. A following torqueOn will get the robot to move into the current target position
 */
void Motors::torqueEnable()
{
   #if DEBUG_NUPLATFORM_VERBOSITY > 0
      debug << "MOTORS: Enabling torque on all motors" << endl;
   #endif
   unsigned char data[] = {P_TORQUE_ENABLE, DX117_TORQUE_ON};
   write(DX117_BROADCASTING_ID, DX117_WRITE, data, 2);
}

/*! Turn all of the motors off as quickly as possible. 
 */
void Motors::emergencyOff()
{
   debug << "MOTORS: Emergency motor off!" << endl;
   for (unsigned char i=0; i<MOTORS_NUM_MOTORS; i++)
   {
      MotorTorqueOn[i] = false;
   }
   unsigned char data[] = {P_TORQUE_ENABLE, DX117_TORQUE_OFF};
   debug << "MOTORS: Writing to motors" << endl;
   write(DX117_BROADCASTING_ID, DX117_WRITE, data, 2);
   debug << "MOTORS: Finished writing to motors" << endl;
}

/*! Turn on the specified motors and start sending motor controls to those motors
 @param motorid[] the motor ids of the motors to turn on
 @param nummotors the length of motorid[]
 */
void Motors::torqueOn(unsigned char motorid[], unsigned char nummotors)
{
   for (unsigned char i=0; i<nummotors; i++)
      MotorTorqueOn[MotorIDToIndex[motorid[i]]] = true;
}

/*! Turn off the spcified motors and stop sending motor controls to those motors
 @param motorid[] the motor ids of the motors to turn off
 @param nummotors the length of motorid[]
 */
void Motors::torqueOff(unsigned char motorid[], unsigned char nummotors)
{
   for (unsigned char i=0; i<nummotors; i++)
   {
      MotorTorqueOn[MotorIDToIndex[motorid[i]]] = false;
   }
   unsigned char idata[] = {P_TORQUE_ENABLE, DX117_TORQUE_OFF};
   unsigned char* data[nummotors];
   for (unsigned char i=0; i<nummotors; i++)
   {
      data[i] = idata;
   }
   write(motorid, nummotors, DX117_WRITE, data, 2);
}

/*! Update the motors controls and punches, the updated values will be sent to the motor in the next cycle
 MotorControls[][] and MotorPunches[][] are updated to reflect the new position, speed and punch
 Positions, speeds and punches are unsigned, the value '-1' is used to indicate that particular entry should remain unchanged.
 
 @todo TODO: update the retardness of the -1 to NaN
 
 @param motorid, the id of the motor the commands are for
 @param positions, the new target position for the motor
 @param speeds, the new speed for the motor
 @param punches, the new punche for the motor
 */
void Motors::updateControl(unsigned char motorid, unsigned short position, unsigned short speed, unsigned short punch)
{
   if (position != (unsigned short) -1)
   {
      if (position > (unsigned short) 1023)
         position = (unsigned short) 1023;
      // MotorControls[i][0] is the start address for the write (it is always the same)
      MotorControls[MotorIDToIndex[motorid]][1] = (unsigned char) (position & 0xFF);          // the low byte is first
      MotorControls[MotorIDToIndex[motorid]][2] = (unsigned char) ((position >> 8) & 0xFF);
   }
   
   if (speed != (unsigned short) -1)
   {
      if (speed > (unsigned short) 1023)
         speed = (unsigned short) 1023;
      else if (speed == (unsigned short) 0) 
         speed = 1;         // On the Dynamixel's a speed of 0 is mapped to no speed control (ie max speed)
      else if (speed == (unsigned short) -2)
          speed = 0;
       
      MotorControls[MotorIDToIndex[motorid]][3] = (unsigned char) (speed & 0xFF);
      MotorControls[MotorIDToIndex[motorid]][4] = (unsigned char) ((speed >> 8) & 0xFF);
   }
   
   if (punch != (unsigned short) -1)
   {
      if (punch > (unsigned short) 1023)
         punch = (unsigned short) 1023;
      // MotorPunches[i][0] is the address of the low byte of the punches
      MotorPunches[MotorIDToIndex[motorid]][1] = (unsigned char) (punch & 0xFF);
      MotorPunches[MotorIDToIndex[motorid]][2] = (unsigned char) ((punch >> 8) & 0xFF);
   }

}

/*! Update the motors controls and punches, the updated values will be sent ot the motors in the next cycle
 MotorControls[][] and MotorPunches[][] are updated to reflect the new position and speeds
 Positions, speeds and punches are unsigned, the value '-1' is used to indicate that particular entry should remain unchanged.
 
 @param motorid[nummotors], the ids of the motors the commands are for
 @param nummotors, the length of every array passed here
 @param positions[nummotors], the new target position for each motor
 @param speeds[nummotors], the new speeds for each motor
 @param punches[nummotors], the new punches for each motor
 */
void Motors::updateControls(unsigned char motorid[], unsigned char nummotors, unsigned short positions[], unsigned short speeds[], unsigned short punches[])
{
   for (unsigned char i=0; i<nummotors; i++)
   {
      updateControl(motorid[i], positions[i], speeds[i], punches[i]);
   }
   
#if DEBUG_NUPLATFORM_VERBOSITY > 2
   debug << "MOTORS: Updating controls: " << endl;
   for (int j=0; j<nummotors; j++)
   {
      debug << j << ": ";
      for (int i=0; i<5; i++)
         debug << (unsigned int)MotorControls[motorid[i]][i] << ", ";
      for (int i=0; i<3; i++)
         debug << (unsigned int)MotorPunches[motorid[i]][i] << ", ";
      debug << endl;
   }
#endif
   return;
}

/*! Update the motors controls and punches, the updated values will be sent ot the motors in the next cycle
 MotorControls[][] and MotorPunches[][] are updated to reflect the new position and speeds
 Positions, speeds and punches are unsigned, the value '-1' is used to indicate that particular entry should remain unchanged.
 
 @param positions[] the new target position for each motor (must be of length MOTORS_NUM_MOTORS)
 @param speeds[] the new speeds for each motor (must be of length MOTORS_NUM_MOTORS)
 @param punches[nummotors], the new punches for each motor (must be of length MOTORS_NUM_MOTORS)
 */
void Motors::updateControls(unsigned short positions[], unsigned short speeds[], unsigned short punches[])
{
   updateControls(Motors::IndexToMotorID, MOTORS_NUM_MOTORS, positions, speeds, punches);
}

/* Writes many instruction packets to the motors
 @param motorid[], the unique id of the dynamixels to send the command to
 @param nummotors, the number of motors to send the command to (ie. nummotors = len(motorid)
 @param command, the instruction command (it is the same command sent to each motor)
 @param data[], an array of arrays each of length datalength containing the data associated with the command
 @param datalength, the length of each array in data[] (they must all be the same)
 
 @return true if the send was succesful, false if it fails
 */
bool Motors::write(unsigned char motorid[], unsigned char nummotors, unsigned char command, unsigned char* data[], unsigned char datalength)
{
   // the idea behind this function, is that it *is* faster to put multiply commands into a single message
   // Now things are a little complicated because I have two channels; I need two of everything
   unsigned char lowermessagebuffer[MOTORS_NUM_MOTORS*MAX_MESSAGE_LENGTH];
   unsigned short lowerindex = 0;
   unsigned char uppermessagebuffer[MOTORS_NUM_MOTORS*MAX_MESSAGE_LENGTH];
   unsigned short upperindex = 0;
   
   appendPacketsToBuffer(motorid, nummotors, command, data, datalength, lowermessagebuffer, uppermessagebuffer, &lowerindex, &upperindex);
   
   FT_STATUS status;
   DWORD bytessentlower, bytessentupper;

   status = FT_Write(lowerHandle, lowermessagebuffer, lowerindex, &bytessentlower);
   status += FT_Write(upperHandle, uppermessagebuffer, upperindex, &bytessentupper);
   
   // error checking
   if (status != FT_OK)
   {
      debug << "MOTORS: write(motorid[]) failed with FT_Write returning sum of error codes " << status << endl;
      return false;
   }
   if (bytessentlower+bytessentupper != lowerindex+upperindex) 
   {
      debug << "MOTORS: write(motorid[]) failed to write all the data; only " << bytessentlower+bytessentupper << " bytes were written, instead of " << lowerindex+upperindex;
      return false;
   }
   
   return true;
}

/* Writes an instruction packet to the motors
 @param motorid, the unique id of the dynamixel to send the command to
 @param command, the instruction command
 @param data[], an array of length datalength containing the data associated with the command
 @param datalength, the length of the data[] array
 
 @return true if the send was succesful, false if it fails
*/
bool Motors::write(unsigned char motorid, unsigned char command, unsigned char data[], unsigned char datalength)
{
   /* Instruction packet format 
       0xFF, 0xFF, ID, length, instruction, para1, para2, ..., paraN, checksum
       
       ID = the unique ID of the Dynamixel the instruction is for
       length = number of parameters + 2
       instruction = ping, read data, write data, reg write, action and reset
       checksum = ~(ID + length + instruction + para1,..., paraN) & 0xFF 
   */
   #if DEBUG_NUPLATFORM_VERBOSITY > 0
      debug << "MOTORS: Writing command " << (int)command << " to " << (int)motorid << " ";
   #endif
   
   unsigned char messagebuffer[MAX_MESSAGE_LENGTH];
   unsigned short messagelength = 0;
   
   appendPacketToBuffer(motorid, command, data, datalength, messagebuffer, &messagelength);
   
   FT_STATUS status;
   DWORD bytessent;
   FT_HANDLE fthandle;
   if (motorid == DX117_BROADCASTING_ID)
   {
      status = FT_Write(lowerHandle, messagebuffer, messagelength, &bytessent);
      status += FT_Write(upperHandle, messagebuffer, messagelength, &bytessent);
   }
   else
   {
      if (Motors::MotorIDToLowerBody[motorid])
         fthandle = lowerHandle;
      else
         fthandle = upperHandle;
      status = FT_Write(fthandle, messagebuffer, messagelength, &bytessent);
   }
   
   if (status != FT_OK)
   {
      debug << "MOTORS: write failed with FT_Write returning error code " << status << endl;
      return false;
   }
   if (bytessent != messagelength) 
   {
      debug << "MOTORS: write failed to write all the data; only " << bytessent << " bytes were written, instead of " << messagelength;
      return false;
   }
   
   // debug output
   #if DEBUG_NUPLATFORM_VERBOSITY > 0
      debug << ": ";
   for (unsigned short i=0; i<messagelength; i++)
      debug << (int)messagebuffer[i] << ", ";
   debug << endl;
   #endif
   
   return true;
}

/* Writes an instruction packet to the motors
 @param motorid, the unique id of the dynamixel to send the command to
 @param command, the instruction command
 @param data[], an array of length datalength containing the data associated with the command
 @param datalength, the length of the data[] array
 
 @return true if the send was succesful, false if it fails
 */
bool Motors::write(unsigned char command, unsigned char data[MOTORS_NUM_MOTORS][MOTORS_NUM_CONTROLS])
{
   // the idea behind this function, is that it *is* faster to put multiply commands into a single message
   // Now things are a little complicated because I have two channels; I need two of everything
   unsigned char lowermessagebuffer[MOTORS_NUM_MOTORS*MAX_MESSAGE_LENGTH];
   unsigned short lowerindex = 0;
   unsigned char uppermessagebuffer[MOTORS_NUM_MOTORS*MAX_MESSAGE_LENGTH];
   unsigned short upperindex = 0;
   
   appendPacketsToBuffer(command, data, lowermessagebuffer, uppermessagebuffer, &lowerindex, &upperindex);
   
   FT_STATUS status;
   DWORD bytessentlower, bytessentupper;
   
   status = FT_Write(lowerHandle, lowermessagebuffer, lowerindex, &bytessentlower);
   status += FT_Write(upperHandle, uppermessagebuffer, upperindex, &bytessentupper);
   
   // error checking
   if (status != FT_OK)
   {
      debug << "MOTORS: write(motorid[]) failed with FT_Write returning sum of error codes " << status << endl;
      return false;
   }
   if (bytessentlower+bytessentupper != lowerindex+upperindex) 
   {
      debug << "MOTORS: write(motorid[]) failed to write all the data; only " << bytessentlower+bytessentupper << " bytes were written, instead of " << lowerindex+upperindex;
      return false;
   }
   
   return true;
}

/* Append the control packets (MotorControls) to the lower and upper message buffers
 @param lowermessagebuffer[]: the buffer to store packets to the lower body motors (please ensure it is plenty big)
 @param uppermessagebuffer[]: the buffer to store packets to the upper body motors (please ensure it is plenty big)
 @param lowerindex: the index into lowermessagebuffer[] to start appending the packets, the index will be updated to point to the position after the last one added (so that the index will be both equal to the length, and the position to start appending new data)
 @param upperindex: the index into uppermessagebuffer[] to start appending packets
 */
void Motors::appendControlPacketsToBuffer(unsigned char lowermessagebuffer[], unsigned char uppermessagebuffer[], unsigned short* lowerindex, unsigned short* upperindex)
{
   appendPacketsToBuffer(DX117_WRITE, MotorControls, lowermessagebuffer, uppermessagebuffer, lowerindex, upperindex);
   appendPacketsToBuffer(DX117_WRITE, MotorPunches, lowermessagebuffer, uppermessagebuffer, lowerindex, upperindex);
   return;
}

/* Append packets to the buffer (the same command with different data to each motor in the list)
 
 @param motorid[]: the list of motor ids for each of the packets
 @param nummotors: the length of motorid
 @param command: the command for each packet (the same command is sent to every motor)
 @param data[]: an array of pointers to the data associated with the each packet (the data can be different for each motor, but each entry has to be the same length)
 @param datalength: the length of each entry in datalength
 @param lowermessagebuffer[]: the buffer to store packets to the lower body motors (please ensure it is plenty big)
 @param uppermessagebuffer[]: the buffer to store packets to the upper body motors (please ensure it is plenty big)
 @param lowerindex: the index into lowermessagebuffer[] to start appending the packets, the index will be updated to point to the position after the last one added (so that the index will be both equal to the length, and the position to start appending new data)
 @param upperindex: the index into uppermessagebuffer[] to start appending packets
 */
void Motors::appendPacketsToBuffer(unsigned char motorid[], unsigned char nummotors, unsigned char command, unsigned char* data[], unsigned char datalength, unsigned char lowermessagebuffer[], unsigned char uppermessagebuffer[], unsigned short* lowerindex, unsigned short* upperindex)
{
   unsigned char* messagebuffer;
   unsigned short* index;
   for (unsigned short i=0; i<nummotors; i++)
   {
      if (Motors::MotorIDToLowerBody[motorid[i]])             // decide which buffer this command needs to be added to
      {
         messagebuffer = lowermessagebuffer;
         index = lowerindex;
      }
      else
      {
         messagebuffer = uppermessagebuffer;
         index = upperindex;
      }
      appendPacketToBuffer(motorid[i], command, data[i], datalength, messagebuffer, index);
   }
}

/* APPENDS an instruction packet to the passed message buffer
      This function is designed to constuct a buffer containing many instruction packets, by appending a new packet to the messagebuffer.
      The variable currentbufferindex is used to control the placement of the packet in the buffer. Its value is used as the starting point for the added packet.
      After the packet has been edited currentbufferindex is updated so that it will now be the start of the next packet. In this way it is also the length of the message in the buffer.
 
 @param motorid, the id of the targeted motor
 @param command, the command to be sent
 @param data[], the array of data associated with command
 @param datalength, the length of data[]
 @param messagebuffer[], the buffer to append the packet to
 @param currentbufferindex, the position in the messagebuffer where the packet will be placed
 
Postconditions:
   messagebuffer will have an instruction packet placed such that the beginning is at index currentbufferindex
   currentbufferindex will be updated to point to the position after the last byte of the new packet. In this way it is both where the next packet should be placed, and the length of the message in the buffer.
*/
void Motors::appendPacketToBuffer(unsigned char motorid, unsigned char command, unsigned char data[], unsigned char datalength, unsigned char messagebuffer[], unsigned short* currentbufferindex)
{
   unsigned short offset = *currentbufferindex;
   unsigned char checksum = 0;
   
   // Packet header
   messagebuffer[0+offset] = 0xFF;
   messagebuffer[1+offset] = 0xFF;
   messagebuffer[2+offset] = motorid;
   messagebuffer[3+offset] = datalength + 2;
   messagebuffer[4+offset] = command;
   static unsigned char headerlength = 5;                            // the number of bytes before the command data
   unsigned char packetlength = headerlength + datalength + 1;      // the length of the message header, data, checksum
   
   // Message data
   for (unsigned short i=0; i<datalength; i++)
      messagebuffer[headerlength + i + offset] = data[i];
   
   // Message checksum
   for (unsigned short i=2; i<packetlength-1; i++)
      checksum += messagebuffer[i+offset];          // it is OK if the checksum overflows, because it needs to be clipped to a single byte anyway
   messagebuffer[packetlength-1+offset] = ~checksum;
   *currentbufferindex = *currentbufferindex + packetlength;
}

/* Append packets the message buffers (this is a specialised version to simplify and speed up writing controls to motors)
 
 @param command: the same command is used for each packet
 @param data[][]: carefully formated data for the packets [for each motor][complete packet data (address, data0, data1,...)] 
 @param lowermessagebuffer[]: the buffer to store packets to the lower body motors (please ensure it is plenty big)
 @param uppermessagebuffer[]: the buffer to store packets to the upper body motors (please ensure it is plenty big)
 @param lowerindex: the index into lowermessagebuffer[] to start appending the packets, the index will be updated to point to the position after the last one added (so that the index will be both equal to the length, and the position to start appending new data)
 @param upperindex: the index into uppermessagebuffer[] to start appending packets
 */
void Motors::appendPacketsToBuffer(unsigned char command, unsigned char data[MOTORS_NUM_MOTORS][MOTORS_NUM_CONTROLS], unsigned char lowermessagebuffer[], unsigned char uppermessagebuffer[], unsigned short* lowerindex, unsigned short* upperindex)
{
   unsigned char* messagebuffer;
   unsigned short* index;
   for (unsigned short i=0; i<MOTORS_NUM_MOTORS; i++)
   {
      if (MotorTorqueOn[i])            // writing to a motor that is off will turn it back on
      {
         if (Motors::MotorIDToLowerBody[Motors::IndexToMotorID[i]])             // decide which buffer this command needs to be added to
         {
            messagebuffer = lowermessagebuffer;
            index = lowerindex;
         }
         else
         {
            messagebuffer = uppermessagebuffer;
            index = upperindex;
         }
         appendPacketToBuffer(Motors::IndexToMotorID[i], command, data[i], MOTORS_NUM_CONTROLS, messagebuffer, index);
      }
   }
   return;
}

/* Append packets the message buffers (this is a specialised version to simplify and speed up writing controls to motors)
 
 @param command: the same command is used for each packet
 @param data[][]: carefully formated data for the packets [for each motor][complete packet data (address, data0, data1,...)] 
 @param lowermessagebuffer[]: the buffer to store packets to the lower body motors (please ensure it is plenty big)
 @param uppermessagebuffer[]: the buffer to store packets to the upper body motors (please ensure it is plenty big)
 @param lowerindex: the index into lowermessagebuffer[] to start appending the packets, the index will be updated to point to the position after the last one added (so that the index will be both equal to the length, and the position to start appending new data)
 @param upperindex: the index into uppermessagebuffer[] to start appending packets
 */
void Motors::appendPacketsToBuffer(unsigned char command, unsigned char data[MOTORS_NUM_MOTORS][MOTORS_NUM_PUNCHES], unsigned char lowermessagebuffer[], unsigned char uppermessagebuffer[], unsigned short* lowerindex, unsigned short* upperindex)
{
   unsigned char* messagebuffer;
   unsigned short* index;
   for (unsigned short i=0; i<MOTORS_NUM_MOTORS; i++)
   {
      if (MotorTorqueOn[i])            // writing to a motor that is off will turn it back on
      {
         if (Motors::MotorIDToLowerBody[Motors::IndexToMotorID[i]])             // decide which buffer this command needs to be added to
         {
            messagebuffer = lowermessagebuffer;
            index = lowerindex;
         }
         else
         {
            messagebuffer = uppermessagebuffer;
            index = upperindex;
         }
         appendPacketToBuffer(Motors::IndexToMotorID[i], command, data[i], MOTORS_NUM_PUNCHES, messagebuffer, index);
      }
   }
   return;
}

/* Writes an instruction packet to all motors using the broadcast id
 @param command, the instruction command
 @param data[], an array of length datalength containing the data associated with the command
 @param datalength, the length of the data[] array
 
 @return true if the send was succesful, false if it fails
 */
bool Motors::broadcast(unsigned char command, unsigned char data[], unsigned short datalength)
{
   /* Instruction packet format 
    0xFF, 0xFF, ID, length, instruction, para1, para2, ..., paraN, checksum
    
    ID = the unique ID of the Dynamixel the instruction is for
    length = number of parameters + 2
    instruction = ping, read data, write data, reg write, action and reset
    checksum = ~(ID + length + instruction + para1,..., paraN) & 0xFF 
    */
#if DEBUG_NUPLATFORM_VERBOSITY > 0
   debug << "MOTORS: Broadcasting command " << (int)command << endl;
#endif
   unsigned char messagebuffer[MAX_MESSAGE_LENGTH];
   unsigned char checksum = 0;
   
   // Message header
   messagebuffer[0] = 0xFF;
   messagebuffer[1] = 0xFF;
   messagebuffer[2] = DX117_BROADCASTING_ID;
   messagebuffer[3] = datalength + 2;
   messagebuffer[4] = command;
   static unsigned char headerlength = 5;                            // the number of bytes before the command data
   unsigned char messagelength = headerlength + datalength + 1;      // the length of the message header, data, checksum
   
   // Message data
   for (unsigned short i=0; i<datalength; i++)
      messagebuffer[headerlength + i] = data[i];
   
   // Message checksum
   for (unsigned short i=2; i<messagelength-1; i++)
      checksum += messagebuffer[i];          // it is OK if the checksum overflows, because it needs to be clipped to a single byte anyway
   messagebuffer[messagelength-1] = ~checksum;
   
   FT_STATUS status;
   DWORD bytessent;
   
   status = FT_Write(lowerHandle, messagebuffer, messagelength, &bytessent);
   status = FT_Write(upperHandle, messagebuffer, messagelength, &bytessent);
   
   if (status != FT_OK)
   {
      debug << "MOTORS: broadcast failed with FT_Write returning error code " << status << endl;
      return false;
   }
   if (bytessent != messagelength) 
   {
      debug << "MOTORS: broadcast failed to write all the data; only " << bytessent << " bytes were written, instead of " << messagelength;
      return false;
   }
   
   // debug output
#if DEBUG_NUPLATFORM_VERBOSITY > 0
   debug << "MOTORS: Packets sent: ";
   for (unsigned short i=0; i<messagelength; i++)
      debug << (int)messagebuffer[i] << ", ";
   debug << endl;
#endif
   
   return true;
}

/*! Sends MotorControls[][] to the motors (ie. control to motors, every motor that has MotorTorqueOn[i] == true, will get the controls in MotorControls[i][])
 To control the motors:
   updateControls(newpositions, newspeeds)
   write()        <--- this function
 */
bool Motors::write()
{
   unsigned char lowermessagebuffer[MOTORS_NUM_MOTORS*MAX_MESSAGE_LENGTH];
   unsigned short lowerindex = 0;
   unsigned char uppermessagebuffer[MOTORS_NUM_MOTORS*MAX_MESSAGE_LENGTH];
   unsigned short upperindex = 0;
   
   appendControlPacketsToBuffer(lowermessagebuffer, uppermessagebuffer, &lowerindex, &upperindex);
      
   FT_STATUS status;
   DWORD bytessentlower, bytessentupper;
   
   // I want this function to be as fast as fast can be, so it is 'hard-coded' and threaded
   pthread_t lowerthread, upperthread;
   
   // ----------------------------- status = FT_Write(lowerHandle, lowermessagebuffer, lowerindex, &bytessentlower);
   ThreadLowerWrite.Handle = lowerHandle;
   ThreadLowerWrite.Buffer = lowermessagebuffer;
   ThreadLowerWrite.BufferLength = lowerindex;
   
   // create the lower thread, and set its priority
   int err = pthread_create(&lowerthread, NULL, runThreadedWrite, (void*) &ThreadLowerWrite);
   sched_param param;
   param.sched_priority = 40;
   pthread_setschedparam(lowerthread, SCHED_FIFO, &param);
   
   // ----------------------------- status += FT_Write(upperHandle, uppermessagebuffer, upperindex, &bytessentupper);
   ThreadUpperWrite.Handle = upperHandle;
   ThreadUpperWrite.Buffer = uppermessagebuffer;
   ThreadUpperWrite.BufferLength = upperindex;
   
   // create the lower thread, and set its priority
   err = pthread_create(&upperthread, NULL, runThreadedWrite, (void*) &ThreadUpperWrite);
   param.sched_priority = 39;
   pthread_setschedparam(upperthread, SCHED_FIFO, &param);
   
   pthread_join(lowerthread, NULL);
   pthread_join(upperthread, NULL);
   
   return true;
}

/*! Request new feedback from the data;  A request packet is sent to each motor all nicely timed so that the replies will appear in the read buffer some time later ;)
 
 Note. This function can take sometime to return (4ms or so)
 */
bool Motors::request()
{
    FT_STATUS status;
    DWORD bytessentlower, bytessentupper;
    
    pthread_t lowerthread, upperthread;
    sched_param lowerparam, upperparam;
    lowerparam.sched_priority = 40;
    upperparam.sched_priority = 39;
    
    unsigned char numblocks = max(MOTORS_NUM_LOWER_REQUEST_BLOCKS, MOTORS_NUM_UPPER_REQUEST_BLOCKS);
    ThreadLowerWrite.Handle = lowerHandle;
    ThreadUpperWrite.Handle = upperHandle;
    for (unsigned char i=0; i<numblocks; i++)
    {
        // lower motors
        ThreadLowerWrite.Buffer = MotorRequestsLower[i];
        ThreadLowerWrite.BufferLength = MotorRequestsLowerLength[i];
        pthread_create(&lowerthread, NULL, runThreadedWrite, (void*) &ThreadLowerWrite);
        pthread_setschedparam(lowerthread, SCHED_FIFO, &lowerparam);
        // upper motors
        ThreadUpperWrite.Buffer = MotorRequestsUpper[i];
        ThreadUpperWrite.BufferLength = MotorRequestsUpperLength[i];
        pthread_create(&upperthread, NULL, runThreadedWrite, (void*) &ThreadUpperWrite);
        pthread_setschedparam(upperthread, SCHED_FIFO, &upperparam);
        
        // wait for the threads to complete
        pthread_join(lowerthread, NULL);
        pthread_join(upperthread, NULL);
        
        // sleep for a little while
        struct timespec nextRunTime;                    // The absolute time for the main thread to be executed
        clock_gettime(CLOCK_REALTIME, &nextRunTime);    // Initialise the next run time to be now
        // calculation of next run time
        nextRunTime.tv_nsec += 1e9*2e-3;
        if (nextRunTime.tv_nsec > 1e9)              // we need to be careful with the nanosecond clock overflowing...
        {
            nextRunTime.tv_sec += 1;
            nextRunTime.tv_nsec -= 1e9;
        }
        clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &nextRunTime, NULL);
    }
    return true;
}

/* Read numbytes of data on ftHandle and put it into data[]
 @param fthandle: the handle of the d2xx channel
 @param data[]: the array that is going to get the read data (make sure it is big enough)
 @param numbytestoread: the number of bytes to read
 
 @return returns true if the correct number of bytes have been read, returns false if it was less
 */
bool Motors::read(FT_HANDLE fthandle, unsigned char data[], unsigned short numbytestoread)
{
   #if DEBUG_NUPLATFORM_VERBOSITY > 0
      debug << "MOTORS: Reading buffer:" << endl;
   #endif
   FT_STATUS status;
   DWORD numbytesread;
   status = FT_Read(fthandle, data, numbytestoread, &numbytesread);
   if (status != FT_OK)
   {
      debug << "MOTORS: read failed with FT_READ error code: " << status << endl;
      return false;
   }
   
   #if DEBUG_NUPLATFORM_VERBOSITY > 1
      debug << "MOTORS: Bytes read: " << numbytesread << " :";
      for (unsigned short i=0; i<numbytesread; i++)
         debug << (int)data[i] << ", ";
      debug << endl;
   #endif
   #if DEBUG_NUPLATFORM_VERBOSITY > 0
      debug << numbytesread << endl;
   #endif
   if (numbytesread != numbytestoread)
      return false;
   
   return true;
}

/* Return the number of bytes waiting in the d2xx buffer
 */
unsigned short Motors::getNumBytesInQueue(FT_HANDLE fthandle)
{
   DWORD numbytes;
   FT_STATUS status = FT_GetQueueStatus(fthandle, &numbytes);
   if (status != FT_OK)
   {
      debug << "MOTORS: getNumBytesInQueue failed to get the number of bytes in the queue; error code " << status;
      return 0;
   }
   return numbytes;
}

/* Read upto maxdatalength bytes on fthandle into data
 @param fthandle: the handle to the d2xx channel
 @param data[]: the array to get the data (make sure it is at least maxdatalength long)
 @param maxdatalength: the maximum number of bytes to read
 
 @return the number of bytes in the queue that were read
 */
unsigned short Motors::readQueue(FT_HANDLE fthandle, unsigned char data[], unsigned short maxdatalength)
{
   //debug << "MOTORS: readQueue" << endl;
   unsigned short numbytesinqueue = getNumBytesInQueue(fthandle);
   if (numbytesinqueue > maxdatalength)
      numbytesinqueue = maxdatalength;
   read(fthandle, data, numbytesinqueue);
   return numbytesinqueue;
}

/* Unpack the contents of readdata and copy the results into the global feedback arrays
 
 This function scans through readdata looking for valid packets (need to have a valid header and checksum, if not the packet is discarded).
 When a valid packet is found the id is used to update jointPositions[id], jointSpeeds[id], jointLoads[id]
 @param readdata[]: the data to be used to update the feedback arrays
 @param numbytes: the length of readdata[]
 
 @return the number of successfully updated motors
 */
unsigned char Motors::updateFeedbackData(unsigned char readdata[], unsigned short numbytes)
{
   // debug << "MOTORS: UpdateFeedbackData" << numbytes << endl;
   
   /* DX117 Reply Packet Format:
    0xFF, 0xFF, ID, length, error, para1, para2, ..., para(length-2), checksum
    */
   unsigned short index = 0;
   unsigned char id, length, error, checksum, calculatedchecksum;
   unsigned char packetdata[MAX_MESSAGE_LENGTH];                          // [parm0, parm1, ... , parmN];
   unsigned short numupdates = 0;
   // scan through and find the first header
   while (index < numbytes)
   {
      if(findHeader(readdata, numbytes, &index))
      {
         // debug << "MOTORS: scanData: header end at index: " << index << endl;
         if (index + 2 < numbytes)
         {
            index++;
            id = readdata[index];
            index++;
            length = readdata[index];
         }
         // debug << "MOTORS: scanData: id: " << (int)id << " length: " << (int)length << endl;
         if (length < MAX_MESSAGE_LENGTH)
         {
            if (index + length < numbytes)
            {
               index++;
               error = readdata[index];
               index++;
               calculatedchecksum = id + length + error;
               for (unsigned char i=0; i<length-2; i++)
               {
                  packetdata[i] = readdata[index + i];
                  calculatedchecksum += packetdata[i];
               }
               index += length - 2;
               checksum = readdata[index];
               calculatedchecksum = ~calculatedchecksum;
            
               if ((calculatedchecksum == checksum) && ((length - 2) == NUM_FEEDBACK_MOTOR) && id < MOTORS_MAX_ID && id > MOTORS_MIN_ID)
               {
                  if (error > 0)
                     debug << "MOTORS: Motor " << (int)id << " has error " << (int)error << ", you should look into it ;)" << endl;
                  int index = MotorIDToIndex[id];
                  JointPositions[index] = ((packetdata[1] & 0x3) << 8) + packetdata[0];
                  JointSpeeds[index] = ((packetdata[3] & 0x3) << 8) + packetdata[2];
                  JointLoads[index] = ((packetdata[5] & 0x3) << 8) + packetdata[4];
                  numupdates++;
               }
            }
         }
      }
   }
   return numupdates;
}

/* Scans through readdata looking for a header. If one is found true is return and index is left at the second start byte.
 So accessing the motorid of the packet can be done with readdata[index+1] etc
 @param readdata[]: the data read from the serial port
 @param numbytes: the length of readdata[]
 @param index: the current scanning position in readdata[] 
 
 @return true if a valid header is found, false otherwise
 */
bool Motors::findHeader(unsigned char readdata[], unsigned short numbytes, unsigned short* index)
{
   while (*index < numbytes)
   {
      if (readdata[*index] == DX117_START_BYTE)        // find the first start byte
      {
         *index = *index +1;
         if ((*index < numbytes) && (readdata[*index] == DX117_START_BYTE))      // check the next byte, if it is also a start byte then we have found a header :)
            return true;
      }
      *index = *index + 1;
   }
   return false;
}

/*! Read the serial buffer and update the global feedback arrays
 */
bool Motors::read()
{
   unsigned char data[4096];
   unsigned short bytesread;
   
   struct timespec time;
   clock_gettime(CLOCK_REALTIME, &time);
   JointTime = (time.tv_nsec/1e6 + time.tv_sec*1e3) - StartTime;
   
   bytesread = readQueue(lowerHandle, data, 4096);
   updateFeedbackData(data, bytesread);
   bytesread = readQueue(upperHandle, data, 4096);
   updateFeedbackData(data, bytesread);
   return true;
}

/* Threaded write to a motor serial channel. The structure passed as an arguement contains all of the data necessary to perform the write
 This is not a class function because it is too hard to create a thread using a class member.
 
 @param arg threaddata_t where arg->Handle, arg->Buffer and arg->BufferLength contain the relavent data
 */
void* runThreadedWrite(void *arg)
{
   threaddata_t* data = (threaddata_t*) arg; 
   
   DWORD bytessent;
   FT_STATUS status = FT_Write(data->Handle, data->Buffer, data->BufferLength, &bytessent);
   pthread_exit(NULL);
}




