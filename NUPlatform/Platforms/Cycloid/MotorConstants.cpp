/*! @file MotorConstants.cpp
    @brief Declaration of constants for the Cycloid's serial

    @author Jason Kulk
 
 Copyright (c) 2008, 2009, 2010, 2011 Jason Kulk
 
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
#include "MotorConstants.h"
unsigned char MotorConstants::e_MotorIDToLowerBody[MOTORS_MAX_ID+1] = {MOTORS_UPPER_BODY, MOTORS_UPPER_BODY, MOTORS_UPPER_BODY, MOTORS_UPPER_BODY, MOTORS_UPPER_BODY, MOTORS_UPPER_BODY, MOTORS_UPPER_BODY, MOTORS_UPPER_BODY, MOTORS_UPPER_BODY, MOTORS_UPPER_BODY, MOTORS_LOWER_BODY, MOTORS_LOWER_BODY, MOTORS_LOWER_BODY, MOTORS_LOWER_BODY, MOTORS_LOWER_BODY, MOTORS_LOWER_BODY, MOTORS_LOWER_BODY, MOTORS_LOWER_BODY, MOTORS_LOWER_BODY, MOTORS_LOWER_BODY, MOTORS_LOWER_BODY, MOTORS_LOWER_BODY, MOTORS_UPPER_BODY};
unsigned char MotorConstants::e_IndexToMotorID[MOTORS_NUM_MOTORS] = {0, 4, 2, 8, 6, 5, 3, 9, 7, 1, 22, 10, 14, 12, 16, 20, 18, 11, 15, 13, 17, 21, 19};
unsigned char MotorConstants::e_MotorIDToIndex[MOTORS_MAX_ID+1] = {0, 9, 2, 6, 1, 5, 4, 8, 3, 7, 11, 17, 13, 19, 12, 18, 14, 20, 16, 22, 15, 21, 10};
unsigned char MotorConstants::e_LowerBodyIndexToMotorID[MOTORS_NUM_LOWER_MOTORS] = {10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21};
unsigned char MotorConstants::e_UpperBodyIndexToMotorID[MOTORS_NUM_UPPER_MOTORS] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 22};

//                                                                         HP   LSR   LSP   LEP   LEY   RSR   RSP   REP   REY    TP    TY   LHR   LHP   LHY   LK   LAR   LAP   RHR   RHP    RHY   RK   RAR   RAP
//                                                                          0    1     2     3     4     5     6     7     8     9    10    11    12    13    14    15    16    17    18    19    20    21    22
char MotorConstants::e_MotorSigns[MOTORS_NUM_MOTORS] =                   {  1,  -1,    1,   -1,   -1,   -1,   -1,    1,   -1,    1,   -1,    1,    1,   -1,   -1,   -1,    1,    1,   -1,   -1,    1,   -1,   -1};
unsigned short MotorConstants::e_DefaultPositions[MOTORS_NUM_MOTORS] =   {495, 651,  327,  400,  502,  343,  706,  595,  501,  442,  480,  492,  631,  538,  945,  533,  463,  528,  389,  471,   87,  480,  558};
unsigned short MotorConstants::e_DefaultSpeeds[MOTORS_NUM_MOTORS] =      {100, 100,  100,  100,  100,  100,  100,  100,  100,  100,  100,  100,  100,  100,  100,  100,  100,  100,  100,  100,  100,  100,  100}; 
unsigned char MotorConstants::e_DefaultSlopes[MOTORS_NUM_MOTORS] =       {007, 007,  007,  007,  007,  007,  007,  007,  007,  007,  007,  005,  003,  005,  003,  005,  004,  005,  003,  005,  003,  005,  004}; 
unsigned char MotorConstants::e_DefaultMargins[MOTORS_NUM_MOTORS] =      {000, 000,  000,  000,  000,  000,  000,  000,  000,  000,  000,  000,  000,  000,  000,  000,  000,  000,  000,  000,  000,  000,  000}; 
unsigned short MotorConstants::e_DefaultPunches[MOTORS_NUM_MOTORS] =     {000, 000,  000,  000,  000,  000,  000,  000,  000,  000,  000,  000,  000,  000,  000,  000,  000,  000,  000,  000,  000,  000,  000}; 


