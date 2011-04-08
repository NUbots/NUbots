/*! @file MotorConstants.cpp
    @brief Declaration of constants for the Bear's serial

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

unsigned char MotorConstants::e_MotorIDToLowerBody[MOTORS_MAX_ID+1] = {MOTORS_LOWER_BODY, MOTORS_LOWER_BODY, MOTORS_LOWER_BODY, MOTORS_LOWER_BODY, MOTORS_LOWER_BODY, MOTORS_LOWER_BODY, MOTORS_LOWER_BODY, MOTORS_LOWER_BODY, MOTORS_LOWER_BODY, MOTORS_LOWER_BODY, MOTORS_LOWER_BODY, MOTORS_LOWER_BODY, MOTORS_LOWER_BODY, MOTORS_LOWER_BODY, MOTORS_LOWER_BODY, MOTORS_LOWER_BODY, MOTORS_LOWER_BODY, MOTORS_LOWER_BODY, MOTORS_LOWER_BODY, MOTORS_LOWER_BODY, MOTORS_LOWER_BODY, MOTORS_LOWER_BODY, MOTORS_LOWER_BODY};
unsigned char MotorConstants::e_IndexToMotorID[MOTORS_NUM_MOTORS] = {22, 21, 20, 6, 4, 8, 7, 5, 9, 3, 2, 12, 10, 14, 18, 16, 13, 11, 15, 19, 17};
unsigned char MotorConstants::e_MotorIDToIndex[MOTORS_MAX_ID+1] = {-1, -1, 10, 9, 4, 7, 3, 6, 5, 8, 12, 17, 11, 16, 13, 18, 15, 20, 14, 19, 2, 1, 0};
unsigned char MotorConstants::e_LowerBodyIndexToMotorID[MOTORS_NUM_LOWER_MOTORS] = {2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22};
unsigned char MotorConstants::e_UpperBodyIndexToMotorID[MOTORS_NUM_UPPER_MOTORS] = {};

//                                                                         HP     HY    NP   LSR   LSP   LEP   RSR   RSP   REP   TR    TY   LHR   LHP    LK   LAR   LAP   RHR   RHP    RK   RAR   RAP
//                                                                          0     1     2     3     4     5     6     7     8     9    10    11    12    13    14    15    16    17    18    19    20  
char MotorConstants::e_MotorSigns[MOTORS_NUM_MOTORS] =                   { -1,    1,    1,   -1,    1,    1,   -1,   -1,   -1,   -1,    1,   -1,    1,    1,    1,    1,   -1,   -1,   -1,    1,   -1}; 
unsigned short MotorConstants::e_DefaultPositions[MOTORS_NUM_MOTORS] =   {499,  481,  374,  709,  526,  612,  318,  500,  412,  525,  528,  657,  372,  531,  773,  508,  563,  642,  514,  500,  519}; 
unsigned short MotorConstants::e_DefaultSpeeds[MOTORS_NUM_MOTORS] =      {100,  100,  100,  100,  100,  100,  100,  100,  100,  100,  100,  100,  100,  100,  100,  100,  100,  100,  100,  100,  100}; 
unsigned char MotorConstants::e_DefaultSlopes[MOTORS_NUM_MOTORS] =       {006,  006,  005,  005,  004,  006,  005,  004,  006,  006,  006,  005,  005,  006,  007,  006,  005,  005,  006,  007,  006}; 
unsigned char MotorConstants::e_DefaultMargins[MOTORS_NUM_MOTORS] =      {000,  000,  000,  000,  000,  000,  000,  000,  000,  000,  000,  000,  000,  000,  000,  000,  000,  000,  000,  000,  000}; 
unsigned short MotorConstants::e_DefaultPunches[MOTORS_NUM_MOTORS] =     {000,  000,  000,  000,  000,  000,  000,  000,  000,  000,  000,  000,  000,  000,  000,  000,  000,  000,  000,  000,  000}; 


