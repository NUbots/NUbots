/*! @file MotorConstants.h
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

#ifndef MOTOR_CONSTANTS_H
#define MOTOR_CONSTANTS_H

#define MOTORS_UPPER_BODY                 0
#define MOTORS_LOWER_BODY                 1

#define MOTORS_NUM_MOTORS                 23
#define MOTORS_NUM_LOWER_MOTORS           12
#define MOTORS_NUM_UPPER_MOTORS           11
#define MOTORS_MIN_ID                     0
#define MOTORS_MAX_ID                     22

#define MOTORS_NUM_MOTORS_PER_BLOCK       6                       // the number of motors in a bulk read_data message
#define MOTORS_NUM_LOWER_REQUEST_BLOCKS   (MOTORS_NUM_LOWER_MOTORS+(MOTORS_NUM_MOTORS_PER_BLOCK-1))/MOTORS_NUM_MOTORS_PER_BLOCK        // the number of bulk read_data messages required to poll all of the motors in the lower body
#define MOTORS_NUM_UPPER_REQUEST_BLOCKS   (MOTORS_NUM_UPPER_MOTORS+(MOTORS_NUM_MOTORS_PER_BLOCK-1))/MOTORS_NUM_MOTORS_PER_BLOCK

class MotorConstants
{
public:
    static unsigned char e_MotorIDToLowerBody[];
    static unsigned char e_IndexToMotorID[];
    static unsigned char e_MotorIDToIndex[];
    static unsigned char e_LowerBodyIndexToMotorID[];
    static unsigned char e_UpperBodyIndexToMotorID[];

    static char e_MotorSigns[];
    static unsigned short e_DefaultPositions[];
    static unsigned short e_DefaultSpeeds[];
    static unsigned char e_DefaultSlopes[];
    static unsigned char e_DefaultMargins[];
    static unsigned short e_DefaultPunches[];
};


#endif
