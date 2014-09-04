/*! @file DarwinSensorNames.h
    @brief Definition of Darwin sensor names (ie. Device names to be used with "Darwin CM730 Motor")
 
    @author Jason Kulk, Aaron Wong
 
 Copyright (c) 2009 Jason Kulk, Aaron Wong
 
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

#ifndef DARWINSENSORNAMES_H
#define DARWINSENSORNAMES_H

#include <string>



class JointID  
	{
	public:
		enum
		{
			ID_R_SHOULDER_PITCH     = 1,
			ID_L_SHOULDER_PITCH     = 2,
			ID_R_SHOULDER_ROLL      = 3,
			ID_L_SHOULDER_ROLL      = 4,
			ID_R_ELBOW              = 5,
			ID_L_ELBOW              = 6,
			ID_R_HIP_YAW            = 7,
			ID_L_HIP_YAW            = 8,
			ID_R_HIP_ROLL           = 9,
			ID_L_HIP_ROLL           = 10,
			ID_R_HIP_PITCH          = 11,
			ID_L_HIP_PITCH          = 12,
			ID_R_KNEE               = 13,
			ID_L_KNEE               = 14,
			ID_R_ANKLE_PITCH        = 15,
			ID_L_ANKLE_PITCH        = 16,
			ID_R_ANKLE_ROLL         = 17,
			ID_L_ANKLE_ROLL         = 18,
			ID_HEAD_PAN             = 19,
			ID_HEAD_TILT            = 20,
			NUMBER_OF_JOINTS
		};
};

/*
// Position
#define HEAD_YAW_POSITION          DN_PREFIX + std::string("HeadYaw/Position") + SN_POSTFIX
#define HEAD_PITCH_POSITION        DN_PREFIX + std::string("HeadPitch/Position") + SN_POSTFIX
#define L_SHOULDER_ROLL_POSITION   DN_PREFIX + std::string("LShoulderRoll/Position") + SN_POSTFIX
#define L_SHOULDER_PITCH_POSITION  DN_PREFIX + std::string("LShoulderPitch/Position") + SN_POSTFIX
#define L_ELBOW_YAW_POSITION       DN_PREFIX + std::string("LElbowYaw/Position") + SN_POSTFIX
#define L_ELBOW_ROLL_POSITION      DN_PREFIX + std::string("LElbowRoll/Position") + SN_POSTFIX
#define R_SHOULDER_ROLL_POSITION   DN_PREFIX + std::string("RShoulderRoll/Position") + SN_POSTFIX
#define R_SHOULDER_PITCH_POSITION  DN_PREFIX + std::string("RShoulderPitch/Position") + SN_POSTFIX
#define R_ELBOW_YAW_POSITION       DN_PREFIX + std::string("RElbowYaw/Position") + SN_POSTFIX 
#define R_ELBOW_ROLL_POSITION      DN_PREFIX + std::string("RElbowRoll/Position") + SN_POSTFIX
#define L_HIP_YAWPITCH_POSITION    DN_PREFIX + std::string("LHipYawPitch/Position") + SN_POSTFIX 
#define L_HIP_ROLL_POSITION        DN_PREFIX + std::string("LHipRoll/Position") + SN_POSTFIX 
#define L_HIP_PITCH_POSITION       DN_PREFIX + std::string("LHipPitch/Position") + SN_POSTFIX
#define L_KNEE_PITCH_POSITION      DN_PREFIX + std::string("LKneePitch/Position") + SN_POSTFIX
#define L_ANKLE_PITCH_POSITION     DN_PREFIX + std::string("LAnklePitch/Position") + SN_POSTFIX 
#define L_ANKLE_ROLL_POSITION      DN_PREFIX + std::string("LAnkleRoll/Position") + SN_POSTFIX
#define R_HIP_YAWPITCH_POSITION    DN_PREFIX + std::string("RHipYawPitch/Position") + SN_POSTFIX
#define R_HIP_ROLL_POSITION        DN_PREFIX + std::string("RHipRoll/Position") + SN_POSTFIX 
#define R_HIP_PITCH_POSITION       DN_PREFIX + std::string("RHipPitch/Position") + SN_POSTFIX
#define R_KNEE_PITCH_POSITION      DN_PREFIX + std::string("RKneePitch/Position") + SN_POSTFIX
#define R_ANKLE_PITCH_POSITION     DN_PREFIX + std::string("RAnklePitch/Position") + SN_POSTFIX 
#define R_ANKLE_ROLL_POSITION      DN_PREFIX + std::string("RAnkleRoll/Position") + SN_POSTFIX

// Current
#define HEAD_YAW_CURRENT          DN_PREFIX + std::string("HeadYaw/ElectricCurrent") + SN_POSTFIX
#define HEAD_PITCH_CURRENT        DN_PREFIX + std::string("HeadPitch/ElectricCurrent") + SN_POSTFIX
#define L_SHOULDER_ROLL_CURRENT   DN_PREFIX + std::string("LShoulderRoll/ElectricCurrent") + SN_POSTFIX
#define L_SHOULDER_PITCH_CURRENT  DN_PREFIX + std::string("LShoulderPitch/ElectricCurrent") + SN_POSTFIX
#define L_ELBOW_YAW_CURRENT       DN_PREFIX + std::string("LElbowYaw/ElectricCurrent") + SN_POSTFIX
#define L_ELBOW_ROLL_CURRENT      DN_PREFIX + std::string("LElbowRoll/ElectricCurrent") + SN_POSTFIX
#define R_SHOULDER_ROLL_CURRENT   DN_PREFIX + std::string("RShoulderRoll/ElectricCurrent") + SN_POSTFIX
#define R_SHOULDER_PITCH_CURRENT  DN_PREFIX + std::string("RShoulderPitch/ElectricCurrent") + SN_POSTFIX
#define R_ELBOW_YAW_CURRENT       DN_PREFIX + std::string("RElbowYaw/ElectricCurrent") + SN_POSTFIX
#define R_ELBOW_ROLL_CURRENT      DN_PREFIX + std::string("RElbowRoll/ElectricCurrent") + SN_POSTFIX
#define L_HIP_YAWPITCH_CURRENT    DN_PREFIX + std::string("LHipYawPitch/ElectricCurrent") + SN_POSTFIX
#define L_HIP_ROLL_CURRENT        DN_PREFIX + std::string("LHipRoll/ElectricCurrent") + SN_POSTFIX
#define L_HIP_PITCH_CURRENT       DN_PREFIX + std::string("LHipPitch/ElectricCurrent") + SN_POSTFIX
#define L_KNEE_PITCH_CURRENT      DN_PREFIX + std::string("LKneePitch/ElectricCurrent") + SN_POSTFIX
#define L_ANKLE_PITCH_CURRENT     DN_PREFIX + std::string("LAnklePitch/ElectricCurrent") + SN_POSTFIX
#define L_ANKLE_ROLL_CURRENT      DN_PREFIX + std::string("LAnkleRoll/ElectricCurrent") + SN_POSTFIX
#define R_HIP_YAWPITCH_CURRENT    DN_PREFIX + std::string("LHipYawPitch/ElectricCurrent") + SN_POSTFIX
#define R_HIP_ROLL_CURRENT        DN_PREFIX + std::string("RHipRoll/ElectricCurrent") + SN_POSTFIX
#define R_HIP_PITCH_CURRENT       DN_PREFIX + std::string("RHipPitch/ElectricCurrent") + SN_POSTFIX
#define R_KNEE_PITCH_CURRENT      DN_PREFIX + std::string("RKneePitch/ElectricCurrent") + SN_POSTFIX
#define R_ANKLE_PITCH_CURRENT     DN_PREFIX + std::string("RAnklePitch/ElectricCurrent") + SN_POSTFIX
#define R_ANKLE_ROLL_CURRENT      DN_PREFIX + std::string("RAnkleRoll/ElectricCurrent") + SN_POSTFIX

// Actuator Target
#define HEAD_YAW_TARGET          DN_PREFIX + std::string("HeadYaw/Position") + AN_POSTFIX
#define HEAD_PITCH_TARGET        DN_PREFIX + std::string("HeadPitch/Position") + AN_POSTFIX
#define L_SHOULDER_ROLL_TARGET   DN_PREFIX + std::string("LShoulderRoll/Position") + AN_POSTFIX
#define L_SHOULDER_PITCH_TARGET  DN_PREFIX + std::string("LShoulderPitch/Position") + AN_POSTFIX
#define L_ELBOW_YAW_TARGET       DN_PREFIX + std::string("LElbowYaw/Position") + AN_POSTFIX 
#define L_ELBOW_ROLL_TARGET      DN_PREFIX + std::string("LElbowRoll/Position") + AN_POSTFIX
#define R_SHOULDER_ROLL_TARGET   DN_PREFIX + std::string("RShoulderRoll/Position") + AN_POSTFIX
#define R_SHOULDER_PITCH_TARGET  DN_PREFIX + std::string("RShoulderPitch/Position") + AN_POSTFIX
#define R_ELBOW_YAW_TARGET       DN_PREFIX + std::string("RElbowYaw/Position") + AN_POSTFIX 
#define R_ELBOW_ROLL_TARGET      DN_PREFIX + std::string("RElbowRoll/Position") + AN_POSTFIX
#define L_HIP_YAWPITCH_TARGET    DN_PREFIX + std::string("LHipYawPitch/Position") + AN_POSTFIX 
#define L_HIP_ROLL_TARGET        DN_PREFIX + std::string("LHipRoll/Position") + AN_POSTFIX 
#define L_HIP_PITCH_TARGET       DN_PREFIX + std::string("LHipPitch/Position") + AN_POSTFIX
#define L_KNEE_PITCH_TARGET      DN_PREFIX + std::string("LKneePitch/Position") + AN_POSTFIX
#define L_ANKLE_PITCH_TARGET     DN_PREFIX + std::string("LAnklePitch/Position") + AN_POSTFIX 
#define L_ANKLE_ROLL_TARGET      DN_PREFIX + std::string("LAnkleRoll/Position") + AN_POSTFIX
#define R_HIP_YAWPITCH_TARGET    DN_PREFIX + std::string("LHipYawPitch/Position") + AN_POSTFIX
#define R_HIP_ROLL_TARGET        DN_PREFIX + std::string("RHipRoll/Position") + AN_POSTFIX 
#define R_HIP_PITCH_TARGET       DN_PREFIX + std::string("RHipPitch/Position") + AN_POSTFIX
#define R_KNEE_PITCH_TARGET      DN_PREFIX + std::string("RKneePitch/Position") + AN_POSTFIX
#define R_ANKLE_PITCH_TARGET     DN_PREFIX + std::string("RAnklePitch/Position") + AN_POSTFIX 
#define R_ANKLE_ROLL_TARGET      DN_PREFIX + std::string("RAnkleRoll/Position") + AN_POSTFIX

// Actuator Target
#define HEAD_YAW_HARDNESS          DN_PREFIX + std::string("HeadYaw/Hardness") + AN_POSTFIX
#define HEAD_PITCH_HARDNESS        DN_PREFIX + std::string("HeadPitch/Hardness") + AN_POSTFIX
#define L_SHOULDER_ROLL_HARDNESS   DN_PREFIX + std::string("LShoulderRoll/Hardness") + AN_POSTFIX
#define L_SHOULDER_PITCH_HARDNESS  DN_PREFIX + std::string("LShoulderPitch/Hardness") + AN_POSTFIX
#define L_ELBOW_YAW_HARDNESS       DN_PREFIX + std::string("LElbowYaw/Hardness") + AN_POSTFIX 
#define L_ELBOW_ROLL_HARDNESS      DN_PREFIX + std::string("LElbowRoll/Hardness") + AN_POSTFIX
#define R_SHOULDER_ROLL_HARDNESS   DN_PREFIX + std::string("RShoulderRoll/Hardness") + AN_POSTFIX
#define R_SHOULDER_PITCH_HARDNESS  DN_PREFIX + std::string("RShoulderPitch/Hardness") + AN_POSTFIX
#define R_ELBOW_YAW_HARDNESS       DN_PREFIX + std::string("RElbowYaw/Hardness") + AN_POSTFIX 
#define R_ELBOW_ROLL_HARDNESS      DN_PREFIX + std::string("RElbowRoll/Hardness") + AN_POSTFIX
#define L_HIP_YAWPITCH_HARDNESS    DN_PREFIX + std::string("LHipYawPitch/Hardness") + AN_POSTFIX 
#define L_HIP_ROLL_HARDNESS        DN_PREFIX + std::string("LHipRoll/Hardness") + AN_POSTFIX 
#define L_HIP_PITCH_HARDNESS       DN_PREFIX + std::string("LHipPitch/Hardness") + AN_POSTFIX
#define L_KNEE_PITCH_HARDNESS      DN_PREFIX + std::string("LKneePitch/Hardness") + AN_POSTFIX
#define L_ANKLE_PITCH_HARDNESS     DN_PREFIX + std::string("LAnklePitch/Hardness") + AN_POSTFIX 
#define L_ANKLE_ROLL_HARDNESS      DN_PREFIX + std::string("LAnkleRoll/Hardness") + AN_POSTFIX
#define R_HIP_YAWPITCH_HARDNESS    DN_PREFIX + std::string("LHipYawPitch/Hardness") + AN_POSTFIX
#define R_HIP_ROLL_HARDNESS        DN_PREFIX + std::string("RHipRoll/Hardness") + AN_POSTFIX 
#define R_HIP_PITCH_HARDNESS       DN_PREFIX + std::string("RHipPitch/Hardness") + AN_POSTFIX
#define R_KNEE_PITCH_HARDNESS      DN_PREFIX + std::string("RKneePitch/Hardness") + AN_POSTFIX
#define R_ANKLE_PITCH_HARDNESS     DN_PREFIX + std::string("RAnklePitch/Hardness") + AN_POSTFIX 
#define R_ANKLE_ROLL_HARDNESS      DN_PREFIX + std::string("RAnkleRoll/Hardness") + AN_POSTFIX

// Temperature
#define HEAD_YAW_TEMPERATURE          DN_PREFIX + std::string("HeadYaw/Temperature") + SN_POSTFIX
#define HEAD_PITCH_TEMPERATURE        DN_PREFIX + std::string("HeadPitch/Temperature") + SN_POSTFIX
#define L_SHOULDER_ROLL_TEMPERATURE   DN_PREFIX + std::string("LShoulderRoll/Temperature") + SN_POSTFIX
#define L_SHOULDER_PITCH_TEMPERATURE  DN_PREFIX + std::string("LShoulderPitch/Temperature") + SN_POSTFIX
#define L_ELBOW_YAW_TEMPERATURE       DN_PREFIX + std::string("LElbowYaw/Temperature") + SN_POSTFIX
#define L_ELBOW_ROLL_TEMPERATURE      DN_PREFIX + std::string("LElbowRoll/Temperature") + SN_POSTFIX
#define R_SHOULDER_ROLL_TEMPERATURE   DN_PREFIX + std::string("RShoulderRoll/Temperature") + SN_POSTFIX
#define R_SHOULDER_PITCH_TEMPERATURE  DN_PREFIX + std::string("RShoulderPitch/Temperature") + SN_POSTFIX
#define R_ELBOW_YAW_TEMPERATURE       DN_PREFIX + std::string("RElbowYaw/Temperature") + SN_POSTFIX
#define R_ELBOW_ROLL_TEMPERATURE      DN_PREFIX + std::string("RElbowRoll/Temperature") + SN_POSTFIX
#define L_HIP_YAWPITCH_TEMPERATURE    DN_PREFIX + std::string("LHipYawPitch/Temperature") + SN_POSTFIX
#define L_HIP_ROLL_TEMPERATURE        DN_PREFIX + std::string("LHipRoll/Temperature") + SN_POSTFIX
#define L_HIP_PITCH_TEMPERATURE       DN_PREFIX + std::string("LHipPitch/Temperature") + SN_POSTFIX
#define L_KNEE_PITCH_TEMPERATURE      DN_PREFIX + std::string("LKneePitch/Temperature") + SN_POSTFIX
#define L_ANKLE_PITCH_TEMPERATURE     DN_PREFIX + std::string("LAnklePitch/Temperature") + SN_POSTFIX
#define L_ANKLE_ROLL_TEMPERATURE      DN_PREFIX + std::string("LAnkleRoll/Temperature") + SN_POSTFIX
#define R_HIP_YAWPITCH_TEMPERATURE    DN_PREFIX + std::string("LHipYawPitch/Temperature") + SN_POSTFIX
#define R_HIP_ROLL_TEMPERATURE        DN_PREFIX + std::string("RHipRoll/Temperature") + SN_POSTFIX
#define R_HIP_PITCH_TEMPERATURE       DN_PREFIX + std::string("RHipPitch/Temperature") + SN_POSTFIX
#define R_KNEE_PITCH_TEMPERATURE      DN_PREFIX + std::string("RKneePitch/Temperature") + SN_POSTFIX
#define R_ANKLE_PITCH_TEMPERATURE     DN_PREFIX + std::string("RAnklePitch/Temperature") + SN_POSTFIX
#define R_ANKLE_ROLL_TEMPERATURE      DN_PREFIX + std::string("RAnkleRoll/Temperature") + SN_POSTFIX

// Balance
#define ACCEL_X                  		DN_PREFIX + std::string("InertialSensor/AccX") + SN_POSTFIX
#define ACCEL_Y                  		DN_PREFIX + std::string("InertialSensor/AccY") + SN_POSTFIX
#define ACCEL_Z                  		DN_PREFIX + std::string("InertialSensor/AccZ") + SN_POSTFIX
#define ANGLE_X                  		DN_PREFIX + std::string("InertialSensor/AngleX") + SN_POSTFIX
#define ANGLE_Y                  		DN_PREFIX + std::string("InertialSensor/AngleY") + SN_POSTFIX
#define GYRO_X                   		DN_PREFIX + std::string("InertialSensor/GyrX") + SN_POSTFIX
#define GYRO_Y                   		DN_PREFIX + std::string("InertialSensor/GyrY") + SN_POSTFIX

// Touch
#define L_FSR_FL                 		DN_PREFIX + std::string("LFoot/FSR/FrontLeft") + SN_POSTFIX
#define L_FSR_FR                 		DN_PREFIX + std::string("LFoot/FSR/FrontRight") + SN_POSTFIX
#define L_FSR_BL                 		DN_PREFIX + std::string("LFoot/FSR/RearLeft") + SN_POSTFIX
#define L_FSR_BR                 		DN_PREFIX + std::string("LFoot/FSR/RearRight") + SN_POSTFIX
#define L_BUMP_L                 		DN_PREFIX + std::string("LFoot/Bumper/Left") + SN_POSTFIX
#define L_BUMP_R                 		DN_PREFIX + std::string("LFoot/Bumper/Right") + SN_POSTFIX
#define R_FSR_FL                 		DN_PREFIX + std::string("RFoot/FSR/FrontLeft") + SN_POSTFIX
#define R_FSR_FR                 		DN_PREFIX + std::string("RFoot/FSR/FrontRight") + SN_POSTFIX
#define R_FSR_BL                 		DN_PREFIX + std::string("RFoot/FSR/RearLeft") + SN_POSTFIX
#define R_FSR_BR                 		DN_PREFIX + std::string("RFoot/FSR/RearRight") + SN_POSTFIX
#define R_BUMP_L                 		DN_PREFIX + std::string("RFoot/Bumper/Left") + SN_POSTFIX
#define R_BUMP_R                 		DN_PREFIX + std::string("RFoot/Bumper/Right") + SN_POSTFIX
#define CHEST_BUTTON             		DN_PREFIX + std::string("ChestBoard/Button") + SN_POSTFIX

// Battery
#define CHARGE                   		DN_PREFIX + std::string("Battery/Charge") + SN_POSTFIX
#define CURRENT                  		DN_PREFIX + std::string("Battery/Current") + SN_POSTFIX
#define VOLTAGE_MIN              		DN_PREFIX + std::string("Battery/Charge/Sensor/CellVoltageMin")
#define VOLTAGE_MAX              		DN_PREFIX + std::string("Battery/Charge/Sensor/CellVoltageMax")
#define TEMPERATURE              		DN_PREFIX + std::string("Battery/Temperature") + SN_POSTFIX

// Distance (ultrasonic and infrared sensors)
#define US_DISTANCE_LEFT_VALUE0         DN_PREFIX + std::string("US/Left") + SN_POSTFIX
#define US_DISTANCE_LEFT_VALUE1         DN_PREFIX + std::string("US/Left") + SN_POSTFIX + "1"
#define US_DISTANCE_LEFT_VALUE2         DN_PREFIX + std::string("US/Left") + SN_POSTFIX + "2"
#define US_DISTANCE_LEFT_VALUE3         DN_PREFIX + std::string("US/Left") + SN_POSTFIX + "3"
#define US_DISTANCE_LEFT_VALUE4         DN_PREFIX + std::string("US/Left") + SN_POSTFIX + "4"
#define US_DISTANCE_LEFT_VALUE5         DN_PREFIX + std::string("US/Left") + SN_POSTFIX + "5"
#define US_DISTANCE_LEFT_VALUE6         DN_PREFIX + std::string("US/Left") + SN_POSTFIX + "6"
#define US_DISTANCE_LEFT_VALUE7         DN_PREFIX + std::string("US/Left") + SN_POSTFIX + "7"
#define US_DISTANCE_LEFT_VALUE8         DN_PREFIX + std::string("US/Left") + SN_POSTFIX + "8"
#define US_DISTANCE_LEFT_VALUE9         DN_PREFIX + std::string("US/Left") + SN_POSTFIX + "9"

#define US_DISTANCE_RIGHT_VALUE0        DN_PREFIX + std::string("US/Right") + SN_POSTFIX
#define US_DISTANCE_RIGHT_VALUE1        DN_PREFIX + std::string("US/Right") + SN_POSTFIX + "1"
#define US_DISTANCE_RIGHT_VALUE2        DN_PREFIX + std::string("US/Right") + SN_POSTFIX + "2"
#define US_DISTANCE_RIGHT_VALUE3        DN_PREFIX + std::string("US/Right") + SN_POSTFIX + "3"
#define US_DISTANCE_RIGHT_VALUE4        DN_PREFIX + std::string("US/Right") + SN_POSTFIX + "4"
#define US_DISTANCE_RIGHT_VALUE5        DN_PREFIX + std::string("US/Right") + SN_POSTFIX + "5"
#define US_DISTANCE_RIGHT_VALUE6        DN_PREFIX + std::string("US/Right") + SN_POSTFIX + "6"
#define US_DISTANCE_RIGHT_VALUE7        DN_PREFIX + std::string("US/Right") + SN_POSTFIX + "7"
#define US_DISTANCE_RIGHT_VALUE8        DN_PREFIX + std::string("US/Right") + SN_POSTFIX + "8"
#define US_DISTANCE_RIGHT_VALUE9        DN_PREFIX + std::string("US/Right") + SN_POSTFIX + "9"
*/

#endif

