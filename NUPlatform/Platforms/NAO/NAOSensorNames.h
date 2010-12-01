/*! @file NAOSensorNames.h
    @brief Definition of NAO sensor names (ie. Device names to be used with almemory)
 
    @author Jason Kulk
 
 Copyright (c) 2009 Jason Kulk
 
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

#ifndef NAOSENSORNAMES_H
#define NAOSENSORNAMES_H

#include <string>
using namespace std;

#define DN_PREFIX                  string("Device/SubDeviceList/")
#define SN_POSTFIX                 string("/Sensor/Value")
#define AN_POSTFIX				   string("/Actuator/Value")

// Position
#define HEAD_YAW_POSITION          DN_PREFIX + string("HeadYaw/Position") + SN_POSTFIX
#define HEAD_PITCH_POSITION        DN_PREFIX + string("HeadPitch/Position") + SN_POSTFIX
#define L_SHOULDER_ROLL_POSITION   DN_PREFIX + string("LShoulderRoll/Position") + SN_POSTFIX
#define L_SHOULDER_PITCH_POSITION  DN_PREFIX + string("LShoulderPitch/Position") + SN_POSTFIX
#define L_ELBOW_YAW_POSITION       DN_PREFIX + string("LElbowYaw/Position") + SN_POSTFIX
#define L_ELBOW_ROLL_POSITION      DN_PREFIX + string("LElbowRoll/Position") + SN_POSTFIX
#define R_SHOULDER_ROLL_POSITION   DN_PREFIX + string("RShoulderRoll/Position") + SN_POSTFIX
#define R_SHOULDER_PITCH_POSITION  DN_PREFIX + string("RShoulderPitch/Position") + SN_POSTFIX
#define R_ELBOW_YAW_POSITION       DN_PREFIX + string("RElbowYaw/Position") + SN_POSTFIX 
#define R_ELBOW_ROLL_POSITION      DN_PREFIX + string("RElbowRoll/Position") + SN_POSTFIX
#define L_HIP_YAWPITCH_POSITION    DN_PREFIX + string("LHipYawPitch/Position") + SN_POSTFIX 
#define L_HIP_ROLL_POSITION        DN_PREFIX + string("LHipRoll/Position") + SN_POSTFIX 
#define L_HIP_PITCH_POSITION       DN_PREFIX + string("LHipPitch/Position") + SN_POSTFIX
#define L_KNEE_PITCH_POSITION      DN_PREFIX + string("LKneePitch/Position") + SN_POSTFIX
#define L_ANKLE_PITCH_POSITION     DN_PREFIX + string("LAnklePitch/Position") + SN_POSTFIX 
#define L_ANKLE_ROLL_POSITION      DN_PREFIX + string("LAnkleRoll/Position") + SN_POSTFIX
#define R_HIP_YAWPITCH_POSITION    DN_PREFIX + string("RHipYawPitch/Position") + SN_POSTFIX
#define R_HIP_ROLL_POSITION        DN_PREFIX + string("RHipRoll/Position") + SN_POSTFIX 
#define R_HIP_PITCH_POSITION       DN_PREFIX + string("RHipPitch/Position") + SN_POSTFIX
#define R_KNEE_PITCH_POSITION      DN_PREFIX + string("RKneePitch/Position") + SN_POSTFIX
#define R_ANKLE_PITCH_POSITION     DN_PREFIX + string("RAnklePitch/Position") + SN_POSTFIX 
#define R_ANKLE_ROLL_POSITION      DN_PREFIX + string("RAnkleRoll/Position") + SN_POSTFIX

// Current
#define HEAD_YAW_CURRENT          DN_PREFIX + string("HeadYaw/ElectricCurrent") + SN_POSTFIX
#define HEAD_PITCH_CURRENT        DN_PREFIX + string("HeadPitch/ElectricCurrent") + SN_POSTFIX
#define L_SHOULDER_ROLL_CURRENT   DN_PREFIX + string("LShoulderRoll/ElectricCurrent") + SN_POSTFIX
#define L_SHOULDER_PITCH_CURRENT  DN_PREFIX + string("LShoulderPitch/ElectricCurrent") + SN_POSTFIX
#define L_ELBOW_YAW_CURRENT       DN_PREFIX + string("LElbowYaw/ElectricCurrent") + SN_POSTFIX
#define L_ELBOW_ROLL_CURRENT      DN_PREFIX + string("LElbowRoll/ElectricCurrent") + SN_POSTFIX
#define R_SHOULDER_ROLL_CURRENT   DN_PREFIX + string("RShoulderRoll/ElectricCurrent") + SN_POSTFIX
#define R_SHOULDER_PITCH_CURRENT  DN_PREFIX + string("RShoulderPitch/ElectricCurrent") + SN_POSTFIX
#define R_ELBOW_YAW_CURRENT       DN_PREFIX + string("RElbowYaw/ElectricCurrent") + SN_POSTFIX
#define R_ELBOW_ROLL_CURRENT      DN_PREFIX + string("RElbowRoll/ElectricCurrent") + SN_POSTFIX
#define L_HIP_YAWPITCH_CURRENT    DN_PREFIX + string("LHipYawPitch/ElectricCurrent") + SN_POSTFIX
#define L_HIP_ROLL_CURRENT        DN_PREFIX + string("LHipRoll/ElectricCurrent") + SN_POSTFIX
#define L_HIP_PITCH_CURRENT       DN_PREFIX + string("LHipPitch/ElectricCurrent") + SN_POSTFIX
#define L_KNEE_PITCH_CURRENT      DN_PREFIX + string("LKneePitch/ElectricCurrent") + SN_POSTFIX
#define L_ANKLE_PITCH_CURRENT     DN_PREFIX + string("LAnklePitch/ElectricCurrent") + SN_POSTFIX
#define L_ANKLE_ROLL_CURRENT      DN_PREFIX + string("LAnkleRoll/ElectricCurrent") + SN_POSTFIX
#define R_HIP_YAWPITCH_CURRENT    DN_PREFIX + string("LHipYawPitch/ElectricCurrent") + SN_POSTFIX
#define R_HIP_ROLL_CURRENT        DN_PREFIX + string("RHipRoll/ElectricCurrent") + SN_POSTFIX
#define R_HIP_PITCH_CURRENT       DN_PREFIX + string("RHipPitch/ElectricCurrent") + SN_POSTFIX
#define R_KNEE_PITCH_CURRENT      DN_PREFIX + string("RKneePitch/ElectricCurrent") + SN_POSTFIX
#define R_ANKLE_PITCH_CURRENT     DN_PREFIX + string("RAnklePitch/ElectricCurrent") + SN_POSTFIX
#define R_ANKLE_ROLL_CURRENT      DN_PREFIX + string("RAnkleRoll/ElectricCurrent") + SN_POSTFIX

// Actuator Target
#define HEAD_YAW_TARGET          DN_PREFIX + string("HeadYaw/Position") + AN_POSTFIX
#define HEAD_PITCH_TARGET        DN_PREFIX + string("HeadPitch/Position") + AN_POSTFIX
#define L_SHOULDER_ROLL_TARGET   DN_PREFIX + string("LShoulderRoll/Position") + AN_POSTFIX
#define L_SHOULDER_PITCH_TARGET  DN_PREFIX + string("LShoulderPitch/Position") + AN_POSTFIX
#define L_ELBOW_YAW_TARGET       DN_PREFIX + string("LElbowYaw/Position") + AN_POSTFIX 
#define L_ELBOW_ROLL_TARGET      DN_PREFIX + string("LElbowRoll/Position") + AN_POSTFIX
#define R_SHOULDER_ROLL_TARGET   DN_PREFIX + string("RShoulderRoll/Position") + AN_POSTFIX
#define R_SHOULDER_PITCH_TARGET  DN_PREFIX + string("RShoulderPitch/Position") + AN_POSTFIX
#define R_ELBOW_YAW_TARGET       DN_PREFIX + string("RElbowYaw/Position") + AN_POSTFIX 
#define R_ELBOW_ROLL_TARGET      DN_PREFIX + string("RElbowRoll/Position") + AN_POSTFIX
#define L_HIP_YAWPITCH_TARGET    DN_PREFIX + string("LHipYawPitch/Position") + AN_POSTFIX 
#define L_HIP_ROLL_TARGET        DN_PREFIX + string("LHipRoll/Position") + AN_POSTFIX 
#define L_HIP_PITCH_TARGET       DN_PREFIX + string("LHipPitch/Position") + AN_POSTFIX
#define L_KNEE_PITCH_TARGET      DN_PREFIX + string("LKneePitch/Position") + AN_POSTFIX
#define L_ANKLE_PITCH_TARGET     DN_PREFIX + string("LAnklePitch/Position") + AN_POSTFIX 
#define L_ANKLE_ROLL_TARGET      DN_PREFIX + string("LAnkleRoll/Position") + AN_POSTFIX
#define R_HIP_YAWPITCH_TARGET    DN_PREFIX + string("LHipYawPitch/Position") + AN_POSTFIX
#define R_HIP_ROLL_TARGET        DN_PREFIX + string("RHipRoll/Position") + AN_POSTFIX 
#define R_HIP_PITCH_TARGET       DN_PREFIX + string("RHipPitch/Position") + AN_POSTFIX
#define R_KNEE_PITCH_TARGET      DN_PREFIX + string("RKneePitch/Position") + AN_POSTFIX
#define R_ANKLE_PITCH_TARGET     DN_PREFIX + string("RAnklePitch/Position") + AN_POSTFIX 
#define R_ANKLE_ROLL_TARGET      DN_PREFIX + string("RAnkleRoll/Position") + AN_POSTFIX

// Actuator Target
#define HEAD_YAW_HARDNESS          DN_PREFIX + string("HeadYaw/Hardness") + AN_POSTFIX
#define HEAD_PITCH_HARDNESS        DN_PREFIX + string("HeadPitch/Hardness") + AN_POSTFIX
#define L_SHOULDER_ROLL_HARDNESS   DN_PREFIX + string("LShoulderRoll/Hardness") + AN_POSTFIX
#define L_SHOULDER_PITCH_HARDNESS  DN_PREFIX + string("LShoulderPitch/Hardness") + AN_POSTFIX
#define L_ELBOW_YAW_HARDNESS       DN_PREFIX + string("LElbowYaw/Hardness") + AN_POSTFIX 
#define L_ELBOW_ROLL_HARDNESS      DN_PREFIX + string("LElbowRoll/Hardness") + AN_POSTFIX
#define R_SHOULDER_ROLL_HARDNESS   DN_PREFIX + string("RShoulderRoll/Hardness") + AN_POSTFIX
#define R_SHOULDER_PITCH_HARDNESS  DN_PREFIX + string("RShoulderPitch/Hardness") + AN_POSTFIX
#define R_ELBOW_YAW_HARDNESS       DN_PREFIX + string("RElbowYaw/Hardness") + AN_POSTFIX 
#define R_ELBOW_ROLL_HARDNESS      DN_PREFIX + string("RElbowRoll/Hardness") + AN_POSTFIX
#define L_HIP_YAWPITCH_HARDNESS    DN_PREFIX + string("LHipYawPitch/Hardness") + AN_POSTFIX 
#define L_HIP_ROLL_HARDNESS        DN_PREFIX + string("LHipRoll/Hardness") + AN_POSTFIX 
#define L_HIP_PITCH_HARDNESS       DN_PREFIX + string("LHipPitch/Hardness") + AN_POSTFIX
#define L_KNEE_PITCH_HARDNESS      DN_PREFIX + string("LKneePitch/Hardness") + AN_POSTFIX
#define L_ANKLE_PITCH_HARDNESS     DN_PREFIX + string("LAnklePitch/Hardness") + AN_POSTFIX 
#define L_ANKLE_ROLL_HARDNESS      DN_PREFIX + string("LAnkleRoll/Hardness") + AN_POSTFIX
#define R_HIP_YAWPITCH_HARDNESS    DN_PREFIX + string("LHipYawPitch/Hardness") + AN_POSTFIX
#define R_HIP_ROLL_HARDNESS        DN_PREFIX + string("RHipRoll/Hardness") + AN_POSTFIX 
#define R_HIP_PITCH_HARDNESS       DN_PREFIX + string("RHipPitch/Hardness") + AN_POSTFIX
#define R_KNEE_PITCH_HARDNESS      DN_PREFIX + string("RKneePitch/Hardness") + AN_POSTFIX
#define R_ANKLE_PITCH_HARDNESS     DN_PREFIX + string("RAnklePitch/Hardness") + AN_POSTFIX 
#define R_ANKLE_ROLL_HARDNESS      DN_PREFIX + string("RAnkleRoll/Hardness") + AN_POSTFIX

// Temperature
#define HEAD_YAW_TEMPERATURE          DN_PREFIX + string("HeadYaw/Temperature") + SN_POSTFIX
#define HEAD_PITCH_TEMPERATURE        DN_PREFIX + string("HeadPitch/Temperature") + SN_POSTFIX
#define L_SHOULDER_ROLL_TEMPERATURE   DN_PREFIX + string("LShoulderRoll/Temperature") + SN_POSTFIX
#define L_SHOULDER_PITCH_TEMPERATURE  DN_PREFIX + string("LShoulderPitch/Temperature") + SN_POSTFIX
#define L_ELBOW_YAW_TEMPERATURE       DN_PREFIX + string("LElbowYaw/Temperature") + SN_POSTFIX
#define L_ELBOW_ROLL_TEMPERATURE      DN_PREFIX + string("LElbowRoll/Temperature") + SN_POSTFIX
#define R_SHOULDER_ROLL_TEMPERATURE   DN_PREFIX + string("RShoulderRoll/Temperature") + SN_POSTFIX
#define R_SHOULDER_PITCH_TEMPERATURE  DN_PREFIX + string("RShoulderPitch/Temperature") + SN_POSTFIX
#define R_ELBOW_YAW_TEMPERATURE       DN_PREFIX + string("RElbowYaw/Temperature") + SN_POSTFIX
#define R_ELBOW_ROLL_TEMPERATURE      DN_PREFIX + string("RElbowRoll/Temperature") + SN_POSTFIX
#define L_HIP_YAWPITCH_TEMPERATURE    DN_PREFIX + string("LHipYawPitch/Temperature") + SN_POSTFIX
#define L_HIP_ROLL_TEMPERATURE        DN_PREFIX + string("LHipRoll/Temperature") + SN_POSTFIX
#define L_HIP_PITCH_TEMPERATURE       DN_PREFIX + string("LHipPitch/Temperature") + SN_POSTFIX
#define L_KNEE_PITCH_TEMPERATURE      DN_PREFIX + string("LKneePitch/Temperature") + SN_POSTFIX
#define L_ANKLE_PITCH_TEMPERATURE     DN_PREFIX + string("LAnklePitch/Temperature") + SN_POSTFIX
#define L_ANKLE_ROLL_TEMPERATURE      DN_PREFIX + string("LAnkleRoll/Temperature") + SN_POSTFIX
#define R_HIP_YAWPITCH_TEMPERATURE    DN_PREFIX + string("LHipYawPitch/Temperature") + SN_POSTFIX
#define R_HIP_ROLL_TEMPERATURE        DN_PREFIX + string("RHipRoll/Temperature") + SN_POSTFIX
#define R_HIP_PITCH_TEMPERATURE       DN_PREFIX + string("RHipPitch/Temperature") + SN_POSTFIX
#define R_KNEE_PITCH_TEMPERATURE      DN_PREFIX + string("RKneePitch/Temperature") + SN_POSTFIX
#define R_ANKLE_PITCH_TEMPERATURE     DN_PREFIX + string("RAnklePitch/Temperature") + SN_POSTFIX
#define R_ANKLE_ROLL_TEMPERATURE      DN_PREFIX + string("RAnkleRoll/Temperature") + SN_POSTFIX

// Balance
#define ACCEL_X                  		DN_PREFIX + string("InertialSensor/AccX") + SN_POSTFIX
#define ACCEL_Y                  		DN_PREFIX + string("InertialSensor/AccY") + SN_POSTFIX
#define ACCEL_Z                  		DN_PREFIX + string("InertialSensor/AccZ") + SN_POSTFIX
#define ANGLE_X                  		DN_PREFIX + string("InertialSensor/AngleX") + SN_POSTFIX
#define ANGLE_Y                  		DN_PREFIX + string("InertialSensor/AngleY") + SN_POSTFIX
#define GYRO_X                   		DN_PREFIX + string("InertialSensor/GyrX") + SN_POSTFIX
#define GYRO_Y                   		DN_PREFIX + string("InertialSensor/GyrY") + SN_POSTFIX

// Touch
#define L_FSR_FL                 		DN_PREFIX + string("LFoot/FSR/FrontLeft") + SN_POSTFIX
#define L_FSR_FR                 		DN_PREFIX + string("LFoot/FSR/FrontRight") + SN_POSTFIX
#define L_FSR_BL                 		DN_PREFIX + string("LFoot/FSR/RearLeft") + SN_POSTFIX
#define L_FSR_BR                 		DN_PREFIX + string("LFoot/FSR/RearRight") + SN_POSTFIX
#define L_BUMP_L                 		DN_PREFIX + string("LFoot/Bumper/Left") + SN_POSTFIX
#define L_BUMP_R                 		DN_PREFIX + string("LFoot/Bumper/Right") + SN_POSTFIX
#define R_FSR_FL                 		DN_PREFIX + string("RFoot/FSR/FrontLeft") + SN_POSTFIX
#define R_FSR_FR                 		DN_PREFIX + string("RFoot/FSR/FrontRight") + SN_POSTFIX
#define R_FSR_BL                 		DN_PREFIX + string("RFoot/FSR/RearLeft") + SN_POSTFIX
#define R_FSR_BR                 		DN_PREFIX + string("RFoot/FSR/RearRight") + SN_POSTFIX
#define R_BUMP_L                 		DN_PREFIX + string("RFoot/Bumper/Left") + SN_POSTFIX
#define R_BUMP_R                 		DN_PREFIX + string("RFoot/Bumper/Right") + SN_POSTFIX
#define CHEST_BUTTON             		DN_PREFIX + string("ChestBoard/Button") + SN_POSTFIX

// Battery
#define CHARGE                   		DN_PREFIX + string("Battery/Charge") + SN_POSTFIX
#define CURRENT                  		DN_PREFIX + string("Battery/Current") + SN_POSTFIX
#define VOLTAGE_MIN              		DN_PREFIX + string("Battery/Charge/Sensor/CellVoltageMin")
#define VOLTAGE_MAX              		DN_PREFIX + string("Battery/Charge/Sensor/CellVoltageMax")
#define TEMPERATURE              		DN_PREFIX + string("Battery/Temperature") + SN_POSTFIX

// Distance (ultrasonic and infrared sensors)
#define US_DISTANCE_LEFT_VALUE0         DN_PREFIX + string("US/Left") + SN_POSTFIX
#define US_DISTANCE_LEFT_VALUE1         DN_PREFIX + string("US/Left") + SN_POSTFIX + "1"
#define US_DISTANCE_LEFT_VALUE2         DN_PREFIX + string("US/Left") + SN_POSTFIX + "2"
#define US_DISTANCE_LEFT_VALUE3         DN_PREFIX + string("US/Left") + SN_POSTFIX + "3"
#define US_DISTANCE_LEFT_VALUE4         DN_PREFIX + string("US/Left") + SN_POSTFIX + "4"
#define US_DISTANCE_LEFT_VALUE5         DN_PREFIX + string("US/Left") + SN_POSTFIX + "5"
#define US_DISTANCE_LEFT_VALUE6         DN_PREFIX + string("US/Left") + SN_POSTFIX + "6"
#define US_DISTANCE_LEFT_VALUE7         DN_PREFIX + string("US/Left") + SN_POSTFIX + "7"
#define US_DISTANCE_LEFT_VALUE8         DN_PREFIX + string("US/Left") + SN_POSTFIX + "8"
#define US_DISTANCE_LEFT_VALUE9         DN_PREFIX + string("US/Left") + SN_POSTFIX + "9"

#define US_DISTANCE_RIGHT_VALUE0        DN_PREFIX + string("US/Right") + SN_POSTFIX
#define US_DISTANCE_RIGHT_VALUE1        DN_PREFIX + string("US/Right") + SN_POSTFIX + "1"
#define US_DISTANCE_RIGHT_VALUE2        DN_PREFIX + string("US/Right") + SN_POSTFIX + "2"
#define US_DISTANCE_RIGHT_VALUE3        DN_PREFIX + string("US/Right") + SN_POSTFIX + "3"
#define US_DISTANCE_RIGHT_VALUE4        DN_PREFIX + string("US/Right") + SN_POSTFIX + "4"
#define US_DISTANCE_RIGHT_VALUE5        DN_PREFIX + string("US/Right") + SN_POSTFIX + "5"
#define US_DISTANCE_RIGHT_VALUE6        DN_PREFIX + string("US/Right") + SN_POSTFIX + "6"
#define US_DISTANCE_RIGHT_VALUE7        DN_PREFIX + string("US/Right") + SN_POSTFIX + "7"
#define US_DISTANCE_RIGHT_VALUE8        DN_PREFIX + string("US/Right") + SN_POSTFIX + "8"
#define US_DISTANCE_RIGHT_VALUE9        DN_PREFIX + string("US/Right") + SN_POSTFIX + "9"


#endif

