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
#define DN_POSTFIX                 string("/Sensor/Value")

// Position
#define HEAD_YAW_POSITION          string("HeadYaw/Position")
#define HEAD_PITCH_POSITION        string("HeadPitch/Position")
#define L_SHOULDER_ROLL_POSITION   string("LShoulderRoll/Position")
#define L_SHOULDER_PITCH_POSITION  string("LShoulderPitch/Position")
#define L_ELBOW_YAW_POSITION       string("LElbowYaw/Position") 
#define L_ELBOW_ROLL_POSITION      string("LElbowRoll/Position")
#define R_SHOULDER_ROLL_POSITION   string("RShoulderRoll/Position")
#define R_SHOULDER_PITCH_POSITION  string("RShoulderPitch/Position")
#define R_ELBOW_YAW_POSITION       string("RElbowYaw/Position") 
#define R_ELBOW_ROLL_POSITION      string("RElbowRoll/Position")
#define L_HIP_YAWPITCH_POSITION    string("LHipYawPitch/Position") 
#define L_HIP_ROLL_POSITION        string("LHipRoll/Position") 
#define L_HIP_PITCH_POSITION       string("LHipPitch/Position")
#define L_KNEE_PITCH_POSITION      string("LKneePitch/Position")
#define L_ANKLE_PITCH_POSITION     string("LAnklePitch/Position") 
#define L_ANKLE_ROLL_POSITION      string("LAnkleRoll/Position")
#define R_HIP_YAWPITCH_POSITION    string("RHipYawPitch/Position")
#define R_HIP_ROLL_POSITION        string("RHipRoll/Position") 
#define R_HIP_PITCH_POSITION       string("RHipPitch/Position")
#define R_KNEE_PITCH_POSITION      string("RKneePitch/Position")
#define R_ANKLE_PITCH_POSITION     string("RAnklePitch/Position") 
#define R_ANKLE_ROLL_POSITION      string("RAnkleRoll/Position")

// Current
#define HEAD_YAW_CURRENT          string("HeadYaw/ElectricCurrent")
#define HEAD_PITCH_CURRENT        string("HeadPitch/ElectricCurrent")
#define L_SHOULDER_ROLL_CURRENT   string("LShoulderRoll/ElectricCurrent")
#define L_SHOULDER_PITCH_CURRENT  string("LShoulderPitch/ElectricCurrent")
#define L_ELBOW_YAW_CURRENT       string("LElbowYaw/ElectricCurrent") 
#define L_ELBOW_ROLL_CURRENT      string("LElbowRoll/ElectricCurrent")
#define R_SHOULDER_ROLL_CURRENT   string("RShoulderRoll/ElectricCurrent")
#define R_SHOULDER_PITCH_CURRENT  string("RShoulderPitch/ElectricCurrent")
#define R_ELBOW_YAW_CURRENT       string("RElbowYaw/ElectricCurrent") 
#define R_ELBOW_ROLL_CURRENT      string("RElbowRoll/ElectricCurrent")
#define L_HIP_YAWPITCH_CURRENT    string("LHipYawPitch/ElectricCurrent") 
#define L_HIP_ROLL_CURRENT        string("LHipRoll/ElectricCurrent") 
#define L_HIP_PITCH_CURRENT       string("LHipPitch/ElectricCurrent")
#define L_KNEE_PITCH_CURRENT      string("LKneePitch/ElectricCurrent")
#define L_ANKLE_PITCH_CURRENT     string("LAnklePitch/ElectricCurrent") 
#define L_ANKLE_ROLL_CURRENT      string("LAnkleRoll/ElectricCurrent")
#define R_HIP_YAWPITCH_CURRENT    string("LHipYawPitch/ElectricCurrent")
#define R_HIP_ROLL_CURRENT        string("RHipRoll/ElectricCurrent") 
#define R_HIP_PITCH_CURRENT       string("RHipPitch/ElectricCurrent")
#define R_KNEE_PITCH_CURRENT      string("RKneePitch/ElectricCurrent")
#define R_ANKLE_PITCH_CURRENT     string("RAnklePitch/ElectricCurrent") 
#define R_ANKLE_ROLL_CURRENT      string("RAnkleRoll/ElectricCurrent")

// Actuator Target
#define HEAD_YAW_TARGET          string("HeadYaw/Position/Actuator/Value")
#define HEAD_PITCH_TARGET        string("HeadPitch/Position/Actuator/Value")
#define L_SHOULDER_ROLL_TARGET   string("LShoulderRoll/Position/Actuator/Value")
#define L_SHOULDER_PITCH_TARGET  string("LShoulderPitch/Position/Actuator/Value")
#define L_ELBOW_YAW_TARGET       string("LElbowYaw/Position/Actuator/Value") 
#define L_ELBOW_ROLL_TARGET      string("LElbowRoll/Position/Actuator/Value")
#define R_SHOULDER_ROLL_TARGET   string("RShoulderRoll/Position/Actuator/Value")
#define R_SHOULDER_PITCH_TARGET  string("RShoulderPitch/Position/Actuator/Value")
#define R_ELBOW_YAW_TARGET       string("RElbowYaw/Position/Actuator/Value") 
#define R_ELBOW_ROLL_TARGET      string("RElbowRoll/Position/Actuator/Value")
#define L_HIP_YAWPITCH_TARGET    string("LHipYawPitch/Position/Actuator/Value") 
#define L_HIP_ROLL_TARGET        string("LHipRoll/Position/Actuator/Value") 
#define L_HIP_PITCH_TARGET       string("LHipPitch/Position/Actuator/Value")
#define L_KNEE_PITCH_TARGET      string("LKneePitch/Position/Actuator/Value")
#define L_ANKLE_PITCH_TARGET     string("LAnklePitch/Position/Actuator/Value") 
#define L_ANKLE_ROLL_TARGET      string("LAnkleRoll/Position/Actuator/Value")
#define R_HIP_YAWPITCH_TARGET    string("LHipYawPitch/Position/Actuator/Value")
#define R_HIP_ROLL_TARGET        string("RHipRoll/Position/Actuator/Value") 
#define R_HIP_PITCH_TARGET       string("RHipPitch/Position/Actuator/Value")
#define R_KNEE_PITCH_TARGET      string("RKneePitch/Position/Actuator/Value")
#define R_ANKLE_PITCH_TARGET     string("RAnklePitch/Position/Actuator/Value") 
#define R_ANKLE_ROLL_TARGET      string("RAnkleRoll/Position/Actuator/Value")

// Actuator Target
#define HEAD_YAW_HARDNESS          string("HeadYaw/Hardness/Actuator/Value")
#define HEAD_PITCH_HARDNESS        string("HeadPitch/Hardness/Actuator/Value")
#define L_SHOULDER_ROLL_HARDNESS   string("LShoulderRoll/Hardness/Actuator/Value")
#define L_SHOULDER_PITCH_HARDNESS  string("LShoulderPitch/Hardness/Actuator/Value")
#define L_ELBOW_YAW_HARDNESS       string("LElbowYaw/Hardness/Actuator/Value") 
#define L_ELBOW_ROLL_HARDNESS      string("LElbowRoll/Hardness/Actuator/Value")
#define R_SHOULDER_ROLL_HARDNESS   string("RShoulderRoll/Hardness/Actuator/Value")
#define R_SHOULDER_PITCH_HARDNESS  string("RShoulderPitch/Hardness/Actuator/Value")
#define R_ELBOW_YAW_HARDNESS       string("RElbowYaw/Hardness/Actuator/Value") 
#define R_ELBOW_ROLL_HARDNESS      string("RElbowRoll/Hardness/Actuator/Value")
#define L_HIP_YAWPITCH_HARDNESS    string("LHipYawPitch/Hardness/Actuator/Value") 
#define L_HIP_ROLL_HARDNESS        string("LHipRoll/Hardness/Actuator/Value") 
#define L_HIP_PITCH_HARDNESS       string("LHipPitch/Hardness/Actuator/Value")
#define L_KNEE_PITCH_HARDNESS      string("LKneePitch/Hardness/Actuator/Value")
#define L_ANKLE_PITCH_HARDNESS     string("LAnklePitch/Hardness/Actuator/Value") 
#define L_ANKLE_ROLL_HARDNESS      string("LAnkleRoll/Hardness/Actuator/Value")
#define R_HIP_YAWPITCH_HARDNESS    string("LHipYawPitch/Hardness/Actuator/Value")
#define R_HIP_ROLL_HARDNESS        string("RHipRoll/Hardness/Actuator/Value") 
#define R_HIP_PITCH_HARDNESS       string("RHipPitch/Hardness/Actuator/Value")
#define R_KNEE_PITCH_HARDNESS      string("RKneePitch/Hardness/Actuator/Value")
#define R_ANKLE_PITCH_HARDNESS     string("RAnklePitch/Hardness/Actuator/Value") 
#define R_ANKLE_ROLL_HARDNESS      string("RAnkleRoll/Hardness/Actuator/Value")

// Temperature
#define HEAD_YAW_TEMPERATURE          string("HeadYaw/Temperature")
#define HEAD_PITCH_TEMPERATURE        string("HeadPitch/Temperature")
#define L_SHOULDER_ROLL_TEMPERATURE   string("LShoulderRoll/Temperature")
#define L_SHOULDER_PITCH_TEMPERATURE  string("LShoulderPitch/Temperature")
#define L_ELBOW_YAW_TEMPERATURE       string("LElbowYaw/Temperature") 
#define L_ELBOW_ROLL_TEMPERATURE      string("LElbowRoll/Temperature")
#define R_SHOULDER_ROLL_TEMPERATURE   string("RShoulderRoll/Temperature")
#define R_SHOULDER_PITCH_TEMPERATURE  string("RShoulderPitch/Temperature")
#define R_ELBOW_YAW_TEMPERATURE       string("RElbowYaw/Temperature") 
#define R_ELBOW_ROLL_TEMPERATURE      string("RElbowRoll/Temperature")
#define L_HIP_YAWPITCH_TEMPERATURE    string("LHipYawPitch/Temperature") 
#define L_HIP_ROLL_TEMPERATURE        string("LHipRoll/Temperature") 
#define L_HIP_PITCH_TEMPERATURE       string("LHipPitch/Temperature")
#define L_KNEE_PITCH_TEMPERATURE      string("LKneePitch/Temperature")
#define L_ANKLE_PITCH_TEMPERATURE     string("LAnklePitch/Temperature") 
#define L_ANKLE_ROLL_TEMPERATURE      string("LAnkleRoll/Temperature")
#define R_HIP_YAWPITCH_TEMPERATURE    string("LHipYawPitch/Temperature")
#define R_HIP_ROLL_TEMPERATURE        string("RHipRoll/Temperature") 
#define R_HIP_PITCH_TEMPERATURE       string("RHipPitch/Temperature")
#define R_KNEE_PITCH_TEMPERATURE      string("RKneePitch/Temperature")
#define R_ANKLE_PITCH_TEMPERATURE     string("RAnklePitch/Temperature") 
#define R_ANKLE_ROLL_TEMPERATURE      string("RAnkleRoll/Temperature")

// Balance
#define ACCEL_X                  string("InertialSensor/AccX")
#define ACCEL_Y                  string("InertialSensor/AccY") 
#define ACCEL_Z                  string("InertialSensor/AccZ")
#define ANGLE_X                  string("InertialSensor/AngleX")
#define ANGLE_Y                  string("InertialSensor/AngleY")
#define GYRO_X                   string("InertialSensor/GyrX")
#define GYRO_Y                   string("InertialSensor/GyrY") 

// Touch
#define L_FSR_FL                 string("LFoot/FSR/FrontLeft")
#define L_FSR_FR                 string("LFoot/FSR/FrontRight")
#define L_FSR_BL                 string("LFoot/FSR/RearLeft")
#define L_FSR_BR                 string("LFoot/FSR/RearRight")
#define L_BUMP_L                 string("LFoot/Bumper/Left")
#define L_BUMP_R                 string("LFoot/Bumper/Right")
#define R_FSR_FL                 string("RFoot/FSR/FrontLeft")
#define R_FSR_FR                 string("RFoot/FSR/FrontRight")
#define R_FSR_BL                 string("RFoot/FSR/RearLeft")
#define R_FSR_BR                 string("RFoot/FSR/RearRight")
#define R_BUMP_L                 string("RFoot/Bumper/Left")
#define R_BUMP_R                 string("RFoot/Bumper/Right")
#define CHEST_BUTTON             string("ChestBoard/Button")
#define SIMPLE_CLICK             string("ALWatchDog/SimpleClickOccured")
#define DOUBLE_CLICK             string("ALWatchDog/DoubleClickOccured")
#define TRIPLE_CLICK             string("ALWatchDog/TripleClickOccured")

// Battery
#define CHARGE                   string("Battery/Charge")
#define CURRENT                  string("Battery/Current")
#define VOLTAGE_MIN              string("Battery/Charge/Sensor/CellVoltageMin")
#define VOLTAGE_MAX              string("Battery/Charge/Sensor/CellVoltageMax")
#define TEMPERATURE              string("Battery/Temperature")

// Distance (ultrasonic and infrared sensors)
#define US_DISTANCE_LEFT_VALUE0              string("US/Left")
#define US_DISTANCE_LEFT_VALUE1              string("US/Left1")
#define US_DISTANCE_LEFT_VALUE2              string("US/Left2")
#define US_DISTANCE_LEFT_VALUE3              string("US/Left3")
#define US_DISTANCE_LEFT_VALUE4              string("US/Left4")
#define US_DISTANCE_LEFT_VALUE5              string("US/Left5")
#define US_DISTANCE_LEFT_VALUE6              string("US/Left6")
#define US_DISTANCE_LEFT_VALUE7              string("US/Left7")
#define US_DISTANCE_LEFT_VALUE8              string("US/Left8")
#define US_DISTANCE_LEFT_VALUE9              string("US/Left9")

#define US_DISTANCE_RIGHT_VALUE0              string("US/Right")
#define US_DISTANCE_RIGHT_VALUE1              string("US/Right1")
#define US_DISTANCE_RIGHT_VALUE2              string("US/Right2")
#define US_DISTANCE_RIGHT_VALUE3              string("US/Right3")
#define US_DISTANCE_RIGHT_VALUE4              string("US/Right4")
#define US_DISTANCE_RIGHT_VALUE5              string("US/Right5")
#define US_DISTANCE_RIGHT_VALUE6              string("US/Right6")
#define US_DISTANCE_RIGHT_VALUE7              string("US/Right7")
#define US_DISTANCE_RIGHT_VALUE8              string("US/Right8")
#define US_DISTANCE_RIGHT_VALUE9              string("US/Right9")


#endif
