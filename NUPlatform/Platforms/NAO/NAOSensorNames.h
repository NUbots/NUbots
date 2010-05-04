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
// Position
#define DN_HEAD_YAW_POSITION          std::string("Device/SubDeviceList/HeadYaw/Position/Sensor/Value")
#define DN_HEAD_PITCH_POSITION        std::string("Device/SubDeviceList/HeadPitch/Position/Sensor/Value")
#define DN_L_SHOULDER_ROLL_POSITION   std::string("Device/SubDeviceList/LShoulderRoll/Position/Sensor/Value")
#define DN_L_SHOULDER_PITCH_POSITION  std::string("Device/SubDeviceList/LShoulderPitch/Position/Sensor/Value")
#define DN_L_ELBOW_YAW_POSITION       std::string("Device/SubDeviceList/LElbowYaw/Position/Sensor/Value") 
#define DN_L_ELBOW_ROLL_POSITION      std::string("Device/SubDeviceList/LElbowRoll/Position/Sensor/Value")
#define DN_R_SHOULDER_ROLL_POSITION   std::string("Device/SubDeviceList/RShoulderRoll/Position/Sensor/Value")
#define DN_R_SHOULDER_PITCH_POSITION  std::string("Device/SubDeviceList/RShoulderPitch/Position/Sensor/Value")
#define DN_R_ELBOW_YAW_POSITION       std::string("Device/SubDeviceList/RElbowYaw/Position/Sensor/Value") 
#define DN_R_ELBOW_ROLL_POSITION      std::string("Device/SubDeviceList/RElbowRoll/Position/Sensor/Value")
#define DN_L_HIP_YAWPITCH_POSITION    std::string("Device/SubDeviceList/LHipYawPitch/Position/Sensor/Value") 
#define DN_L_HIP_ROLL_POSITION        std::string("Device/SubDeviceList/LHipRoll/Position/Sensor/Value") 
#define DN_L_HIP_PITCH_POSITION       std::string("Device/SubDeviceList/LHipPitch/Position/Sensor/Value")
#define DN_L_KNEE_PITCH_POSITION      std::string("Device/SubDeviceList/LKneePitch/Position/Sensor/Value")
#define DN_L_ANKLE_PITCH_POSITION     std::string("Device/SubDeviceList/LAnklePitch/Position/Sensor/Value") 
#define DN_L_ANKLE_ROLL_POSITION      std::string("Device/SubDeviceList/LAnkleRoll/Position/Sensor/Value")
#define DN_R_HIP_YAWPITCH_POSITION    std::string("Device/SubDeviceList/RHipYawPitch/Position/Sensor/Value")
#define DN_R_HIP_ROLL_POSITION        std::string("Device/SubDeviceList/RHipRoll/Position/Sensor/Value") 
#define DN_R_HIP_PITCH_POSITION       std::string("Device/SubDeviceList/RHipPitch/Position/Sensor/Value")
#define DN_R_KNEE_PITCH_POSITION      std::string("Device/SubDeviceList/RKneePitch/Position/Sensor/Value")
#define DN_R_ANKLE_PITCH_POSITION     std::string("Device/SubDeviceList/RAnklePitch/Position/Sensor/Value") 
#define DN_R_ANKLE_ROLL_POSITION      std::string("Device/SubDeviceList/RAnkleRoll/Position/Sensor/Value")

// Current
#define DN_HEAD_YAW_CURRENT          std::string("Device/SubDeviceList/HeadYaw/ElectricCurrent/Sensor/Value")
#define DN_HEAD_PITCH_CURRENT        std::string("Device/SubDeviceList/HeadPitch/ElectricCurrent/Sensor/Value")
#define DN_L_SHOULDER_ROLL_CURRENT   std::string("Device/SubDeviceList/LShoulderRoll/ElectricCurrent/Sensor/Value")
#define DN_L_SHOULDER_PITCH_CURRENT  std::string("Device/SubDeviceList/LShoulderPitch/ElectricCurrent/Sensor/Value")
#define DN_L_ELBOW_YAW_CURRENT       std::string("Device/SubDeviceList/LElbowYaw/ElectricCurrent/Sensor/Value") 
#define DN_L_ELBOW_ROLL_CURRENT      std::string("Device/SubDeviceList/LElbowRoll/ElectricCurrent/Sensor/Value")
#define DN_R_SHOULDER_ROLL_CURRENT   std::string("Device/SubDeviceList/RShoulderRoll/ElectricCurrent/Sensor/Value")
#define DN_R_SHOULDER_PITCH_CURRENT  std::string("Device/SubDeviceList/RShoulderPitch/ElectricCurrent/Sensor/Value")
#define DN_R_ELBOW_YAW_CURRENT       std::string("Device/SubDeviceList/RElbowYaw/ElectricCurrent/Sensor/Value") 
#define DN_R_ELBOW_ROLL_CURRENT      std::string("Device/SubDeviceList/RElbowRoll/ElectricCurrent/Sensor/Value")
#define DN_L_HIP_YAWPITCH_CURRENT    std::string("Device/SubDeviceList/LHipYawPitch/ElectricCurrent/Sensor/Value") 
#define DN_L_HIP_ROLL_CURRENT        std::string("Device/SubDeviceList/LHipRoll/ElectricCurrent/Sensor/Value") 
#define DN_L_HIP_PITCH_CURRENT       std::string("Device/SubDeviceList/LHipPitch/ElectricCurrent/Sensor/Value")
#define DN_L_KNEE_PITCH_CURRENT      std::string("Device/SubDeviceList/LKneePitch/ElectricCurrent/Sensor/Value")
#define DN_L_ANKLE_PITCH_CURRENT     std::string("Device/SubDeviceList/LAnklePitch/ElectricCurrent/Sensor/Value") 
#define DN_L_ANKLE_ROLL_CURRENT      std::string("Device/SubDeviceList/LAnkleRoll/ElectricCurrent/Sensor/Value")
#define DN_R_HIP_YAWPITCH_CURRENT    std::string("Device/SubDeviceList/LHipYawPitch/ElectricCurrent/Sensor/Value")
#define DN_R_HIP_ROLL_CURRENT        std::string("Device/SubDeviceList/RHipRoll/ElectricCurrent/Sensor/Value") 
#define DN_R_HIP_PITCH_CURRENT       std::string("Device/SubDeviceList/RHipPitch/ElectricCurrent/Sensor/Value")
#define DN_R_KNEE_PITCH_CURRENT      std::string("Device/SubDeviceList/RKneePitch/ElectricCurrent/Sensor/Value")
#define DN_R_ANKLE_PITCH_CURRENT     std::string("Device/SubDeviceList/RAnklePitch/ElectricCurrent/Sensor/Value") 
#define DN_R_ANKLE_ROLL_CURRENT      std::string("Device/SubDeviceList/RAnkleRoll/ElectricCurrent/Sensor/Value")

// Actuator Target
#define DN_HEAD_YAW_TARGET          std::string("Device/SubDeviceList/HeadYaw/Position/Actuator/Value")
#define DN_HEAD_PITCH_TARGET        std::string("Device/SubDeviceList/HeadPitch/Position/Actuator/Value")
#define DN_L_SHOULDER_ROLL_TARGET   std::string("Device/SubDeviceList/LShoulderRoll/Position/Actuator/Value")
#define DN_L_SHOULDER_PITCH_TARGET  std::string("Device/SubDeviceList/LShoulderPitch/Position/Actuator/Value")
#define DN_L_ELBOW_YAW_TARGET       std::string("Device/SubDeviceList/LElbowYaw/Position/Actuator/Value") 
#define DN_L_ELBOW_ROLL_TARGET      std::string("Device/SubDeviceList/LElbowRoll/Position/Actuator/Value")
#define DN_R_SHOULDER_ROLL_TARGET   std::string("Device/SubDeviceList/RShoulderRoll/Position/Actuator/Value")
#define DN_R_SHOULDER_PITCH_TARGET  std::string("Device/SubDeviceList/RShoulderPitch/Position/Actuator/Value")
#define DN_R_ELBOW_YAW_TARGET       std::string("Device/SubDeviceList/RElbowYaw/Position/Actuator/Value") 
#define DN_R_ELBOW_ROLL_TARGET      std::string("Device/SubDeviceList/RElbowRoll/Position/Actuator/Value")
#define DN_L_HIP_YAWPITCH_TARGET    std::string("Device/SubDeviceList/LHipYawPitch/Position/Actuator/Value") 
#define DN_L_HIP_ROLL_TARGET        std::string("Device/SubDeviceList/LHipRoll/Position/Actuator/Value") 
#define DN_L_HIP_PITCH_TARGET       std::string("Device/SubDeviceList/LHipPitch/Position/Actuator/Value")
#define DN_L_KNEE_PITCH_TARGET      std::string("Device/SubDeviceList/LKneePitch/Position/Actuator/Value")
#define DN_L_ANKLE_PITCH_TARGET     std::string("Device/SubDeviceList/LAnklePitch/Position/Actuator/Value") 
#define DN_L_ANKLE_ROLL_TARGET      std::string("Device/SubDeviceList/LAnkleRoll/Position/Actuator/Value")
#define DN_R_HIP_YAWPITCH_TARGET    std::string("Device/SubDeviceList/LHipYawPitch/Position/Actuator/Value")
#define DN_R_HIP_ROLL_TARGET        std::string("Device/SubDeviceList/RHipRoll/Position/Actuator/Value") 
#define DN_R_HIP_PITCH_TARGET       std::string("Device/SubDeviceList/RHipPitch/Position/Actuator/Value")
#define DN_R_KNEE_PITCH_TARGET      std::string("Device/SubDeviceList/RKneePitch/Position/Actuator/Value")
#define DN_R_ANKLE_PITCH_TARGET     std::string("Device/SubDeviceList/RAnklePitch/Position/Actuator/Value") 
#define DN_R_ANKLE_ROLL_TARGET      std::string("Device/SubDeviceList/RAnkleRoll/Position/Actuator/Value")

// Actuator Target
#define DN_HEAD_YAW_HARDNESS          std::string("Device/SubDeviceList/HeadYaw/Hardness/Actuator/Value")
#define DN_HEAD_PITCH_HARDNESS        std::string("Device/SubDeviceList/HeadPitch/Hardness/Actuator/Value")
#define DN_L_SHOULDER_ROLL_HARDNESS   std::string("Device/SubDeviceList/LShoulderRoll/Hardness/Actuator/Value")
#define DN_L_SHOULDER_PITCH_HARDNESS  std::string("Device/SubDeviceList/LShoulderPitch/Hardness/Actuator/Value")
#define DN_L_ELBOW_YAW_HARDNESS       std::string("Device/SubDeviceList/LElbowYaw/Hardness/Actuator/Value") 
#define DN_L_ELBOW_ROLL_HARDNESS      std::string("Device/SubDeviceList/LElbowRoll/Hardness/Actuator/Value")
#define DN_R_SHOULDER_ROLL_HARDNESS   std::string("Device/SubDeviceList/RShoulderRoll/Hardness/Actuator/Value")
#define DN_R_SHOULDER_PITCH_HARDNESS  std::string("Device/SubDeviceList/RShoulderPitch/Hardness/Actuator/Value")
#define DN_R_ELBOW_YAW_HARDNESS       std::string("Device/SubDeviceList/RElbowYaw/Hardness/Actuator/Value") 
#define DN_R_ELBOW_ROLL_HARDNESS      std::string("Device/SubDeviceList/RElbowRoll/Hardness/Actuator/Value")
#define DN_L_HIP_YAWPITCH_HARDNESS    std::string("Device/SubDeviceList/LHipYawPitch/Hardness/Actuator/Value") 
#define DN_L_HIP_ROLL_HARDNESS        std::string("Device/SubDeviceList/LHipRoll/Hardness/Actuator/Value") 
#define DN_L_HIP_PITCH_HARDNESS       std::string("Device/SubDeviceList/LHipPitch/Hardness/Actuator/Value")
#define DN_L_KNEE_PITCH_HARDNESS      std::string("Device/SubDeviceList/LKneePitch/Hardness/Actuator/Value")
#define DN_L_ANKLE_PITCH_HARDNESS     std::string("Device/SubDeviceList/LAnklePitch/Hardness/Actuator/Value") 
#define DN_L_ANKLE_ROLL_HARDNESS      std::string("Device/SubDeviceList/LAnkleRoll/Hardness/Actuator/Value")
#define DN_R_HIP_YAWPITCH_HARDNESS    std::string("Device/SubDeviceList/LHipYawPitch/Hardness/Actuator/Value")
#define DN_R_HIP_ROLL_HARDNESS        std::string("Device/SubDeviceList/RHipRoll/Hardness/Actuator/Value") 
#define DN_R_HIP_PITCH_HARDNESS       std::string("Device/SubDeviceList/RHipPitch/Hardness/Actuator/Value")
#define DN_R_KNEE_PITCH_HARDNESS      std::string("Device/SubDeviceList/RKneePitch/Hardness/Actuator/Value")
#define DN_R_ANKLE_PITCH_HARDNESS     std::string("Device/SubDeviceList/RAnklePitch/Hardness/Actuator/Value") 
#define DN_R_ANKLE_ROLL_HARDNESS      std::string("Device/SubDeviceList/RAnkleRoll/Hardness/Actuator/Value")

// Temperature
#define DN_HEAD_YAW_TEMPERATURE          std::string("Device/SubDeviceList/HeadYaw/Temperature/Sensor/Value")
#define DN_HEAD_PITCH_TEMPERATURE        std::string("Device/SubDeviceList/HeadPitch/Temperature/Sensor/Value")
#define DN_L_SHOULDER_ROLL_TEMPERATURE   std::string("Device/SubDeviceList/LShoulderRoll/Temperature/Sensor/Value")
#define DN_L_SHOULDER_PITCH_TEMPERATURE  std::string("Device/SubDeviceList/LShoulderPitch/Temperature/Sensor/Value")
#define DN_L_ELBOW_YAW_TEMPERATURE       std::string("Device/SubDeviceList/LElbowYaw/Temperature/Sensor/Value") 
#define DN_L_ELBOW_ROLL_TEMPERATURE      std::string("Device/SubDeviceList/LElbowRoll/Temperature/Sensor/Value")
#define DN_R_SHOULDER_ROLL_TEMPERATURE   std::string("Device/SubDeviceList/RShoulderRoll/Temperature/Sensor/Value")
#define DN_R_SHOULDER_PITCH_TEMPERATURE  std::string("Device/SubDeviceList/RShoulderPitch/Temperature/Sensor/Value")
#define DN_R_ELBOW_YAW_TEMPERATURE       std::string("Device/SubDeviceList/RElbowYaw/Temperature/Sensor/Value") 
#define DN_R_ELBOW_ROLL_TEMPERATURE      std::string("Device/SubDeviceList/RElbowRoll/Temperature/Sensor/Value")
#define DN_L_HIP_YAWPITCH_TEMPERATURE    std::string("Device/SubDeviceList/LHipYawPitch/Temperature/Sensor/Value") 
#define DN_L_HIP_ROLL_TEMPERATURE        std::string("Device/SubDeviceList/LHipRoll/Temperature/Sensor/Value") 
#define DN_L_HIP_PITCH_TEMPERATURE       std::string("Device/SubDeviceList/LHipPitch/Temperature/Sensor/Value")
#define DN_L_KNEE_PITCH_TEMPERATURE      std::string("Device/SubDeviceList/LKneePitch/Temperature/Sensor/Value")
#define DN_L_ANKLE_PITCH_TEMPERATURE     std::string("Device/SubDeviceList/LAnklePitch/Temperature/Sensor/Value") 
#define DN_L_ANKLE_ROLL_TEMPERATURE      std::string("Device/SubDeviceList/LAnkleRoll/Temperature/Sensor/Value")
#define DN_R_HIP_YAWPITCH_TEMPERATURE    std::string("Device/SubDeviceList/LHipYawPitch/Temperature/Sensor/Value")
#define DN_R_HIP_ROLL_TEMPERATURE        std::string("Device/SubDeviceList/RHipRoll/Temperature/Sensor/Value") 
#define DN_R_HIP_PITCH_TEMPERATURE       std::string("Device/SubDeviceList/RHipPitch/Temperature/Sensor/Value")
#define DN_R_KNEE_PITCH_TEMPERATURE      std::string("Device/SubDeviceList/RKneePitch/Temperature/Sensor/Value")
#define DN_R_ANKLE_PITCH_TEMPERATURE     std::string("Device/SubDeviceList/RAnklePitch/Temperature/Sensor/Value") 
#define DN_R_ANKLE_ROLL_TEMPERATURE      std::string("Device/SubDeviceList/RAnkleRoll/Temperature/Sensor/Value")

// Balance
#define DN_ACCEL_X                  std::string("Device/SubDeviceList/InertialSensor/AccX/Sensor/Value")
#define DN_ACCEL_Y                  std::string("Device/SubDeviceList/InertialSensor/AccY/Sensor/Value") 
#define DN_ACCEL_Z                  std::string("Device/SubDeviceList/InertialSensor/AccZ/Sensor/Value")
#define DN_ANGLE_X                  std::string("Device/SubDeviceList/InertialSensor/AngleX/Sensor/Value")
#define DN_ANGLE_Y                  std::string("Device/SubDeviceList/InertialSensor/AngleY/Sensor/Value")
#define DN_GYRO_X                   std::string("Device/SubDeviceList/InertialSensor/GyrX/Sensor/Value")
#define DN_GYRO_Y                   std::string("Device/SubDeviceList/InertialSensor/GyrY/Sensor/Value") 

// Touch
#define DN_L_FSR_FL                 std::string("Device/SubDeviceList/LFoot/FSR/FrontLeft/Sensor/Value")
#define DN_L_FSR_FR                 std::string("Device/SubDeviceList/LFoot/FSR/FrontRight/Sensor/Value")
#define DN_L_FSR_BL                 std::string("Device/SubDeviceList/LFoot/FSR/RearLeft/Sensor/Value")
#define DN_L_FSR_BR                 std::string("Device/SubDeviceList/LFoot/FSR/RearRight/Sensor/Value")
#define DN_L_BUMP_L                 std::string("Device/SubDeviceList/LFoot/Bumper/Left/Sensor/Value")
#define DN_L_BUMP_R                 std::string("Device/SubDeviceList/LFoot/Bumper/Right/Sensor/Value")
#define DN_R_FSR_FL                 std::string("Device/SubDeviceList/RFoot/FSR/FrontLeft/Sensor/Value")
#define DN_R_FSR_FR                 std::string("Device/SubDeviceList/RFoot/FSR/FrontRight/Sensor/Value")
#define DN_R_FSR_BL                 std::string("Device/SubDeviceList/RFoot/FSR/RearLeft/Sensor/Value")
#define DN_R_FSR_BR                 std::string("Device/SubDeviceList/RFoot/FSR/RearRight/Sensor/Value")
#define DN_R_BUMP_L                 std::string("Device/SubDeviceList/RFoot/Bumper/Left/Sensor/Value")
#define DN_R_BUMP_R                 std::string("Device/SubDeviceList/RFoot/Bumper/Right/Sensor/Value")
#define DN_CHEST_BUTTON             std::string("Device/SubDeviceList/ChestBoard/Button/Sensor/Value")
#define DN_SIMPLE_CLICK             std::string("ALWatchDog/SimpleClickOccured")
#define DN_DOUBLE_CLICK             std::string("ALWatchDog/DoubleClickOccured")
#define DN_TRIPLE_CLICK             std::string("ALWatchDog/TripleClickOccured")

// Battery
#define DN_CHARGE                   std::string("Device/SubDeviceList/Battery/Charge/Sensor/Value")
#define DN_CURRENT                  std::string("Device/SubDeviceList/Battery/Current/Sensor/Value")
#define DN_VOLTAGE_MIN              std::string("Device/SubDeviceList/Battery/Charge/Sensor/CellVoltageMin")
#define DN_VOLTAGE_MAX              std::string("Device/SubDeviceList/Battery/Charge/Sensor/CellVoltageMax")
#define DN_TEMPERATURE              std::string("Device/SubDeviceList/Battery/Temperature/Sensor/Value")

// Distance (ultrasonic and infrared sensors)
#define DN_US_DISTANCE              std::string("Device/SubDeviceList/US/Sensor/Value")

#endif
