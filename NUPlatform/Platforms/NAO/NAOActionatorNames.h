/*! @file NAOActionatorNames.h
    @brief Definition of NAO actionator names (ie. Device names to be used with dcm)
 
    @author Jason Kulk
 
 Copyright (c) 2009, 2010 Jason Kulk
 
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

#ifndef NAOACTIONATORNAMES_H
#define NAOACTIONATORNAMES_H

#include <string>
using namespace std;

// Joint Names
#define HEAD_YAW             std::string("HeadYaw")
#define HEAD_PITCH           std::string("HeadPitch")
#define L_SHOULDER_ROLL      std::string("LShoulderRoll")
#define L_SHOULDER_PITCH     std::string("LShoulderPitch")
#define L_ELBOW_YAW          std::string("LElbowYaw") 
#define L_ELBOW_ROLL         std::string("LElbowRoll")
#define R_SHOULDER_ROLL      std::string("RShoulderRoll")
#define R_SHOULDER_PITCH     std::string("RShoulderPitch")
#define R_ELBOW_YAW          std::string("RElbowYaw") 
#define R_ELBOW_ROLL         std::string("RElbowRoll")
#define L_HIP_YAWPITCH       std::string("LHipYawPitch") 
#define L_HIP_ROLL           std::string("LHipRoll") 
#define L_HIP_PITCH          std::string("LHipPitch")
#define L_KNEE_PITCH         std::string("LKneePitch")
#define L_ANKLE_PITCH        std::string("LAnklePitch") 
#define L_ANKLE_ROLL         std::string("LAnkleRoll")
#define R_HIP_YAWPITCH       std::string("RHipYawPitch")
#define R_HIP_ROLL           std::string("RHipRoll") 
#define R_HIP_PITCH          std::string("RHipPitch")
#define R_KNEE_PITCH         std::string("RKneePitch")
#define R_ANKLE_PITCH        std::string("RAnklePitch") 
#define R_ANKLE_ROLL         std::string("RAnkleRoll")

// Device names for position actuators
#define DN_HEAD_YAW_POSITION            std::string("HeadYaw/Position/Actuator/Value")
#define DN_HEAD_PITCH_POSITION          std::string("HeadPitch/Position/Actuator/Value")
#define DN_L_SHOULDER_ROLL_POSITION     std::string("LShoulderRoll/Position/Actuator/Value")
#define DN_L_SHOULDER_PITCH_POSITION    std::string("LShoulderPitch/Position/Actuator/Value")
#define DN_L_ELBOW_YAW_POSITION         std::string("LElbowYaw/Position/Actuator/Value") 
#define DN_L_ELBOW_ROLL_POSITION        std::string("LElbowRoll/Position/Actuator/Value")
#define DN_R_SHOULDER_ROLL_POSITION     std::string("RShoulderRoll/Position/Actuator/Value")
#define DN_R_SHOULDER_PITCH_POSITION    std::string("RShoulderPitch/Position/Actuator/Value")
#define DN_R_ELBOW_YAW_POSITION         std::string("RElbowYaw/Position/Actuator/Value") 
#define DN_R_ELBOW_ROLL_POSITION        std::string("RElbowRoll/Position/Actuator/Value")
#define DN_L_HIP_YAWPITCH_POSITION      std::string("LHipYawPitch/Position/Actuator/Value") 
#define DN_L_HIP_ROLL_POSITION          std::string("LHipRoll/Position/Actuator/Value") 
#define DN_L_HIP_PITCH_POSITION         std::string("LHipPitch/Position/Actuator/Value")
#define DN_L_KNEE_PITCH_POSITION        std::string("LKneePitch/Position/Actuator/Value")
#define DN_L_ANKLE_PITCH_POSITION       std::string("LAnklePitch/Position/Actuator/Value") 
#define DN_L_ANKLE_ROLL_POSITION        std::string("LAnkleRoll/Position/Actuator/Value")
#define DN_R_HIP_YAWPITCH_POSITION      std::string("RHipYawPitch/Position/Actuator/Value")
#define DN_R_HIP_ROLL_POSITION          std::string("RHipRoll/Position/Actuator/Value") 
#define DN_R_HIP_PITCH_POSITION         std::string("RHipPitch/Position/Actuator/Value")
#define DN_R_KNEE_PITCH_POSITION        std::string("RKneePitch/Position/Actuator/Value")
#define DN_R_ANKLE_PITCH_POSITION       std::string("RAnklePitch/Position/Actuator/Value") 
#define DN_R_ANKLE_ROLL_POSITION        std::string("RAnkleRoll/Position/Actuator/Value")

// Device names for hardness actuators
#define DN_HEAD_YAW_HARDNESS            std::string("HeadYaw/Hardness/Actuator/Value")
#define DN_HEAD_PITCH_HARDNESS          std::string("HeadPitch/Hardness/Actuator/Value")
#define DN_L_SHOULDER_ROLL_HARDNESS     std::string("LShoulderRoll/Hardness/Actuator/Value")
#define DN_L_SHOULDER_PITCH_HARDNESS    std::string("LShoulderPitch/Hardness/Actuator/Value")
#define DN_L_ELBOW_YAW_HARDNESS         std::string("LElbowYaw/Hardness/Actuator/Value") 
#define DN_L_ELBOW_ROLL_HARDNESS        std::string("LElbowRoll/Hardness/Actuator/Value")
#define DN_R_SHOULDER_ROLL_HARDNESS     std::string("RShoulderRoll/Hardness/Actuator/Value")
#define DN_R_SHOULDER_PITCH_HARDNESS    std::string("RShoulderPitch/Hardness/Actuator/Value")
#define DN_R_ELBOW_YAW_HARDNESS         std::string("RElbowYaw/Hardness/Actuator/Value") 
#define DN_R_ELBOW_ROLL_HARDNESS        std::string("RElbowRoll/Hardness/Actuator/Value")
#define DN_L_HIP_YAWPITCH_HARDNESS      std::string("LHipYawPitch/Hardness/Actuator/Value") 
#define DN_L_HIP_ROLL_HARDNESS          std::string("LHipRoll/Hardness/Actuator/Value") 
#define DN_L_HIP_PITCH_HARDNESS         std::string("LHipPitch/Hardness/Actuator/Value")
#define DN_L_KNEE_PITCH_HARDNESS        std::string("LKneePitch/Hardness/Actuator/Value")
#define DN_L_ANKLE_PITCH_HARDNESS       std::string("LAnklePitch/Hardness/Actuator/Value") 
#define DN_L_ANKLE_ROLL_HARDNESS        std::string("LAnkleRoll/Hardness/Actuator/Value")
#define DN_R_HIP_YAWPITCH_HARDNESS      std::string("RHipYawPitch/Hardness/Actuator/Value")
#define DN_R_HIP_ROLL_HARDNESS          std::string("RHipRoll/Hardness/Actuator/Value") 
#define DN_R_HIP_PITCH_HARDNESS         std::string("RHipPitch/Hardness/Actuator/Value")
#define DN_R_KNEE_PITCH_HARDNESS        std::string("RKneePitch/Hardness/Actuator/Value")
#define DN_R_ANKLE_PITCH_HARDNESS       std::string("RAnklePitch/Hardness/Actuator/Value") 
#define DN_R_ANKLE_ROLL_HARDNESS        std::string("RAnkleRoll/Hardness/Actuator/Value")

// Device names for ear LEDs
#define DN_LED_EAR_LEFT_0DEG            std::string("Ears/Led/Left/0Deg/Actuator/Value")
#define DN_LED_EAR_LEFT_36DEG           std::string("Ears/Led/Left/36Deg/Actuator/Value")
#define DN_LED_EAR_LEFT_72DEG           std::string("Ears/Led/Left/72Deg/Actuator/Value")
#define DN_LED_EAR_LEFT_108DEG          std::string("Ears/Led/Left/108Deg/Actuator/Value")
#define DN_LED_EAR_LEFT_144DEG          std::string("Ears/Led/Left/144Deg/Actuator/Value")
#define DN_LED_EAR_LEFT_180DEG          std::string("Ears/Led/Left/180Deg/Actuator/Value")
#define DN_LED_EAR_LEFT_216DEG          std::string("Ears/Led/Left/216Deg/Actuator/Value")
#define DN_LED_EAR_LEFT_252DEG          std::string("Ears/Led/Left/252Deg/Actuator/Value")
#define DN_LED_EAR_LEFT_288DEG          std::string("Ears/Led/Left/288Deg/Actuator/Value")
#define DN_LED_EAR_LEFT_324DEG          std::string("Ears/Led/Left/324Deg/Actuator/Value")

#define DN_LED_EAR_RIGHT_0DEG           std::string("Ears/Led/Right/0Deg/Actuator/Value")
#define DN_LED_EAR_RIGHT_36DEG          std::string("Ears/Led/Right/36Deg/Actuator/Value")
#define DN_LED_EAR_RIGHT_72DEG          std::string("Ears/Led/Right/72Deg/Actuator/Value")
#define DN_LED_EAR_RIGHT_108DEG         std::string("Ears/Led/Right/108Deg/Actuator/Value")
#define DN_LED_EAR_RIGHT_144DEG         std::string("Ears/Led/Right/144Deg/Actuator/Value")
#define DN_LED_EAR_RIGHT_180DEG         std::string("Ears/Led/Right/180Deg/Actuator/Value")
#define DN_LED_EAR_RIGHT_216DEG         std::string("Ears/Led/Right/216Deg/Actuator/Value")
#define DN_LED_EAR_RIGHT_252DEG         std::string("Ears/Led/Right/252Deg/Actuator/Value")
#define DN_LED_EAR_RIGHT_288DEG         std::string("Ears/Led/Right/288Deg/Actuator/Value")
#define DN_LED_EAR_RIGHT_324DEG         std::string("Ears/Led/Right/324Deg/Actuator/Value")

// Device names for eye LEDs
#define DN_LED_EYE_LEFT_RED_0DEG        std::string("Face/Led/Red/Left/0Deg/Actuator/Value")
#define DN_LED_EYE_LEFT_GREEN_0DEG      std::string("Face/Led/Green/Left/0Deg/Actuator/Value")
#define DN_LED_EYE_LEFT_BLUE_0DEG       std::string("Face/Led/Blue/Left/0Deg/Actuator/Value")
#define DN_LED_EYE_LEFT_RED_45DEG       std::string("Face/Led/Red/Left/45Deg/Actuator/Value")
#define DN_LED_EYE_LEFT_GREEN_45DEG     std::string("Face/Led/Green/Left/45Deg/Actuator/Value")
#define DN_LED_EYE_LEFT_BLUE_45DEG      std::string("Face/Led/Blue/Left/45Deg/Actuator/Value")
#define DN_LED_EYE_LEFT_RED_90DEG       std::string("Face/Led/Red/Left/90Deg/Actuator/Value")
#define DN_LED_EYE_LEFT_GREEN_90DEG     std::string("Face/Led/Green/Left/90Deg/Actuator/Value")
#define DN_LED_EYE_LEFT_BLUE_90DEG      std::string("Face/Led/Blue/Left/90Deg/Actuator/Value")
#define DN_LED_EYE_LEFT_RED_135DEG      std::string("Face/Led/Red/Left/135Deg/Actuator/Value")
#define DN_LED_EYE_LEFT_GREEN_135DEG    std::string("Face/Led/Green/Left/135Deg/Actuator/Value")
#define DN_LED_EYE_LEFT_BLUE_135DEG     std::string("Face/Led/Blue/Left/135Deg/Actuator/Value")
#define DN_LED_EYE_LEFT_RED_180DEG      std::string("Face/Led/Red/Left/180Deg/Actuator/Value")
#define DN_LED_EYE_LEFT_GREEN_180DEG    std::string("Face/Led/Green/Left/180Deg/Actuator/Value")
#define DN_LED_EYE_LEFT_BLUE_180DEG     std::string("Face/Led/Blue/Left/180Deg/Actuator/Value")
#define DN_LED_EYE_LEFT_RED_225DEG      std::string("Face/Led/Red/Left/225Deg/Actuator/Value")
#define DN_LED_EYE_LEFT_GREEN_225DEG    std::string("Face/Led/Green/Left/225Deg/Actuator/Value")
#define DN_LED_EYE_LEFT_BLUE_225DEG     std::string("Face/Led/Blue/Left/225Deg/Actuator/Value")
#define DN_LED_EYE_LEFT_RED_270DEG      std::string("Face/Led/Red/Left/270Deg/Actuator/Value")
#define DN_LED_EYE_LEFT_GREEN_270DEG    std::string("Face/Led/Green/Left/270Deg/Actuator/Value")
#define DN_LED_EYE_LEFT_BLUE_270DEG     std::string("Face/Led/Blue/Left/270Deg/Actuator/Value")
#define DN_LED_EYE_LEFT_RED_315DEG      std::string("Face/Led/Red/Left/315Deg/Actuator/Value")
#define DN_LED_EYE_LEFT_GREEN_315DEG    std::string("Face/Led/Green/Left/315Deg/Actuator/Value")
#define DN_LED_EYE_LEFT_BLUE_315DEG     std::string("Face/Led/Blue/Left/315Deg/Actuator/Value")

#define DN_LED_EYE_RIGHT_RED_0DEG       std::string("Face/Led/Red/Right/0Deg/Actuator/Value")
#define DN_LED_EYE_RIGHT_GREEN_0DEG     std::string("Face/Led/Green/Right/0Deg/Actuator/Value")
#define DN_LED_EYE_RIGHT_BLUE_0DEG      std::string("Face/Led/Blue/Right/0Deg/Actuator/Value")
#define DN_LED_EYE_RIGHT_RED_45DEG      std::string("Face/Led/Red/Right/45Deg/Actuator/Value")
#define DN_LED_EYE_RIGHT_GREEN_45DEG    std::string("Face/Led/Green/Right/45Deg/Actuator/Value")
#define DN_LED_EYE_RIGHT_BLUE_45DEG     std::string("Face/Led/Blue/Right/45Deg/Actuator/Value")
#define DN_LED_EYE_RIGHT_RED_90DEG      std::string("Face/Led/Red/Right/90Deg/Actuator/Value")
#define DN_LED_EYE_RIGHT_GREEN_90DEG    std::string("Face/Led/Green/Right/90Deg/Actuator/Value")
#define DN_LED_EYE_RIGHT_BLUE_90DEG     std::string("Face/Led/Blue/Right/90Deg/Actuator/Value")
#define DN_LED_EYE_RIGHT_RED_135DEG     std::string("Face/Led/Red/Right/135Deg/Actuator/Value")
#define DN_LED_EYE_RIGHT_GREEN_135DEG   std::string("Face/Led/Green/Right/135Deg/Actuator/Value")
#define DN_LED_EYE_RIGHT_BLUE_135DEG    std::string("Face/Led/Blue/Right/135Deg/Actuator/Value")
#define DN_LED_EYE_RIGHT_RED_180DEG     std::string("Face/Led/Red/Right/180Deg/Actuator/Value")
#define DN_LED_EYE_RIGHT_GREEN_180DEG   std::string("Face/Led/Green/Right/180Deg/Actuator/Value")
#define DN_LED_EYE_RIGHT_BLUE_180DEG    std::string("Face/Led/Blue/Right/180Deg/Actuator/Value")
#define DN_LED_EYE_RIGHT_RED_225DEG     std::string("Face/Led/Red/Right/225Deg/Actuator/Value")
#define DN_LED_EYE_RIGHT_GREEN_225DEG   std::string("Face/Led/Green/Right/225Deg/Actuator/Value")
#define DN_LED_EYE_RIGHT_BLUE_225DEG    std::string("Face/Led/Blue/Right/225Deg/Actuator/Value")
#define DN_LED_EYE_RIGHT_RED_270DEG     std::string("Face/Led/Red/Right/270Deg/Actuator/Value")
#define DN_LED_EYE_RIGHT_GREEN_270DEG   std::string("Face/Led/Green/Right/270Deg/Actuator/Value")
#define DN_LED_EYE_RIGHT_BLUE_270DEG    std::string("Face/Led/Blue/Right/270Deg/Actuator/Value")
#define DN_LED_EYE_RIGHT_RED_315DEG     std::string("Face/Led/Red/Right/315Deg/Actuator/Value")
#define DN_LED_EYE_RIGHT_GREEN_315DEG   std::string("Face/Led/Green/Right/315Deg/Actuator/Value")
#define DN_LED_EYE_RIGHT_BLUE_315DEG    std::string("Face/Led/Blue/Right/315Deg/Actuator/Value")

// Device names for the chest LEDs
#define DN_LED_CHEST_RED                std::string("ChestBoard/Led/Red/Actuator/Value")
#define DN_LED_CHEST_BLUE               std::string("ChestBoard/Led/Blue/Actuator/Value")
#define DN_LED_CHEST_GREEN              std::string("ChestBoard/Led/Green/Actuator/Value")

// Device names for the feet LEDs
#define DN_LED_FOOT_LEFT_RED            std::string("LFoot/Led/Red/Actuator/Value")
#define DN_LED_FOOT_LEFT_GREEN          std::string("LFoot/Led/Green/Actuator/Value")
#define DN_LED_FOOT_LEFT_BLUE           std::string("LFoot/Led/Blue/Actuator/Value")

#define DN_LED_FOOT_RIGHT_RED           std::string("RFoot/Led/Red/Actuator/Value")
#define DN_LED_FOOT_RIGHT_GREEN         std::string("RFoot/Led/Green/Actuator/Value")
#define DN_LED_FOOT_RIGHT_BLUE          std::string("RFoot/Led/Blue/Actuator/Value")

#endif
