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

#define DN_PREFIX           string("")
#define DN_POSTFIX          string("/Actuator/Value")

// Joint Names
#define HEAD_YAW             string("HeadYaw")
#define HEAD_PITCH           string("HeadPitch")
#define L_SHOULDER_ROLL      string("LShoulderRoll")
#define L_SHOULDER_PITCH     string("LShoulderPitch")
#define L_ELBOW_YAW          string("LElbowYaw") 
#define L_ELBOW_ROLL         string("LElbowRoll")
#define R_SHOULDER_ROLL      string("RShoulderRoll")
#define R_SHOULDER_PITCH     string("RShoulderPitch")
#define R_ELBOW_YAW          string("RElbowYaw") 
#define R_ELBOW_ROLL         string("RElbowRoll")
#define L_HIP_YAWPITCH       string("LHipYawPitch") 
#define L_HIP_ROLL           string("LHipRoll") 
#define L_HIP_PITCH          string("LHipPitch")
#define L_KNEE_PITCH         string("LKneePitch")
#define L_ANKLE_PITCH        string("LAnklePitch") 
#define L_ANKLE_ROLL         string("LAnkleRoll")
#define R_HIP_YAWPITCH       string("RHipYawPitch")
#define R_HIP_ROLL           string("RHipRoll") 
#define R_HIP_PITCH          string("RHipPitch")
#define R_KNEE_PITCH         string("RKneePitch")
#define R_ANKLE_PITCH        string("RAnklePitch") 
#define R_ANKLE_ROLL         string("RAnkleRoll")

// Device names for position actuators
#define HEAD_YAW_POSITION            string("HeadYaw/Position")
#define HEAD_PITCH_POSITION          string("HeadPitch/Position")
#define L_SHOULDER_ROLL_POSITION     string("LShoulderRoll/Position")
#define L_SHOULDER_PITCH_POSITION    string("LShoulderPitch/Position")
#define L_ELBOW_YAW_POSITION         string("LElbowYaw/Position") 
#define L_ELBOW_ROLL_POSITION        string("LElbowRoll/Position")
#define R_SHOULDER_ROLL_POSITION     string("RShoulderRoll/Position")
#define R_SHOULDER_PITCH_POSITION    string("RShoulderPitch/Position")
#define R_ELBOW_YAW_POSITION         string("RElbowYaw/Position") 
#define R_ELBOW_ROLL_POSITION        string("RElbowRoll/Position")
#define L_HIP_YAWPITCH_POSITION      string("LHipYawPitch/Position") 
#define L_HIP_ROLL_POSITION          string("LHipRoll/Position") 
#define L_HIP_PITCH_POSITION         string("LHipPitch/Position")
#define L_KNEE_PITCH_POSITION        string("LKneePitch/Position")
#define L_ANKLE_PITCH_POSITION       string("LAnklePitch/Position") 
#define L_ANKLE_ROLL_POSITION        string("LAnkleRoll/Position")
#define R_HIP_YAWPITCH_POSITION      string("RHipYawPitch/Position")
#define R_HIP_ROLL_POSITION          string("RHipRoll/Position") 
#define R_HIP_PITCH_POSITION         string("RHipPitch/Position")
#define R_KNEE_PITCH_POSITION        string("RKneePitch/Position")
#define R_ANKLE_PITCH_POSITION       string("RAnklePitch/Position") 
#define R_ANKLE_ROLL_POSITION        string("RAnkleRoll/Position")

// Device names for hardness actuators
#define HEAD_YAW_HARDNESS            string("HeadYaw/Hardness")
#define HEAD_PITCH_HARDNESS          string("HeadPitch/Hardness")
#define L_SHOULDER_ROLL_HARDNESS     string("LShoulderRoll/Hardness")
#define L_SHOULDER_PITCH_HARDNESS    string("LShoulderPitch/Hardness")
#define L_ELBOW_YAW_HARDNESS         string("LElbowYaw/Hardness") 
#define L_ELBOW_ROLL_HARDNESS        string("LElbowRoll/Hardness")
#define R_SHOULDER_ROLL_HARDNESS     string("RShoulderRoll/Hardness")
#define R_SHOULDER_PITCH_HARDNESS    string("RShoulderPitch/Hardness")
#define R_ELBOW_YAW_HARDNESS         string("RElbowYaw/Hardness") 
#define R_ELBOW_ROLL_HARDNESS        string("RElbowRoll/Hardness")
#define L_HIP_YAWPITCH_HARDNESS      string("LHipYawPitch/Hardness") 
#define L_HIP_ROLL_HARDNESS          string("LHipRoll/Hardness") 
#define L_HIP_PITCH_HARDNESS         string("LHipPitch/Hardness")
#define L_KNEE_PITCH_HARDNESS        string("LKneePitch/Hardness")
#define L_ANKLE_PITCH_HARDNESS       string("LAnklePitch/Hardness") 
#define L_ANKLE_ROLL_HARDNESS        string("LAnkleRoll/Hardness")
#define R_HIP_YAWPITCH_HARDNESS      string("RHipYawPitch/Hardness")
#define R_HIP_ROLL_HARDNESS          string("RHipRoll/Hardness") 
#define R_HIP_PITCH_HARDNESS         string("RHipPitch/Hardness")
#define R_KNEE_PITCH_HARDNESS        string("RKneePitch/Hardness")
#define R_ANKLE_PITCH_HARDNESS       string("RAnklePitch/Hardness") 
#define R_ANKLE_ROLL_HARDNESS        string("RAnkleRoll/Hardness")

// Device names for ear LEDs
#define LED_EAR_LEFT_0DEG            string("Ears/Led/Left/0Deg")
#define LED_EAR_LEFT_36DEG           string("Ears/Led/Left/36Deg")
#define LED_EAR_LEFT_72DEG           string("Ears/Led/Left/72Deg")
#define LED_EAR_LEFT_108DEG          string("Ears/Led/Left/108Deg")
#define LED_EAR_LEFT_144DEG          string("Ears/Led/Left/144Deg")
#define LED_EAR_LEFT_180DEG          string("Ears/Led/Left/180Deg")
#define LED_EAR_LEFT_216DEG          string("Ears/Led/Left/216Deg")
#define LED_EAR_LEFT_252DEG          string("Ears/Led/Left/252Deg")
#define LED_EAR_LEFT_288DEG          string("Ears/Led/Left/288Deg")
#define LED_EAR_LEFT_324DEG          string("Ears/Led/Left/324Deg")

#define LED_EAR_RIGHT_0DEG           string("Ears/Led/Right/0Deg")
#define LED_EAR_RIGHT_36DEG          string("Ears/Led/Right/36Deg")
#define LED_EAR_RIGHT_72DEG          string("Ears/Led/Right/72Deg")
#define LED_EAR_RIGHT_108DEG         string("Ears/Led/Right/108Deg")
#define LED_EAR_RIGHT_144DEG         string("Ears/Led/Right/144Deg")
#define LED_EAR_RIGHT_180DEG         string("Ears/Led/Right/180Deg")
#define LED_EAR_RIGHT_216DEG         string("Ears/Led/Right/216Deg")
#define LED_EAR_RIGHT_252DEG         string("Ears/Led/Right/252Deg")
#define LED_EAR_RIGHT_288DEG         string("Ears/Led/Right/288Deg")
#define LED_EAR_RIGHT_324DEG         string("Ears/Led/Right/324Deg")

// Device names for eye LEDs
#define LED_EYE_LEFT_RED_0DEG        string("Face/Led/Red/Left/0Deg")
#define LED_EYE_LEFT_GREEN_0DEG      string("Face/Led/Green/Left/0Deg")
#define LED_EYE_LEFT_BLUE_0DEG       string("Face/Led/Blue/Left/0Deg")
#define LED_EYE_LEFT_RED_45DEG       string("Face/Led/Red/Left/45Deg")
#define LED_EYE_LEFT_GREEN_45DEG     string("Face/Led/Green/Left/45Deg")
#define LED_EYE_LEFT_BLUE_45DEG      string("Face/Led/Blue/Left/45Deg")
#define LED_EYE_LEFT_RED_90DEG       string("Face/Led/Red/Left/90Deg")
#define LED_EYE_LEFT_GREEN_90DEG     string("Face/Led/Green/Left/90Deg")
#define LED_EYE_LEFT_BLUE_90DEG      string("Face/Led/Blue/Left/90Deg")
#define LED_EYE_LEFT_RED_135DEG      string("Face/Led/Red/Left/135Deg")
#define LED_EYE_LEFT_GREEN_135DEG    string("Face/Led/Green/Left/135Deg")
#define LED_EYE_LEFT_BLUE_135DEG     string("Face/Led/Blue/Left/135Deg")
#define LED_EYE_LEFT_RED_180DEG      string("Face/Led/Red/Left/180Deg")
#define LED_EYE_LEFT_GREEN_180DEG    string("Face/Led/Green/Left/180Deg")
#define LED_EYE_LEFT_BLUE_180DEG     string("Face/Led/Blue/Left/180Deg")
#define LED_EYE_LEFT_RED_225DEG      string("Face/Led/Red/Left/225Deg")
#define LED_EYE_LEFT_GREEN_225DEG    string("Face/Led/Green/Left/225Deg")
#define LED_EYE_LEFT_BLUE_225DEG     string("Face/Led/Blue/Left/225Deg")
#define LED_EYE_LEFT_RED_270DEG      string("Face/Led/Red/Left/270Deg")
#define LED_EYE_LEFT_GREEN_270DEG    string("Face/Led/Green/Left/270Deg")
#define LED_EYE_LEFT_BLUE_270DEG     string("Face/Led/Blue/Left/270Deg")
#define LED_EYE_LEFT_RED_315DEG      string("Face/Led/Red/Left/315Deg")
#define LED_EYE_LEFT_GREEN_315DEG    string("Face/Led/Green/Left/315Deg")
#define LED_EYE_LEFT_BLUE_315DEG     string("Face/Led/Blue/Left/315Deg")

#define LED_EYE_RIGHT_RED_0DEG       string("Face/Led/Red/Right/0Deg")
#define LED_EYE_RIGHT_GREEN_0DEG     string("Face/Led/Green/Right/0Deg")
#define LED_EYE_RIGHT_BLUE_0DEG      string("Face/Led/Blue/Right/0Deg")
#define LED_EYE_RIGHT_RED_45DEG      string("Face/Led/Red/Right/45Deg")
#define LED_EYE_RIGHT_GREEN_45DEG    string("Face/Led/Green/Right/45Deg")
#define LED_EYE_RIGHT_BLUE_45DEG     string("Face/Led/Blue/Right/45Deg")
#define LED_EYE_RIGHT_RED_90DEG      string("Face/Led/Red/Right/90Deg")
#define LED_EYE_RIGHT_GREEN_90DEG    string("Face/Led/Green/Right/90Deg")
#define LED_EYE_RIGHT_BLUE_90DEG     string("Face/Led/Blue/Right/90Deg")
#define LED_EYE_RIGHT_RED_135DEG     string("Face/Led/Red/Right/135Deg")
#define LED_EYE_RIGHT_GREEN_135DEG   string("Face/Led/Green/Right/135Deg")
#define LED_EYE_RIGHT_BLUE_135DEG    string("Face/Led/Blue/Right/135Deg")
#define LED_EYE_RIGHT_RED_180DEG     string("Face/Led/Red/Right/180Deg")
#define LED_EYE_RIGHT_GREEN_180DEG   string("Face/Led/Green/Right/180Deg")
#define LED_EYE_RIGHT_BLUE_180DEG    string("Face/Led/Blue/Right/180Deg")
#define LED_EYE_RIGHT_RED_225DEG     string("Face/Led/Red/Right/225Deg")
#define LED_EYE_RIGHT_GREEN_225DEG   string("Face/Led/Green/Right/225Deg")
#define LED_EYE_RIGHT_BLUE_225DEG    string("Face/Led/Blue/Right/225Deg")
#define LED_EYE_RIGHT_RED_270DEG     string("Face/Led/Red/Right/270Deg")
#define LED_EYE_RIGHT_GREEN_270DEG   string("Face/Led/Green/Right/270Deg")
#define LED_EYE_RIGHT_BLUE_270DEG    string("Face/Led/Blue/Right/270Deg")
#define LED_EYE_RIGHT_RED_315DEG     string("Face/Led/Red/Right/315Deg")
#define LED_EYE_RIGHT_GREEN_315DEG   string("Face/Led/Green/Right/315Deg")
#define LED_EYE_RIGHT_BLUE_315DEG    string("Face/Led/Blue/Right/315Deg")

// Device names for the chest LEDs
#define LED_CHEST_RED                string("ChestBoard/Led/Red")
#define LED_CHEST_GREEN              string("ChestBoard/Led/Green")
#define LED_CHEST_BLUE               string("ChestBoard/Led/Blue")

// Device names for the feet LEDs
#define LED_FOOT_LEFT_RED            string("LFoot/Led/Red")
#define LED_FOOT_LEFT_GREEN          string("LFoot/Led/Green")
#define LED_FOOT_LEFT_BLUE           string("LFoot/Led/Blue")

#define LED_FOOT_RIGHT_RED           string("RFoot/Led/Red")
#define LED_FOOT_RIGHT_GREEN         string("RFoot/Led/Green")
#define LED_FOOT_RIGHT_BLUE          string("RFoot/Led/Blue")

#endif
