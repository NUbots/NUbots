/*! @file NAOActionatorNames.h
    @brief Definition of NAO actionator names (ie. Device names to be used with dcm)
 
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

#ifndef NAOACTIONATORNAMES_H
#define NAOACTIONATORNAMES_H

#include <string>
using namespace std;

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

#define DN_HEAD_YAW_POSITION          std::string("HeadYaw/Position/Actuator/Value")
#define DN_HEAD_PITCH_POSITION        std::string("HeadPitch/Position/Actuator/Value")
#define DN_L_SHOULDER_ROLL_POSITION   std::string("LShoulderRoll/Position/Actuator/Value")
#define DN_L_SHOULDER_PITCH_POSITION  std::string("LShoulderPitch/Position/Actuator/Value")
#define DN_L_ELBOW_YAW_POSITION       std::string("LElbowYaw/Position/Actuator/Value") 
#define DN_L_ELBOW_ROLL_POSITION      std::string("LElbowRoll/Position/Actuator/Value")
#define DN_R_SHOULDER_ROLL_POSITION   std::string("RShoulderRoll/Position/Actuator/Value")
#define DN_R_SHOULDER_PITCH_POSITION  std::string("RShoulderPitch/Position/Actuator/Value")
#define DN_R_ELBOW_YAW_POSITION       std::string("RElbowYaw/Position/Actuator/Value") 
#define DN_R_ELBOW_ROLL_POSITION      std::string("RElbowRoll/Position/Actuator/Value")
#define DN_L_HIP_YAWPITCH_POSITION    std::string("LHipYawPitch/Position/Actuator/Value") 
#define DN_L_HIP_ROLL_POSITION        std::string("LHipRoll/Position/Actuator/Value") 
#define DN_L_HIP_PITCH_POSITION       std::string("LHipPitch/Position/Actuator/Value")
#define DN_L_KNEE_PITCH_POSITION      std::string("LKneePitch/Position/Actuator/Value")
#define DN_L_ANKLE_PITCH_POSITION     std::string("LAnklePitch/Position/Actuator/Value") 
#define DN_L_ANKLE_ROLL_POSITION      std::string("LAnkleRoll/Position/Actuator/Value")
#define DN_R_HIP_YAWPITCH_POSITION    std::string("RHipYawPitch/Position/Actuator/Value")
#define DN_R_HIP_ROLL_POSITION        std::string("RHipRoll/Position/Actuator/Value") 
#define DN_R_HIP_PITCH_POSITION       std::string("RHipPitch/Position/Actuator/Value")
#define DN_R_KNEE_PITCH_POSITION      std::string("RKneePitch/Position/Actuator/Value")
#define DN_R_ANKLE_PITCH_POSITION     std::string("RAnklePitch/Position/Actuator/Value") 
#define DN_R_ANKLE_ROLL_POSITION      std::string("RAnkleRoll/Position/Actuator/Value")

#define DN_HEAD_YAW_HARDNESS          std::string("HeadYaw/Hardness/Actuator/Value")
#define DN_HEAD_PITCH_HARDNESS        std::string("HeadPitch/Hardness/Actuator/Value")
#define DN_L_SHOULDER_ROLL_HARDNESS   std::string("LShoulderRoll/Hardness/Actuator/Value")
#define DN_L_SHOULDER_PITCH_HARDNESS  std::string("LShoulderPitch/Hardness/Actuator/Value")
#define DN_L_ELBOW_YAW_HARDNESS       std::string("LElbowYaw/Hardness/Actuator/Value") 
#define DN_L_ELBOW_ROLL_HARDNESS      std::string("LElbowRoll/Hardness/Actuator/Value")
#define DN_R_SHOULDER_ROLL_HARDNESS   std::string("RShoulderRoll/Hardness/Actuator/Value")
#define DN_R_SHOULDER_PITCH_HARDNESS  std::string("RShoulderPitch/Hardness/Actuator/Value")
#define DN_R_ELBOW_YAW_HARDNESS       std::string("RElbowYaw/Hardness/Actuator/Value") 
#define DN_R_ELBOW_ROLL_HARDNESS      std::string("RElbowRoll/Hardness/Actuator/Value")
#define DN_L_HIP_YAWPITCH_HARDNESS    std::string("LHipYawPitch/Hardness/Actuator/Value") 
#define DN_L_HIP_ROLL_HARDNESS        std::string("LHipRoll/Hardness/Actuator/Value") 
#define DN_L_HIP_PITCH_HARDNESS       std::string("LHipPitch/Hardness/Actuator/Value")
#define DN_L_KNEE_PITCH_HARDNESS      std::string("LKneePitch/Hardness/Actuator/Value")
#define DN_L_ANKLE_PITCH_HARDNESS     std::string("LAnklePitch/Hardness/Actuator/Value") 
#define DN_L_ANKLE_ROLL_HARDNESS      std::string("LAnkleRoll/Hardness/Actuator/Value")
#define DN_R_HIP_YAWPITCH_HARDNESS    std::string("RHipYawPitch/Hardness/Actuator/Value")
#define DN_R_HIP_ROLL_HARDNESS        std::string("RHipRoll/Hardness/Actuator/Value") 
#define DN_R_HIP_PITCH_HARDNESS       std::string("RHipPitch/Hardness/Actuator/Value")
#define DN_R_KNEE_PITCH_HARDNESS      std::string("RKneePitch/Hardness/Actuator/Value")
#define DN_R_ANKLE_PITCH_HARDNESS     std::string("RAnklePitch/Hardness/Actuator/Value") 
#define DN_R_ANKLE_ROLL_HARDNESS      std::string("RAnkleRoll/Hardness/Actuator/Value")

#endif
