/*! @file NUData.h
    @brief Declaration of an abstract NUData class
    @author Jason Kulk
 
    @class NUData
    @brief An abstract NUData: a place to put things that are common to both the NUSensorsData and the NUActionatorsData
 
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

#ifndef NUDATA_H
#define NUDATA_H

class NUData
{
public:
    typedef int id_t;
    
    // Common aliases
    static id_t All;
    static id_t Head;
    static id_t Body;
    static id_t LArm;
    static id_t RArm;
    static id_t Torso;
    static id_t LLeg;
    static id_t RLeg;
    
    // define static members that are both sensors and actionators
    static id_t HeadRoll;
    static id_t HeadPitch;
    static id_t HeadYaw;
    static id_t NeckRoll;
    static id_t NeckPitch;
    static id_t NeckYaw;
    static id_t LShoulderRoll;
    static id_t LShoulderPitch;
    static id_t LShoulderYaw;
    static id_t LElbowRoll;
    static id_t LElbowPitch;
    static id_t LElbowYaw;
    static id_t RShoulderRoll;
    static id_t RShoulderPitch;
    static id_t RShoulderYaw;
    static id_t RElbowRoll;
    static id_t RElbowPitch;
    static id_t RElbowYaw;
    static id_t TorsoRoll;
    static id_t TorsoPitch;
    static id_t TorsoYaw;
    static id_t LHipRoll;
    static id_t LHipPitch;
    static id_t LHipYaw;
    static id_t LHipYawPitch;
    static id_t LKneePitch;
    static id_t LAnkleRoll;
    static id_t LAnklePitch;
    static id_t RHipRoll;
    static id_t RHipPitch;
    static id_t RHipYaw;
    static id_t RHipYawPitch;
    static id_t RKneePitch;
    static id_t RAnkleRoll;
    static id_t RAnklePitch;
    
    double CurrentTime;    
};

#endif

