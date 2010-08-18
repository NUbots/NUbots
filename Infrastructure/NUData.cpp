/*! @file NUData.cpp
    @brief Implementation of an abstract NUData class
    @author Jason Kulk
 
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

#include "NUData.h"

NUData::id_t NUData::All = 0;
NUData::id_t NUData::Head = 1;
NUData::id_t NUData::Body = 3;
NUData::id_t NUData::LArm = 4;
NUData::id_t NUData::RArm = 5;
NUData::id_t NUData::Torso = 6;
NUData::id_t NUData::LLeg = 7;
NUData::id_t NUData::RLeg = 8;

NUData::id_t NUData::HeadRoll = -1;
NUData::id_t NUData::HeadPitch = -1;
NUData::id_t NUData::HeadYaw = -1;
NUData::id_t NUData::NeckRoll = -1;
NUData::id_t NUData::NeckPitch = -1;
NUData::id_t NUData::NeckYaw = -1;
NUData::id_t NUData::LShoulderRoll = -1;
NUData::id_t NUData::LShoulderPitch = -1;
NUData::id_t NUData::LShoulderYaw = -1;
NUData::id_t NUData::LElbowRoll = -1;
NUData::id_t NUData::LElbowPitch = -1;
NUData::id_t NUData::LElbowYaw = -1;
NUData::id_t NUData::RShoulderRoll = -1;
NUData::id_t NUData::RShoulderPitch = -1;
NUData::id_t NUData::RShoulderYaw = -1;
NUData::id_t NUData::RElbowRoll = -1;
NUData::id_t NUData::RElbowPitch = -1;
NUData::id_t NUData::RElbowYaw = -1;
NUData::id_t NUData::TorsoRoll = -1;
NUData::id_t NUData::TorsoPitch = -1;
NUData::id_t NUData::TorsoYaw = -1;
NUData::id_t NUData::LHipRoll = -1;
NUData::id_t NUData::LHipPitch = -1;
NUData::id_t NUData::LHipYaw = -1;
NUData::id_t NUData::LHipYawPitch = -1;
NUData::id_t NUData::LKneePitch = -1;
NUData::id_t NUData::LAnkleRoll = -1;
NUData::id_t NUData::LAnklePitch = -1;
NUData::id_t NUData::RHipRoll = -1;
NUData::id_t NUData::RHipPitch = -1;
NUData::id_t NUData::RHipYaw = -1;
NUData::id_t NUData::RHipYawPitch = -1;
NUData::id_t NUData::RKneePitch = -1;
NUData::id_t NUData::RAnkleRoll = -1;
NUData::id_t NUData::RAnklePitch = -1;

