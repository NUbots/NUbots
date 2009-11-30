/*! @file NUActionatorsData.cpp
    @brief Implementation of actionators data class

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

#include "NUActionatorsData.h"
#include "Tools/debug.h"


NUActionatorsData::NUActionatorsData()
{
    addActionator(&JointPositions, string("JointPositions"), JOINT_POSITIONS);
    addActionator(&JointVelocities, string("JointVelocities"), JOINT_VELOCITIES);
    addActionator(&CameraControl, string("CameraControl"), CAMERA_CONTROL);
    addActionator(&LedLEar, string("LedLEar"), LED_L_EAR);
    addActionator(&LedREar, string("LedREar"), LED_R_EAR);
    addActionator(&LedLEye, string("LedLEar"), LED_L_EYE);
    addActionator(&LedREye, string("LedLEar"), LED_R_EYE);
    addActionator(&LedLEar, string("LedLEar"), LED_L_EAR);
}

void addActionator(actionator_t** p_actionator, string actionatorname, actionator_id_t actionatorid)
{
    *p_actionator = new actionator_t(actionatorname, actionatorid);
    m_actionators.push_back(*p_actionator);
}

NUActionatorsData::~NUActionatorsData()
{
}

void NUActionatorsData::setAvaliableActionators(const vector<string>& actionators)
{
    
}


