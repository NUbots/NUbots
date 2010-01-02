/*! @file JuppWalk.h
    @brief Declaration of Jupp's walk class
 
    @class JuppWalk
    @brief A module to provide locomotion
 
    This module is based on Sven Behnke's omni-directional walk engine detailed in the paper:
    Online Trajectory Generation for Omnidirectional Biped Walking, ICRA 2006.
 
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

#ifndef JUPPWALK_H
#define JUPPWALK_H

#include "Motion/NUWalk.h"
#include "NUPlatform/NUSensors/NUSensorsData.h"
#include "NUPlatform/NUActionators/NUActionatorsData.h"

class JuppWalk : public NUWalk
{
public:
    JuppWalk();
    ~JuppWalk();
protected:
    void doWalk();
private:
    void calculateLegAngles(float legphase, bool leftleg);
    void calculateArmAngles(float legphase, bool leftarm);
public:
protected:
private:
    float m_step_frequency;
    float m_leg_length;
    
    float m_current_time;
    float m_previous_time;
    
    float m_gait_phase;
    float m_left_leg_phase;
    float m_right_leg_phase;
    
    float m_swing_amplitude_roll;
    float m_swing_amplitude_pitch;
    float m_swing_amplitude_yaw;
};

#endif

