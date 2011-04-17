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
#include "Infrastructure/NUSensorsData/NUSensorsData.h"
#include "Infrastructure/NUActionatorsData/NUActionatorsData.h"

#include <fstream>
using namespace std;

class JuppWalk : public NUWalk
{
public:
    JuppWalk(NUSensorsData* data, NUActionatorsData* actions);
    ~JuppWalk();
protected:
    void doWalk();
private:
    void initWalkParameters();
    void getParameters();
    
    void calculateGaitPhase();
    void calculateGyroFeedback();
    
    void calculateLeftLeg();
    void calculateRightLeg();
    void calculateLegAngles(float legphase, float legsign, vector<float>& angles);
    void calculateLegGains(float legphase, vector<float>& gains);
    
    void calculateLeftArm();
    void calculateRightArm();
    void calculateArmAngles(float armphase, float armsign, vector<float>& angles);
    void calculateArmGains(float armphase, vector<float>& gains);
    
    void updateActionatorsData();
public:
protected:
private:
    float m_leg_length;
    
    float m_gait_phase;
    float m_left_leg_phase;
    float m_right_leg_phase;
    
    float m_swing_amplitude_roll;
    float m_swing_amplitude_pitch;
    float m_swing_amplitude_yaw;
    
    // Walk Engine Parameters that require tuning
    float m_step_frequency;
    float m_param_phase_offset;     // the phase offset for the shortening, loading and swing phases
    float m_param_shift_c;         // controls the shift amplitude
    float m_param_ankle_shift;    // controls the fraction of the shift done by the ankles
    float m_param_short_c;          // controls the leg shortening amplitude
    float m_param_short_v;          // controls the duration of the leg shortening phase
    float m_param_load_c;
    float m_param_load_v;
    float m_param_swing_v;
    float m_param_balance_orientation;
    float m_param_balance_sagittal_sway;
    float m_param_gyro_roll;
    float m_param_gyro_pitch;
    float m_param_phase_reset_offset;
    
    
    // Gyro feedback
    float m_gyro_foot_pitch;
    float m_gyro_foot_roll;
    float m_gyro_leg_pitch;
    
    // Legs
    vector<float> m_left_leg_angles;
    vector<float> m_left_leg_gains;
    vector<float> m_right_leg_angles;
    vector<float> m_right_leg_gains;
    
    // Arms
    vector<float> m_left_arm_angles;
    vector<float> m_left_arm_gains;
    vector<float> m_right_arm_angles;
    vector<float> m_right_arm_gains;
};

#endif

