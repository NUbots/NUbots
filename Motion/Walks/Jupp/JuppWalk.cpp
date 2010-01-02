/*! @file JuppWalk.cpp
    @brief Implementation of JuppWalk class

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

#include "JuppWalk.h"

#include "NUPlatform/NUSystem.h"
#include "Tools/debug.h"

#include <math.h>

//! @todo TODO: put M_PI and NORMALISE somewhere else
#ifndef M_PI
    #define M_PI 3.1415926535
#endif

template <class T>
inline T NORMALISE(T theta){
    return atan2(sin(theta), cos(theta));
}

JuppWalk::JuppWalk()
{
    m_step_frequency = 0.8;
    m_leg_length = 20;          // The NAO has legs 20cm long
    
    m_current_time = nusystem->getTime();
    m_previous_time = m_current_time;
    
    m_gait_phase = 0;
    m_left_leg_phase = 0;
    m_right_leg_phase = 0;
    
    m_swing_amplitude_roll = 0;
    m_swing_amplitude_pitch = 0;
    m_swing_amplitude_yaw = 0;
}

/*! @brief Destructor for motion module
 */
JuppWalk::~JuppWalk()
{
    // nothing needs to be deleted at this level
}

void JuppWalk::doWalk()
{
    debug << "JuppWalk::doWalk()" << endl;
    // Convert speed vector into swing leg amplitudes (ar, ap, ay)
    m_swing_amplitude_roll = asin(-m_speed_y/(m_step_frequency*m_leg_length));
    m_swing_amplitude_pitch = asin(m_speed_x/(m_step_frequency*m_leg_length));
    m_swing_amplitude_yaw = m_speed_yaw/m_step_frequency;
    
    //cout << "swings: " << m_swing_amplitude_pitch << endl;
    
    // I need to tick the central clock, and then calculate the leg phases
    // now t needs to start from zero.
    m_current_time = nusystem->getTime();
    m_gait_phase = NORMALISE(m_gait_phase + 2*M_PI*m_step_frequency*(m_current_time - m_previous_time)/1000.0);
    m_previous_time = m_current_time;
    
    m_left_leg_phase = NORMALISE(m_gait_phase + M_PI/2);
    m_right_leg_phase = NORMALISE(m_gait_phase - M_PI/2);
    
    calculateLegAngles(m_left_leg_phase, true);
    calculateLegAngles(m_right_leg_phase, false);
    
    calculateArmAngles(m_left_leg_phase, true);
    calculateArmAngles(m_right_leg_phase, false);
}

void JuppWalk::calculateLegAngles(float legphase, bool leftleg)
{
    // Leg sign 
    float legsign;
    if (leftleg == true)
        legsign = -1;      // -1 for the left leg, 1 for the right leg
    else
        legsign = 1;
    
    // Shifting (this isn't effectively shift the weight to the other foot)
    // amp = 0.12 -> 0.24 with foot at 12.5% and feet close together
    float shift_amp = 0.23 + 0.08*sqrt(pow(m_swing_amplitude_roll, 2) + pow(m_swing_amplitude_pitch, 2)) + 1.2*fabs(m_swing_amplitude_roll);
    float shift = shift_amp*sin(legphase);   
    float shift_leg_roll = -legsign*1.0*shift;
    float shift_foot_roll = legsign*0.125*shift; // this needs to be tuned
    
    // Shortening
    float short_phase = 1.0*(legphase + M_PI/2.0 - 0.05);    // 3.0 controls the duration of the shortening, 0.05 determines the phase shift
    float short_amp = 0.3 + 1*sqrt(pow(m_swing_amplitude_roll, 2) + pow(m_swing_amplitude_pitch, 2));
    float short_leg_length = 0;
    float short_foot_pitch = 0;
    if (fabs(short_phase) < M_PI)
    {
        short_leg_length = -short_amp*0.5*(cos(short_phase) + 1);
        short_foot_pitch = -m_swing_amplitude_pitch*0.125*(cos(short_phase) + 1);
    }
    
    // Loading
    float load_phase = 1.0*NORMALISE(legphase + M_PI/2.0 - M_PI/3.0 - 0.05) - M_PI;
    float load_amp = 0.025 + 0.5*(1 - cos(fabs(m_swing_amplitude_pitch)));
    float load_leg_length = 0;
    if (fabs(load_phase) < M_PI)
        load_leg_length = -load_amp*0.5*(cos(load_phase) + 1);
    
    // Swinging
    float swing_phase = 1.0*(legphase + M_PI/2.0 - 0.05);          // 2.0 is the swing speed, and -0.15 is the phase shift
    float swing = 0;
    float b = -(2/(2*M_PI*2.0 - M_PI));                 // makes the reverse of the swing linear
    if (fabs(swing_phase) < M_PI/2.0)
        swing = sin(swing_phase);
    else if (swing_phase >= M_PI/2.0)
        swing = b*(swing_phase - M_PI/2 - 1);
    else
        swing = b*(swing_phase + M_PI/2 + 1);
    
    float swing_leg_roll = m_swing_amplitude_roll*swing;      // you always want the swing leg to go outwards
    float swing_leg_pitch = m_swing_amplitude_pitch*swing;
    float swing_leg_yaw = legsign*m_swing_amplitude_yaw*swing;
    float swing_foot_roll = -0.125*legsign*m_swing_amplitude_roll*swing;
    float swing_foot_pitch = 0.25*m_swing_amplitude_pitch*swing;
    
    // Balance
    float balance_foot_roll = 0.5*legsign*fabs(m_swing_amplitude_roll)*cos(legphase + 0.35);
    float balance_foot_pitch = 0.04 + 0.08*m_swing_amplitude_pitch - 0.04*m_swing_amplitude_pitch*cos(2*legphase + 0.7);
    float balance_leg_roll = legsign*-0.03 + m_swing_amplitude_roll + legsign*fabs(m_swing_amplitude_roll) + 0.1*m_swing_amplitude_yaw;
    
    
    // Output
    float leg_roll = swing_leg_roll + shift_leg_roll + balance_leg_roll;
    float leg_pitch = swing_leg_pitch;
    float leg_yaw = swing_leg_yaw;
    
    float foot_roll = swing_foot_roll + shift_foot_roll + balance_foot_roll;
    float foot_pitch = swing_foot_pitch + short_foot_pitch + balance_foot_pitch;
    float leg_length = short_leg_length + load_leg_length;
    
    // Now convert their leg interface to the joint angles!
    float knee_pitch = -2*acos(1 + 0.15*leg_length);
    float hip_yaw = leg_yaw;
    float hip_roll = leg_roll - 0.5*knee_pitch*sin(hip_yaw);
    float hip_pitch = leg_pitch - 0.5*knee_pitch*cos(hip_yaw);
    
    float ankle_roll = (foot_roll - leg_roll)*cos(hip_yaw) - (foot_pitch - leg_pitch)*sin(hip_yaw);
    float ankle_pitch = -0.5*knee_pitch + (foot_roll - leg_roll)*sin(hip_yaw) + (foot_pitch - leg_pitch)*cos(hip_yaw);
    
    // Now copy the angles to NUActionatorsData!
    static vector<float> positions (6, 0);
    static vector<float> velocities (6, 0);
    static vector<float> gains (6, 65);
    if (leftleg == true)
        positions[0] = hip_yaw;
    else
        positions[0] = hip_yaw;
    // Jupp's pitch and roll are reversed compared to our standard
    positions[1] = -hip_pitch - 0.5*hip_yaw;      // I need to compensate for the NAO's yawpitch joint
    positions[2] = -hip_roll;
    positions[3] = -knee_pitch;
    positions[4] = -ankle_pitch;
    positions[5] = -ankle_roll;
    
    debug << "positions: " << positions[0] << " " << positions[1] << " " << positions[2] << " " << positions[3] << " " << positions[4] << " " << positions[5] << endl;
    
    if (leftleg == true)
        m_actions->addJointPositions(NUActionatorsData::LLeg, nusystem->getTime(), positions, velocities, gains);
    else
        m_actions->addJointPositions(NUActionatorsData::RLeg, nusystem->getTime(), positions, velocities, gains);
}

void JuppWalk::calculateArmAngles(float legphase, bool leftarm)
{
    // Arm sign 
    float armsign;
    if (leftarm == true)
        armsign = -1;      // -1 for the left leg, 1 for the right leg
    else
        armsign = 1;
    
    float pitch = 0.5*sin(legphase + M_PI) + M_PI/2.0;
    static vector<float> positions (4, 0);
    static vector<float> velocities (4, 0);
    static vector<float> gains (4, 65);
    positions[0] = pitch;
    positions[1] = -0.1*armsign;
    positions[2] = M_PI/2;
    if (leftarm == true)
        m_actions->addJointPositions(NUActionatorsData::LArm, nusystem->getTime(), positions, velocities, gains);
    else
        m_actions->addJointPositions(NUActionatorsData::RArm, nusystem->getTime(), positions, velocities, gains);
}


