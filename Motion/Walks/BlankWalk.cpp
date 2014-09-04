/*! @file BearWalk.cpp
    @brief Implementation of BearWalk class

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

#include "BearWalk.h"

#include "debug.h"
#include "debugverbositynumotion.h"

#include <math.h>

#ifndef M_PI
    #define M_PI 3.1415926535
#endif

BearWalk::BearWalk(NUSensorsData* data, NUActionatorsData* actions) : NUWalk(data, actions)
{
    m_gait_phase = 0;
    m_current_time = 0;
    m_previous_time = 0;
    
    // Initialise the leg values
    m_left_front_angles = std::vector<float>(3,0);
    m_right_front_angles = std::vector<float>(3,0);
    
    m_left_back_angles = std::vector<float>(5,0);
    m_right_back_angles = std::vector<float>(5,0);
    
    m_torso_angles = std::vector<float>(2,0);
    
    m_walk_parameters.load("BearWalkDefault");
}

/*! @brief Destructor for motion module
 */
BearWalk::~BearWalk()
{
}

void BearWalk::doWalk()
{   
    m_current_time = m_data->CurrentTime;
    calculateGaitPhase();
    
    // foot lifting
    float shortening = 0.1;
    float shortening_duration = 0.25;
    float lb_phase = m_gait_phase - (0.25 - shortening_duration)/2;
    if (lb_phase < 0)
        lb_phase++;
    float lf_phase = lb_phase - 0.25;
    if (lf_phase < 0)
        lf_phase++;
    float rb_phase = lb_phase - 0.50;
    if (rb_phase < 0)
        rb_phase++;
    float rf_phase = lb_phase - 0.75;
    if (rf_phase < 0)
        rf_phase++;
    
    float alpha_lb = 1;
    float alpha_lf = 1;
    float alpha_rb = 1;
    float alpha_rf = 1;
    if (lb_phase < shortening_duration)
        alpha_lb = 1 + shortening*0.5*(cos(2*M_PI*lb_phase/shortening_duration) - 1);
    if (lf_phase < shortening_duration)
        alpha_lf = 1 + shortening*0.5*(cos(2*M_PI*lf_phase/shortening_duration) - 1);
    if (rb_phase < shortening_duration)
        alpha_rb = 1 + shortening*0.5*(cos(2*M_PI*rb_phase/shortening_duration) - 1);
    if (rf_phase < shortening_duration)
        alpha_rf = 1 + shortening*0.5*(cos(2*M_PI*rf_phase/shortening_duration) - 1);
    
    // ankle pushing
    float push = 1.2;
    float push_duration = 0.1;
    float lb_push = 0;
    float rb_push = 0;
    if (lb_phase < push_duration or lb_phase > 1 - push_duration)
        lb_push = push*0.5*(1- cos(M_PI*lb_phase/push_duration - M_PI));
    if (rb_phase < push_duration or rb_phase > 1 - push_duration)
        rb_push = push*0.5*(1- cos(M_PI*rb_phase/push_duration - M_PI));
    
    // leg movement
    float forward_amp = 0.5;
    float lb_pitch = -0.2;
    float lf_pitch = 0.0;
    float rb_pitch = -0.2;
    float rf_pitch = 0.0;
    if (lb_phase < shortening_duration)
        lb_pitch += forward_amp - (2*forward_amp/shortening_duration)*lb_phase;
    else
        lb_pitch += (2*forward_amp/(1-shortening_duration))*(lb_phase - shortening_duration) - forward_amp;
    
    if (rb_phase < shortening_duration)
        rb_pitch += forward_amp - (2*forward_amp/shortening_duration)*rb_phase;
    else
        rb_pitch += (2*forward_amp/(1-shortening_duration))*(rb_phase - shortening_duration) - forward_amp;
    
    if (lf_phase < shortening_duration)
        lf_pitch += forward_amp - (2*forward_amp/shortening_duration)*lf_phase;
    else
        lf_pitch += (2*forward_amp/(1-shortening_duration))*(lf_phase - shortening_duration) - forward_amp;
        
    if (rf_phase < shortening_duration)
        rf_pitch += forward_amp - (2*forward_amp/shortening_duration)*rf_phase;
    else
        rf_pitch += (2*forward_amp/(1-shortening_duration))*(rf_phase - shortening_duration) - forward_amp;
    
    m_left_front_angles[1] = acos(alpha_lf) + lf_pitch;
    m_left_front_angles[2] = -2*acos(alpha_lf);
    
    m_right_front_angles[1] = acos(alpha_rf) + rf_pitch;
    m_right_front_angles[2] = -2*acos(alpha_rf);
    
    m_left_back_angles[1] = -acos(alpha_lb) + lb_pitch;
    m_left_back_angles[2] = 2*acos(alpha_lb);
    m_left_back_angles[4] = -acos(alpha_lb) - lb_pitch + lb_push;
    
    m_right_back_angles[1] = -acos(alpha_rb) + rb_pitch;
    m_right_back_angles[2] = 2*acos(alpha_rb);
    m_right_back_angles[4] = -acos(alpha_rb) - rb_pitch + rb_push;
    
    // weight shifting
    float shift_duration = 0.1;
    float shift_amount = 0.1;
    float shift_offset = -shift_duration/2;
    
    float shift = 0;
    float shift_phase = m_gait_phase - shift_offset;
    if (shift_phase < 0) shift_phase++;
    
    if (shift_phase < shift_duration)
        shift = (shift_amount/shift_duration)*shift_phase;
    else if (shift_phase < 0.5 - shift_duration)
        shift = shift_amount;
    else if (shift_phase < 0.5 + shift_duration)
        shift = -(shift_amount/shift_duration)*(shift_phase - 0.5);
    else if (shift_phase < 1 - shift_duration)
        shift = -shift_amount;
    else
        shift = (shift_amount/shift_duration)*(shift_phase - 1);
    
    m_left_front_angles[0] = shift;
    m_right_front_angles[0] = shift;
    m_left_back_angles[0] = 0.5*shift;
    m_right_back_angles[0] = 0.5*shift;
    
    m_left_back_angles[3] = -shift;
    m_right_back_angles[3] = -shift;
    
    // torso twist
    float twist_duration = 0.1;
    float twist_amount = 0.3;
    float twist_offset = -twist_duration/2 + 0.25;
    
    float twist = 0;
    float twist_phase = m_gait_phase - twist_offset;
    if (twist_phase < 0) twist_phase++;
    
    if (twist_phase < twist_duration)
        twist = (twist_amount/twist_duration)*twist_phase;
    else if (twist_phase < 0.5 - twist_duration)
        twist = twist_amount;
    else if (twist_phase < 0.5 + twist_duration)
        twist = -(twist_amount/twist_duration)*(twist_phase - 0.5);
    else if (twist_phase < 1 - twist_duration)
        twist = -twist_amount;
    else
        twist = (twist_amount/twist_duration)*(twist_phase - 1);
    
    m_torso_angles[1] = twist;
    
    updateActionatorsData();
    m_previous_time = m_current_time;
}

void BearWalk::calculateGaitPhase()
{
    double f = 0.6;         // TODO: load the step frequency here from m_walk_parameters
    
    double dt = (m_current_time - m_previous_time)/1000;
    if (dt < 0.2)
    {
        m_gait_phase += dt*f;
        if (m_gait_phase >= 1)
            m_gait_phase--;
    }
}

void BearWalk::updateActionatorsData()
{
    m_actions->add(NUActionatorsData::LLeg, m_current_time, m_left_back_angles, 100);
    m_actions->add(NUActionatorsData::RLeg, m_current_time, m_right_back_angles, 100);
    m_actions->add(NUActionatorsData::Torso, m_current_time, m_torso_angles, 100);
    if (m_larm_enabled)
        m_actions->add(NUActionatorsData::LArm, m_current_time, m_left_front_angles, 100);
    if (m_rarm_enabled)
        m_actions->add(NUActionatorsData::RArm, m_current_time, m_right_front_angles, 100);
}


