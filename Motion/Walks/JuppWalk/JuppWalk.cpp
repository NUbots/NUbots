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
#include <boost/circular_buffer.hpp>
using namespace std;

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
    initWalkParameters();
    
    m_leg_length = 20;          // The NAO has legs 20cm long
    m_current_time = nusystem->getTime();
    m_previous_time = m_current_time;
    
    m_gait_phase = 0;
    m_left_leg_phase = 0;
    m_right_leg_phase = 0;
    
    m_swing_amplitude_roll = 0;
    m_swing_amplitude_pitch = 0;
    m_swing_amplitude_yaw = 0;
    
    // Initialise the leg values
    //! @todo Get the lengths of these angles and gains from m_actions
    m_left_leg_angles = vector<float> (6, 0);
    m_left_leg_gains = vector<float> (6, 0);
    m_right_leg_angles = vector<float> (6, 0);
    m_right_leg_gains = vector<float> (6, 0);
    
    // Initialise the arm values
    m_left_arm_angles = vector<float> (4, 0);
    m_left_arm_gains = vector<float> (4, 0);
    m_right_arm_angles = vector<float> (4, 0);
    m_right_arm_gains = vector<float> (4, 0);
    
    m_pattern_debug.open("patternDebug.log");
    m_pattern_debug << "Phase (rad), LegYaw, LegPitch, LegRoll, LegLength, FootPitch, FootRoll" << endl;
}

void JuppWalk::initWalkParameters()
{
    m_step_frequency = 0.8;
    m_param_phase_offset = 0.6;                 // the phase offset for the shortening, loading and swing phases
    // weight shift parameters
    m_param_shift_c = 0.24;                     // controls the shift amplitude
    m_param_ankle_shift = 0.125;                // controls the fraction of the shift done by the ankles
    // leg shortening parameters
    m_param_short_c = 0.3;                      // controls the leg shortening amplitude
    m_param_short_v = 2.0;                      // controls the duration of the leg shortening phase
    // leg loading parameters
    m_param_load_c = 0.025;                     // controls the loading amplitude
    m_param_load_v = 2.0;                       // controls the loading duration
    // leg swing parameters
    m_param_swing_v = 2.0;                      // controls the swing duration
    // balance parameters
    m_param_balance_orientation = 0.04;         // controls the body orientation
    m_param_balance_sagittal_sway = 0.02;       // controls the sagittal sway amplitude
    // gyro parameters
    m_param_gyro_roll = 0.1;
    m_param_gyro_pitch = 0.1;
    // gait phase resetting
    m_param_phase_reset_offset = 0.24;
    
    // At the moment this walk engine uses the same walk parameters for an entire walking cycle
    m_gait_walk_parameters.push_back(vector<WalkParameters::Parameter>());
    m_gait_walk_parameters[0].push_back(WalkParameters::Parameter(m_step_frequency, 0.1, 3.0));
    m_gait_walk_parameters[0].push_back(WalkParameters::Parameter(m_param_phase_offset, -M_PI/2.0, M_PI/2.0));
    m_gait_walk_parameters[0].push_back(WalkParameters::Parameter(m_param_shift_c, 0.0, 0.4));
    m_gait_walk_parameters[0].push_back(WalkParameters::Parameter(m_param_ankle_shift, 0.0, 1.0));
    m_gait_walk_parameters[0].push_back(WalkParameters::Parameter(m_param_short_c, 0.0, 1.0));
    m_gait_walk_parameters[0].push_back(WalkParameters::Parameter(m_param_short_v, 1.0, 4.0));
    m_gait_walk_parameters[0].push_back(WalkParameters::Parameter(m_param_load_c, 0.0, 1.0));
    m_gait_walk_parameters[0].push_back(WalkParameters::Parameter(m_param_load_v, 1.0, 4.0));
    m_gait_walk_parameters[0].push_back(WalkParameters::Parameter(m_param_swing_v, 1.0, 4.0));
    m_gait_walk_parameters[0].push_back(WalkParameters::Parameter(m_param_balance_orientation, -0.2, 0.2));
    m_gait_walk_parameters[0].push_back(WalkParameters::Parameter(m_param_balance_sagittal_sway, 0.0, 1.0));
    m_gait_walk_parameters[0].push_back(WalkParameters::Parameter(m_param_gyro_roll, 0.0, 1.0));
    m_gait_walk_parameters[0].push_back(WalkParameters::Parameter(m_param_gyro_pitch, 0.0, 1.0));
    m_gait_walk_parameters[0].push_back(WalkParameters::Parameter(m_param_phase_reset_offset, -1.0, 1.0));
    
    m_gait_max_speeds.push_back(7.0);
    m_gait_max_speeds.push_back(3.0);
    m_gait_max_speeds.push_back(0.5);
    
    m_gait_arm_gains.push_back(vector<float>());
    m_gait_arm_gains[0].push_back(50);
    m_gait_arm_gains[0].push_back(50);
    m_gait_arm_gains[0].push_back(25);
    m_gait_arm_gains[0].push_back(25);
    
    m_gait_leg_gains.push_back(vector<float>());
    m_gait_leg_gains[0].push_back(65);
    m_gait_leg_gains[0].push_back(65);
    m_gait_leg_gains[0].push_back(100);
    m_gait_leg_gains[0].push_back(65);
    m_gait_leg_gains[0].push_back(65);
    m_gait_leg_gains[0].push_back(65);
}

/*! @brief Gets the current walk parameters from the m_gait_walk_parameters array
 */
void JuppWalk::getParameters()
{
    m_step_frequency = m_gait_walk_parameters[0][0].Value; 
    m_param_phase_offset = m_gait_walk_parameters[0][1].Value;
    m_param_shift_c = m_gait_walk_parameters[0][2].Value;
    m_param_ankle_shift = m_gait_walk_parameters[0][3].Value;
    m_param_short_c = m_gait_walk_parameters[0][4].Value;
    m_param_short_v = m_gait_walk_parameters[0][5].Value;
    m_param_load_c = m_gait_walk_parameters[0][6].Value;
    m_param_load_v = m_gait_walk_parameters[0][7].Value;
    m_param_swing_v = m_gait_walk_parameters[0][8].Value;
    m_param_balance_orientation = m_gait_walk_parameters[0][9].Value;
    m_param_balance_sagittal_sway = m_gait_walk_parameters[0][10].Value;
    m_param_gyro_roll = m_gait_walk_parameters[0][11].Value;
    m_param_gyro_pitch = m_gait_walk_parameters[0][12].Value;
    m_param_phase_reset_offset = m_gait_walk_parameters[0][13].Value;
}

/*! @brief Destructor for motion module
 */
JuppWalk::~JuppWalk()
{
    m_left_leg_angles.clear();
    m_left_leg_gains.clear();
    m_right_leg_angles.clear();
    m_right_leg_gains.clear();
    
    m_left_arm_angles.clear();
    m_left_arm_gains.clear();
    m_right_arm_angles.clear();
    m_right_arm_gains.clear();
}

void JuppWalk::doWalk()
{
    debug << "JuppWalk::doWalk()" << endl;
    getParameters();
    // Convert speed vector into swing leg amplitudes (ar, ap, ay)
    m_swing_amplitude_roll = asin(-m_speed_y/(2*m_step_frequency*m_leg_length));
    m_swing_amplitude_pitch = asin(m_speed_x/(2*m_step_frequency*m_leg_length));
    m_swing_amplitude_yaw = m_speed_yaw/(2*m_step_frequency);
    
    calculateGaitPhase();
    m_left_leg_phase = NORMALISE(m_gait_phase + M_PI/2);
    m_right_leg_phase = NORMALISE(m_gait_phase - M_PI/2);
    
    calculateGyroFeedback();
    calculateLeftLeg();
    calculateRightLeg();
    
    calculateLeftArm();
    calculateRightArm();
    
    updateActionatorsData();
}

void JuppWalk::calculateGaitPhase()
{
    // do the phase feedback here!
    float impacttime = 0;

    m_current_time = nusystem->getTime();
    if (m_data->footImpact(NUSensorsData::LeftFoot, impacttime))
        m_gait_phase = M_PI/m_param_short_v + m_param_phase_offset - M_PI + m_param_phase_reset_offset;
    else if (m_data->footImpact(NUSensorsData::RightFoot, impacttime))
        m_gait_phase = M_PI/m_param_short_v + m_param_phase_offset + m_param_phase_reset_offset;
    else
        m_gait_phase = NORMALISE(m_gait_phase + 2*M_PI*m_step_frequency*(m_current_time - m_previous_time)/1000.0);

    m_previous_time = m_current_time;
    //cout << "phase: " << m_gait_phase << " left: " << leftsum << " right: " << rightsum << endl;
}

/*! @brief Calculates the angles and gains for the left leg
 */
void JuppWalk::calculateLeftLeg()
{
    calculateLegAngles(m_left_leg_phase, -1, m_left_leg_angles);
    calculateLegGains(m_left_leg_phase, m_left_leg_gains);
}

/*! @brief Calculates the angles and gains for the right leg
 */
void JuppWalk::calculateRightLeg()
{
    calculateLegAngles(m_right_leg_phase, 1, m_right_leg_angles);
    calculateLegGains(m_right_leg_phase, m_right_leg_gains);
}

/*! @brief Calculates leg angles based on the given legphase, and legsign
    @param legphase the phase of the leg you want angles for (+/- M_PI)
    @param legsign 1 for the right leg, -1 for the left leg
 */
void JuppWalk::calculateLegAngles(float legphase, float legsign, vector<float>& angles)
{
    /* Jason's guide to this to walk:
     Step 1. Tune shift_amp such that it looks like the weight is being shifted between the feet
     Step 2. Tune the shortening phase shift. Too late and the robot will fall, too late and the feet wont come off the ground!
     Step 3. Tune the swinging phase shift. You only want to swing when the foot is in the air (this will probably be the same as the shortening phase)
     */
    
    
    // Shifting
    float shift_amp = m_param_shift_c + 0.08*sqrt(pow(m_swing_amplitude_roll, 2) + pow(m_swing_amplitude_pitch, 2)) + 1.3*fabs(m_swing_amplitude_roll);
    float shift = shift_amp*sin(legphase);   
    float shift_leg_roll = -legsign*shift;
    float shift_foot_roll = legsign*m_param_ankle_shift*shift;
    
    // Shortening
    float short_phase = m_param_short_v*(legphase + M_PI/2.0 - m_param_phase_offset);
    float short_amp = m_param_short_c + 2*sqrt(pow(m_swing_amplitude_roll, 2) + pow(m_swing_amplitude_pitch, 2));
    // if legsign and m_swing_amplitude_roll have the same sign shorten more!
    if (legsign > 0 && m_swing_amplitude_roll > 0)
        short_amp += 0.15;
    else if (legsign < 0 && m_swing_amplitude_roll < 0)
        short_amp += 0.15;
    float short_leg_length = 0;
    float short_foot_pitch = 0;
    if (fabs(short_phase) < M_PI)
    {
        short_leg_length = -short_amp*0.5*(cos(short_phase) + 1);
        short_foot_pitch = fabs(m_swing_amplitude_pitch)*0.125*(cos(short_phase) + 1);       // this works really well when walking backwards!
    }
    
    // Loading
    float load_phase = m_param_load_v*NORMALISE(legphase + M_PI/2.0 - M_PI/m_param_short_v - m_param_phase_offset) - M_PI;
    float load_amp = m_param_load_c + 0.5*(1 - cos(fabs(m_swing_amplitude_pitch)));
    float load_leg_length = 0;
    if (fabs(load_phase) < M_PI)
        load_leg_length = -load_amp*0.5*(cos(load_phase) + 1);
    
    // Swinging
    float swing_phase = m_param_swing_v*(legphase + M_PI/2.0 - m_param_phase_offset);
    float swing = 0;
    float b = -(2/(2*M_PI*2.0 - M_PI));                 // makes the reverse of the swing linear
    if (fabs(swing_phase) < M_PI/2.0)
        swing = sin(swing_phase);
    else if (swing_phase >= M_PI/2.0)
        swing = b*(swing_phase - M_PI/2) + 1;
    else
        swing = b*(swing_phase + M_PI/2) - 1;
    
    float swing_leg_roll = m_swing_amplitude_roll*swing;      // you always want the swing leg to go outwards
    float swing_leg_pitch = m_swing_amplitude_pitch*swing;
    float swing_leg_yaw = legsign*m_swing_amplitude_yaw*swing;
    float swing_foot_roll = -0.125*legsign*m_swing_amplitude_roll*swing;
    float swing_foot_pitch = 0.25*m_swing_amplitude_pitch*swing;
    
    // Balance
    float balance_foot_roll = 0.5*legsign*fabs(m_swing_amplitude_roll)*cos(legphase + 0.35);
    float balance_foot_pitch = m_param_balance_orientation + 0.15*m_swing_amplitude_pitch - m_param_balance_sagittal_sway*m_swing_amplitude_pitch*cos(2*(legphase - m_param_phase_offset));
    float balance_leg_pitch = -0.00;
    float balance_leg_roll = legsign*-0.03 - 1.08*m_swing_amplitude_roll + legsign*fabs(m_swing_amplitude_roll) + 0.1*m_swing_amplitude_yaw;
    
    // Apply gyro feedback
    if (fabs(swing_phase) < M_PI/2.0)      // if we are in the swing phase don't apply the foot_gyro_* offsets
    {
        m_gyro_foot_pitch = 0;
        m_gyro_foot_roll = 0;
        m_gyro_leg_pitch = 0;
    }
    
    // Now we can calculate the leg's state
    float leg_yaw = swing_leg_yaw;
    float leg_pitch = swing_leg_pitch + balance_leg_pitch + m_gyro_leg_pitch;
    float leg_roll = swing_leg_roll + shift_leg_roll + balance_leg_roll;
    float leg_length = short_leg_length + load_leg_length;
    
    float foot_pitch = swing_foot_pitch + short_foot_pitch + balance_foot_pitch + m_gyro_foot_pitch;
    float foot_roll = swing_foot_roll + shift_foot_roll + balance_foot_roll + m_gyro_foot_roll;
    
    if (legsign > 0)
        m_pattern_debug << m_right_leg_phase << ", " << leg_yaw << ", " << leg_pitch << ", " << leg_roll << ", " << leg_length << ", " << foot_pitch << ", " << foot_roll << endl;
    
    // do the kinematics, and calculate the joint angles
    float knee_pitch = -2*acos(1 + 0.15*leg_length);
    float hip_yaw = leg_yaw;
    float hip_roll = leg_roll - 0.5*knee_pitch*sin(hip_yaw);
    float hip_pitch = leg_pitch - 0.5*knee_pitch*cos(hip_yaw);
    
    float ankle_roll = (foot_roll - leg_roll)*cos(hip_yaw) - (foot_pitch - leg_pitch)*sin(hip_yaw);
    float ankle_pitch = -0.5*knee_pitch + (foot_roll - leg_roll)*sin(hip_yaw) + (foot_pitch - leg_pitch)*cos(hip_yaw);
    
    // now translate to my coordinate system
    angles[0] = -hip_roll;
    angles[1] = -hip_pitch - 0.5*hip_yaw;      // I need to compensate for the NAO's yawpitch joint
    angles[2] = hip_yaw;
    angles[3] = -knee_pitch;
    angles[4] = -ankle_roll;
    angles[5] = -ankle_pitch;
}

/*! @brief Calculates the leg gains based on the given phase
    @param legphase the phase of the leg
    @param gains the parameter to be updated with the new gains
 */
void JuppWalk::calculateLegGains(float legphase, vector<float>& gains)
{
    gains = m_gait_leg_gains[0];
}

/*! @brief Calculates the left arm angles and gains
 */
void JuppWalk::calculateLeftArm()
{
    calculateArmAngles(m_left_leg_phase, -1, m_left_arm_angles);
    calculateArmGains(m_left_leg_phase, m_left_arm_gains);
}

/*! @brief Calculates the right arm angles and gains
 */
void JuppWalk::calculateRightArm()
{
    calculateArmAngles(m_right_leg_phase, 1, m_right_arm_angles);
    calculateArmGains(m_right_leg_phase, m_right_arm_gains);
}

/*! @brief Calculates arm angles based on the given legphase, and armsign
    @param legphase the phase for the arm
    @param armsign as with the leg; -1 for left and 1 for right
 */
void JuppWalk::calculateArmAngles(float legphase, float armsign, vector<float>& angles)
{
    angles[0] = -0.15*armsign;                          // ShoulderRoll
    angles[1] = 0.4*sin(legphase + M_PI) + M_PI/2.0;    // ShoulderPitch
    angles[2] = 0;                                      // ElbowRoll
    angles[3] = armsign*M_PI/2;                         // ElbowYaw
}

/*! @brief Calculates the arm gains based on the given phase
    @param legphase the phase for the arm
    @param angles the parameter to get the new gains
 */
void JuppWalk::calculateArmGains(float legphase, vector<float>& gains)
{
    gains = m_gait_arm_gains[0];
}

/*! @brief Calculates the gyro-based feedback terms m_gyro_foot_roll and m_gyro_foot_pitch
    
    The formula for this function comes from 
    Faber, 2007. "Stochastic Optimization of Bipedal Walking using Gyro Feedback and Phase Resetting"
 */
void JuppWalk::calculateGyroFeedback()
{
    static vector<float> values;        // [vx, vy, vz]
    m_data->getGyroValues(values);

    static const float roll_threshold = 0.10;
    static const float pitch_threshold = 0.10;          
    if (values[0] > roll_threshold)
        m_gyro_foot_roll = -m_param_gyro_roll*(values[0] - roll_threshold);
    else if (values[0] < -roll_threshold)
        m_gyro_foot_roll = -m_param_gyro_roll*(values[0] + roll_threshold);
    else
        m_gyro_foot_roll = 0;

    
    if (values[1] > pitch_threshold)
    {
        m_gyro_foot_pitch = -m_param_gyro_pitch*(values[1] - pitch_threshold);
        m_gyro_leg_pitch = -0.5*m_param_gyro_pitch*(values[1] - pitch_threshold);
    }
    else if (values[1] < -pitch_threshold)
    {
        m_gyro_foot_pitch = -m_param_gyro_pitch*(values[1] + pitch_threshold);
        m_gyro_leg_pitch = -0.5*m_param_gyro_pitch*(values[1] + pitch_threshold);
    }
    else
    {
        m_gyro_foot_pitch = 0;
        m_gyro_leg_pitch = 0;
    }
}

void JuppWalk::updateActionatorsData()
{
    static vector<float> zerovelleg (m_actions->getNumberOfJoints(NUActionatorsData::LeftLegJoints), 0);
    static vector<float> zerovelarm (m_actions->getNumberOfJoints(NUActionatorsData::LeftArmJoints), 0);
    m_actions->addJointPositions(NUActionatorsData::LeftLegJoints, m_current_time + 40, m_left_leg_angles, zerovelleg, m_left_leg_gains);
    m_actions->addJointPositions(NUActionatorsData::RightLegJoints, m_current_time + 40, m_right_leg_angles, zerovelleg, m_right_leg_gains);
    m_actions->addJointPositions(NUActionatorsData::LeftArmJoints, m_current_time + 40, m_left_arm_angles, zerovelarm, m_left_arm_gains);
    m_actions->addJointPositions(NUActionatorsData::RightArmJoints, m_current_time + 40, m_right_arm_angles, zerovelarm, m_right_arm_gains);
}



