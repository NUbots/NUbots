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

#include "debug.h"
#include "debugverbositynumotion.h"

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

JuppWalk::JuppWalk(NUSensorsData* data, NUActionatorsData* actions) : NUWalk(data, actions)
{
    initWalkParameters();
    
    m_leg_length = 20;          // The NAO has legs 20cm long
    m_current_time = 0;
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
    m_initial_lleg = m_left_leg_angles;
    m_left_leg_gains = vector<float> (6, 0);
    m_right_leg_angles = vector<float> (6, 0);
    m_initial_rleg = m_right_leg_angles;
    m_right_leg_gains = vector<float> (6, 0);
    
    // Initialise the arm values
    float larm[] = {0.1, 1.57, 0.15, -1.57};
    float rarm[] = {-0.1, 1.57, 0.15, 1.57};
    m_initial_larm = vector<float>(larm, larm + sizeof(larm)/sizeof(*larm));
    m_initial_rarm = vector<float>(rarm, rarm + sizeof(rarm)/sizeof(*rarm));
    
    m_left_arm_angles = m_initial_larm;
    m_left_arm_gains = vector<float> (4, 0);
    m_right_arm_angles = m_initial_rarm;
    m_right_arm_gains = vector<float> (4, 0);
    
    m_pattern_debug.open("patternDebug.csv");
    m_pattern_debug << "Phase (rad), Swing, Phase (rad), Swing" << endl;
}

void JuppWalk::initWalkParameters()
{
    m_step_frequency = 0.8;
    m_param_phase_offset = 0.60;                // the phase offset for the shortening, loading and swing phases
    // weight shift parameters
    m_param_shift_c = 0.24;                     // controls the shift amplitude
    m_param_ankle_shift = 0.125;                 // controls the fraction of the shift done by the ankles
    // leg shortening parameters
    m_param_short_c = 0.3;                      // controls the leg shortening amplitude
    m_param_short_v = 2.0;                      // controls the duration of the leg shortening phase
    // leg loading parameters
    m_param_load_c = 0.025;                     // controls the loading amplitude
    m_param_load_v = 2.0;                       // controls the loading duration
    // leg swing parameters
    m_param_swing_v = 2.0;                      // controls the swing duration
    // balance parameters
    m_param_balance_orientation = 0.03;         // controls the body orientation
    m_param_balance_sagittal_sway = 0.02;       // controls the sagittal sway amplitude
    // gyro parameters
    m_param_gyro_roll = 0.1;
    m_param_gyro_pitch = 0.1;
    // gait phase resetting
    m_param_phase_reset_offset = 0.24;
    
    // Create the default set of walk parameters
    vector<float> maxspeeds;
    maxspeeds.push_back(7.0);
    maxspeeds.push_back(2.5);
    maxspeeds.push_back(0.4);
    
    vector<float> maxaccels;
    maxaccels.push_back(3.5);
    maxaccels.push_back(1.25);
    maxaccels.push_back(0.2);
    
    vector<WalkParameters::Parameter> parameters;
    parameters.push_back(WalkParameters::Parameter("StepFrequency", m_step_frequency, 0.1, 3.0, "The step frequency in Hz"));
    parameters.push_back(WalkParameters::Parameter("PhaseOffset", m_param_phase_offset, -M_PI/2.0, M_PI/2.0, "The phase offset in radians for the shortening, swing and loading phases"));
    parameters.push_back(WalkParameters::Parameter("ShiftAmplitude", m_param_shift_c, 0.0, 0.4, "The amplitude of the left-right shifting motion"));
    parameters.push_back(WalkParameters::Parameter("ShiftAnkle", m_param_ankle_shift, 0.0, 1.0, "The fraction of the weight shifting done at the ankle level"));
    parameters.push_back(WalkParameters::Parameter("ShortAmplitude", m_param_short_c, 0.0, 1.0, "The amplitude of the shortening movement"));
    parameters.push_back(WalkParameters::Parameter("ShortDuration", m_param_short_v, 1.0, 4.0, "The duration of the shortening phase"));
    parameters.push_back(WalkParameters::Parameter("LoadAmplitude", m_param_load_c, 0.0, 1.0, "The amplitude of the loading movement"));
    parameters.push_back(WalkParameters::Parameter("LoadDuration", m_param_load_v, 1.0, 4.0, "The duration of the loading phase"));
    parameters.push_back(WalkParameters::Parameter("SwingDuration", m_param_swing_v, 1.0, 4.0, "The duration of the swing phase"));
    parameters.push_back(WalkParameters::Parameter("ForwardLean", m_param_balance_orientation, -0.1, 0.1, "The forward body lean in radians"));
    parameters.push_back(WalkParameters::Parameter("ForwardShiftAmplitude", m_param_balance_sagittal_sway, 0.0, 1.0, "The amplitude of the forward-backward shifting motion"));
    parameters.push_back(WalkParameters::Parameter("GyroRollGain", m_param_gyro_roll, 0.0, 1.0, "The gain of the roll controller"));
    parameters.push_back(WalkParameters::Parameter("GyroPitchGain", m_param_gyro_pitch, 0.0, 1.0, "The gain of the pitch controller"));
    parameters.push_back(WalkParameters::Parameter("PhaseReset", m_param_phase_reset_offset, -1.0, 1.0, "The phase reset offset triggered upon impact with the ground"));
    
    vector<vector<float> > armgains;
    armgains.push_back(vector<float>());
    armgains[0].push_back(50);
    armgains[0].push_back(50);
    armgains[0].push_back(25);
    armgains[0].push_back(25);
    
    vector<vector<float> > torsogains;
    
    vector<vector<float> > leggains;
    leggains.push_back(vector<float>());
    leggains[0].push_back(65);
    leggains[0].push_back(65);
    leggains[0].push_back(100);
    leggains[0].push_back(65);
    leggains[0].push_back(65);
    leggains[0].push_back(65);
    
    m_walk_parameters = WalkParameters("JuppWalkDefault", maxspeeds, maxaccels, parameters, armgains, torsogains, leggains);
    m_walk_parameters.save();
}

/*! @brief Gets the current walk parameters from the m_gait_walk_parameters array
 */
void JuppWalk::getParameters()
{
    vector<WalkParameters::Parameter>& parameters = m_walk_parameters.getParameters();
    m_step_frequency = parameters[0].Value; 
    m_param_phase_offset = parameters[1].Value;
    m_param_shift_c = parameters[2].Value;
    m_param_ankle_shift = parameters[3].Value;
    m_param_short_c = parameters[4].Value;
    m_param_short_v = parameters[5].Value;
    m_param_load_c = parameters[6].Value;
    m_param_load_v = parameters[7].Value;
    m_param_swing_v = parameters[8].Value;
    m_param_balance_orientation = parameters[9].Value;
    m_param_balance_sagittal_sway = parameters[10].Value;
    m_param_gyro_roll = parameters[11].Value;
    m_param_gyro_pitch = parameters[12].Value;
    m_param_phase_reset_offset = parameters[13].Value;
}

/*! @brief Destructor for motion module
 */
JuppWalk::~JuppWalk()
{
}

void JuppWalk::doWalk()
{
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
    // I need to learn to interpolate, because abrupt changes destablise the robot
    // so I want to shift to the measured phase over m_step_frequency/8 seconds
    static float leftimpacttime;
    static float rightimpacttime;
    static const float interpolationtime = 1000*m_step_frequency/4.0;
    static float gaitphaseonimpact = m_gait_phase;
    m_current_time = m_data->CurrentTime;
        
    if (m_current_time - leftimpacttime < interpolationtime)
    {
        float measuredphaseonimpact = M_PI/m_param_short_v - M_PI + m_param_phase_offset + m_param_phase_reset_offset;
        float phasediff = measuredphaseonimpact - gaitphaseonimpact;
        if (fabs(phasediff) < M_PI/8)
            m_gait_phase += (phasediff/interpolationtime)*(m_current_time - m_previous_time);
    }
    if (m_current_time - rightimpacttime < interpolationtime)
    {
        float measuredphaseonimpact = M_PI/m_param_short_v + m_param_phase_offset + m_param_phase_reset_offset;
        float phasediff = measuredphaseonimpact - gaitphaseonimpact;
        if (fabs(phasediff) < M_PI/8)
            m_gait_phase += (phasediff/interpolationtime)*(m_current_time - m_previous_time);
    }
    
    m_gait_phase = NORMALISE(m_gait_phase + 2*M_PI*m_step_frequency*(m_current_time - m_previous_time)/1000.0);
    
    m_previous_time = m_current_time;
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
     Step 2. Tune the shortening phase shift. Too late and the robot will fall, too early and the feet wont come off the ground!
     Step 3. Tune the swinging phase shift. You only want to swing when the foot is in the air (this will probably be the same as the shortening phase)
     */
    
    
    // Shifting
    float shift_amp = m_param_shift_c + 0.08*sqrt(pow(m_swing_amplitude_roll, 2) + pow(m_swing_amplitude_pitch, 2)) + 1.0*fabs(m_swing_amplitude_roll);
    float shift = shift_amp*sin(legphase);   
    float shift_leg_roll = -legsign*shift;
    float shift_foot_roll = legsign*m_param_ankle_shift*shift;

    // Shortening
    float short_phase = m_param_short_v*(legphase + M_PI/2.0 - m_param_phase_offset);
    float short_amp = m_param_short_c + 2*sqrt(pow(m_swing_amplitude_roll, 2) + pow(m_swing_amplitude_pitch, 2));
    if ((legsign > 0 && m_swing_amplitude_roll > 0) || (legsign < 0 && m_swing_amplitude_roll < 0))
    {   // shorten the inside leg a bit more when walking sidewards
        short_amp += 2*(1 - cos(m_swing_amplitude_roll));
    }

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
    float other_swing_phase = m_param_swing_v*(NORMALISE(legphase + M_PI) + M_PI/2.0 - m_param_phase_offset);       // swing phase for the other leg
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
    float swing_foot_roll = -0.125*legsign*m_swing_amplitude_roll*swing;
    float swing_foot_pitch = 0.25*m_swing_amplitude_pitch*swing;
    
    
    float swing_leg_yaw = 0;
    if (fabs(swing_phase) < M_PI/2.0)
    {   // if we are swinging this leg
        swing_leg_yaw = legsign*m_swing_amplitude_yaw*sin(swing_phase) - fabs(m_swing_amplitude_yaw);
    }
    else if (fabs(other_swing_phase) < M_PI/2.0)
    {   // if we are swinging the other leg
        swing_leg_yaw = -legsign*m_swing_amplitude_yaw*sin(other_swing_phase) - fabs(m_swing_amplitude_yaw);
    }            
    else if (swing_phase > M_PI/2.0 && swing_phase < 3*M_PI/2.0)
    {
        swing_leg_yaw = legsign*m_swing_amplitude_yaw - fabs(m_swing_amplitude_yaw);
    }
    else
    {
        swing_leg_yaw = -legsign*m_swing_amplitude_yaw - fabs(m_swing_amplitude_yaw);
    }
    
    m_pattern_debug << swing_phase << ", " << swing_leg_yaw << ", ";
    if (legsign > 0) 
        m_pattern_debug << endl;
    
    // Balance
    float balance_foot_roll = 0;//-2*legsign*m_swing_amplitude_roll*cos(legphase + 0.35);
    float balance_foot_pitch = m_param_balance_orientation + 0.05*m_swing_amplitude_pitch - m_param_balance_sagittal_sway*m_swing_amplitude_pitch*cos(2*(legphase - m_param_phase_offset));
    // leans in the sideward walk direction + term to keep the feet apart + term to keep the feet apart
    float balance_leg_roll = -m_swing_amplitude_roll + legsign*fabs(m_swing_amplitude_roll) + 0.2*legsign*fabs(m_swing_amplitude_yaw);
    
    // Apply gyro feedback
    if (fabs(swing_phase) < M_PI/2.0)      // if we are in the swing phase don't apply the foot_gyro_* offsets
    {
        m_gyro_foot_pitch = 0;
        m_gyro_foot_roll = 0;
        m_gyro_leg_pitch = 0;
    }
    
    // Now we can calculate the leg's state
    float leg_yaw = swing_leg_yaw;
    float leg_pitch = swing_leg_pitch + m_gyro_leg_pitch;
    float leg_roll = swing_leg_roll + shift_leg_roll + balance_leg_roll;
    float leg_length = short_leg_length + load_leg_length;
    
    float foot_pitch = swing_foot_pitch + short_foot_pitch + balance_foot_pitch + m_gyro_foot_pitch;
    float foot_roll = swing_foot_roll + shift_foot_roll + balance_foot_roll + m_gyro_foot_roll;
    
    // do the kinematics, and calculate the joint angles
    float knee_pitch = -2*acos(1 + 0.15*leg_length);
    float hip_yaw = leg_yaw;
    float hip_roll = leg_roll - 0.5*knee_pitch*sin(hip_yaw);
    float hip_pitch = leg_pitch - 0.5*knee_pitch*cos(hip_yaw);
    
    float ankle_roll = (foot_roll - leg_roll)*cos(hip_yaw) - (foot_pitch - leg_pitch)*sin(hip_yaw);
    float ankle_pitch = -0.5*knee_pitch + (foot_roll - leg_roll)*sin(hip_yaw) + (foot_pitch - leg_pitch)*cos(hip_yaw);
    
    // now translate to my coordinate system
    angles[0] = -hip_roll;
    angles[1] = -hip_pitch - 0.5*hip_yaw;      //!< @todo TODO: Figure out why this needs to be 0.65!
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
    vector<vector<float> >& leggains = m_walk_parameters.getLegGains();
    gains = leggains[0];
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
    vector<vector<float> >& armgains = m_walk_parameters.getArmGains();
    gains = armgains[0];
}

/*! @brief Calculates the gyro-based feedback terms m_gyro_foot_roll and m_gyro_foot_pitch
    
    The formula for this function comes from 
    Faber, 2007. "Stochastic Optimization of Bipedal Walking using Gyro Feedback and Phase Resetting"
 */
void JuppWalk::calculateGyroFeedback()
{
    static vector<float> values;        // [vx, vy, vz]
    m_data->get(NUSensorsData::Gyro, values);

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
    m_actions->add(NUActionatorsData::LLeg, m_current_time, m_left_leg_angles, m_left_leg_gains);
    m_actions->add(NUActionatorsData::RLeg, m_current_time, m_right_leg_angles, m_right_leg_gains);
    if (m_larm_enabled)
        m_actions->add(NUActionatorsData::LArm, m_current_time, m_left_arm_angles, m_left_arm_gains);
    if (m_rarm_enabled)
        m_actions->add(NUActionatorsData::RArm, m_current_time, m_right_arm_angles, m_right_arm_gains);
}



