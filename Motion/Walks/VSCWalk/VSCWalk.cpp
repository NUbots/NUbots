/*! @file VSCWalk.cpp
    @brief Implementation of VSCWalk class

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

#include "VSCWalk.h"

#include "NUPlatform/NUSystem.h"
#include "debug.h"
#include "debugverbositynumotion.h"

#include <math.h>

//! @todo TODO: put M_PI and NORMALISE somewhere else
#ifndef M_PI
    #define M_PI 3.1415926535
#endif

template <class T>
inline T NORMALISE(T theta){
    return atan2(sin(theta), cos(theta));
}

VSCWalk::VSCWalk()
{
    /* Jason's guide to find a set of parameters that work
        Step 1. Start with m_theta and m_alpha equal to zero. And use a small m_beta.
                Slowly increase m_gamma until the weight is shifted enough that the foot comes off the ground
        Step 2. Now increase m_beta until the foot comes off the ground enough for forward walk
        Step 3. Try and get some forward movement by selecting m_theta and m_alpha
                The paper says that faster walks require larger m_alpha. 
                So pick a m_theta value and try to find a m_alpha that makes the robot walk.
     */
    m_theta = 0.2;         // toe-off angle (step length) (rad) in paper 0.61rad
    m_alpha = m_theta*0.43; // heel strik thigh angle (compensatory term for impact losses) (rad) in paper 0.26rad
    m_beta = 0.52;          // swing leg thigh angle (step height) (rad) in paper 0.52 rad
    m_period = 600;         // the step period in milliseconds in paper 300ms
    
    m_psi = -0.03;          // leg offset angle (rad) in paper 0.087rad
    m_gamma = 0.42;         // torso shift angle (rad) in paper 0.17rad
    m_phi = 0;              // lateral leg swing thigh angle (rad) in paper 0.17rad
    
    m_lambda = 0.0;         // turn angle (rad) This walk engine can not turn without a fair amount of modification to keep the feet parallel to the ground
    
    m_t = 0;                // the step time in seconds
}

/*! @brief Destructor for walk module
 */
VSCWalk::~VSCWalk()
{
}

void VSCWalk::doWalk()
{
    debug << "VSCWalk::doWalk()" << std::endl;
    
    // I need to go from speed to:
    //          - the four forward gait parameters: theta, alpha, beta and period
    //          - the two lateral gait parameters: psi and gamma
    //          - the single lateral walk parameter: phi
    //          - the single turn parameter lambda
    
    m_previous_t = m_t;
    m_t = nusystem->getTime() - ((int) (nusystem->getTime()/m_period))*m_period;
    if (m_t < m_previous_t)
        m_step_leg = (m_step_leg + 1)%2;
    
    // calculate the pitch trajectories
    float hippitch1, hippitch2, kneepitch1, kneepitch2, anklepitch1, anklepitch2;
    if (m_t <= m_period/2.0)
    {
        hippitch1 = (m_theta/2)*cos(M_PI*m_t/m_period) + m_alpha;
        hippitch2 = -(m_theta/2)*cos(M_PI*m_t/m_period) + (m_beta/2.0)*(1 - cos(2*M_PI*m_t/m_period));
        kneepitch1 = 2*m_alpha;     // correct
        kneepitch2 = m_beta*(1 - cos(2*M_PI*m_t/m_period));
    }
    else
    {
        hippitch1 = (m_theta/2)*cos(M_PI*m_t/m_period) + (m_alpha/2.0)*(1 - cos(2*M_PI*m_t/m_period));
        hippitch2 = -(m_theta/2)*cos(M_PI*m_t/m_period) + m_alpha + ((m_beta - m_alpha)/2.0)*(1 - cos(2*M_PI*m_t/m_period));
        kneepitch1 = m_alpha*(1 - cos(2*M_PI*m_t/m_period));
        kneepitch2 = 2*m_alpha + (m_beta - m_alpha)*(1 - cos(2*M_PI*m_t/m_period));
    }
    anklepitch1 = hippitch1 - kneepitch1;       // for this engine the feet are constrained to be perpendicular to the torso
    anklepitch2 = hippitch2 - kneepitch2;
    // calculate the roll trajectories
    float hiproll1, hiproll2, ankleroll1, ankleroll2;
    float t = m_t + m_step_leg*m_period;
    hiproll1 = m_psi - (m_gamma/2)*sin(M_PI*t/m_period) + m_phi*cos(M_PI*t/m_period);
    hiproll2 = m_psi + (m_gamma/2)*sin(M_PI*t/m_period) + m_phi*cos(M_PI*t/m_period);
    ankleroll1 = -hiproll1;
    ankleroll2 = -hiproll2;
    // calculate the yaw trajectories
    float hipyaw1, hipyaw2;
    hipyaw1 = -m_lambda*sin(M_PI*t/(2*m_period));
    hipyaw2 = hipyaw1;
    
    calculateLeftArm();
    calculateRightArm();
    
    // now we need to decided whether 1 is the left or right leg
    if (m_step_leg == 0)
    {
        if (m_left_leg_angles.size() == 0)
            m_left_leg_angles = std::vector<float>(m_actions->getNumberOfJoints(NUActionatorsData::LeftLegJoints), 0);
        m_left_leg_angles[1] = -hippitch1;        // pitch
        m_left_leg_angles[3] = kneepitch1;        // knee pitch
        m_left_leg_angles[5] = anklepitch1;        // ankle pitch
        
        if (m_right_leg_angles.size() == 0)
            m_right_leg_angles = std::vector<float>(m_actions->getNumberOfJoints(NUActionatorsData::RightLegJoints), 0);
        m_right_leg_angles[1] = -hippitch2;        // pitch
        m_right_leg_angles[3] = kneepitch2;        // knee pitch
        m_right_leg_angles[5] = anklepitch2;        // ankle pitch
    }
    else
    {
        if (m_left_leg_angles.size() == 0)
            m_left_leg_angles = std::vector<float>(m_actions->getNumberOfJoints(NUActionatorsData::LeftLegJoints), 0);
        m_left_leg_angles[1] = -hippitch2;        // pitch
        m_left_leg_angles[3] = kneepitch2;        // knee pitch
        m_left_leg_angles[5] = anklepitch2;        // ankle pitch
        
        if (m_right_leg_angles.size() == 0)
            m_right_leg_angles = std::vector<float>(m_actions->getNumberOfJoints(NUActionatorsData::RightLegJoints), 0);
        m_right_leg_angles[1] = -hippitch1;        // pitch
        m_right_leg_angles[3] = kneepitch1;        // knee pitch
        m_right_leg_angles[5] = anklepitch1;        // ankle pitch
    }
    m_left_leg_angles[0] = hiproll1;        // roll
    m_left_leg_angles[1] -= 0.5*hipyaw1;
    m_left_leg_angles[2] = hipyaw1;        // yaw
    m_left_leg_angles[4] = ankleroll1;        // ankle roll
    m_right_leg_angles[0] = -hiproll2;        // roll
    m_left_leg_angles[1] -= 0.5*hipyaw2;
    m_right_leg_angles[2] = hipyaw2;        // yaw
    m_right_leg_angles[4] = -ankleroll2;        // ankle roll
    
    updateActionatorsData();
}

/*! @brief Calculates the left arm angles and gains
 */
void VSCWalk::calculateLeftArm()
{
    float leftlegphase = M_PI*(m_t + m_step_leg*m_period)/m_period - M_PI/2;
    calculateArmAngles(leftlegphase, -1, m_left_arm_angles);
    calculateArmGains(leftlegphase, m_left_arm_gains);
}

/*! @brief Calculates the right arm angles and gains
 */
void VSCWalk::calculateRightArm()
{
    float rightlegphase = M_PI*(m_t + m_step_leg*m_period)/m_period + M_PI/2;
    calculateArmAngles(rightlegphase, 1, m_right_arm_angles);
    calculateArmGains(rightlegphase, m_right_arm_gains);
}

/*! @brief Calculates arm angles based on the given legphase, and armsign
 @param legphase the phase for the arm
 @param armsign as with the leg; -1 for left and 1 for right
 */
void VSCWalk::calculateArmAngles(float legphase, float armsign, std::vector<float>& angles)
{
    if (angles.size() == 0)
        angles = std::vector<float> (m_actions->getNumberOfJoints(NUActionatorsData::LeftArmJoints),0);
    angles[0] = -0.15*armsign;                          // ShoulderRoll
    angles[1] = 0.4*sin(legphase + M_PI) + M_PI/2.0;    // ShoulderPitch
    angles[2] = 0;                                      // ElbowRoll
    angles[3] = armsign*M_PI/2;                         // ElbowYaw
}

/*! @brief Calculates the arm gains based on the given phase
 @param legphase the phase for the arm
 @param angles the parameter to get the new gains
 */
void VSCWalk::calculateArmGains(float legphase, std::vector<float>& gains)
{
    if (gains.size() == 0)
        gains = std::vector<float> (m_actions->getNumberOfJoints(NUActionatorsData::LeftArmJoints),0);
    gains[0] = 50;
    gains[1] = 50;
    gains[2] = 25;
    gains[3] = 25;
}

void VSCWalk::updateActionatorsData()
{
    static std::vector<float> zeroleg (m_actions->getNumberOfJoints(NUActionatorsData::LeftLegJoints), 0);
    static std::vector<float> zeroarm (m_actions->getNumberOfJoints(NUActionatorsData::LeftArmJoints), 0);
    
    static std::vector<float> m_left_leg_gains (m_actions->getNumberOfJoints(NUActionatorsData::LeftLegJoints), 100);
    static std::vector<float> m_right_leg_gains (m_actions->getNumberOfJoints(NUActionatorsData::RightLegJoints), 100);
    m_actions->addJointPositions(NUActionatorsData::LeftLegJoints, nusystem->getTime() + 40, m_left_leg_angles, zeroleg, m_left_leg_gains);
    m_actions->addJointPositions(NUActionatorsData::RightLegJoints, nusystem->getTime() + 40, m_right_leg_angles, zeroleg, m_right_leg_gains);
    if (m_larm_enabled)
        m_actions->addJointPositions(NUActionatorsData::LeftArmJoints, nusystem->getTime() + 40, m_left_arm_angles, zeroarm, m_left_arm_gains);
    if (m_rarm_enabled)
        m_actions->addJointPositions(NUActionatorsData::RightArmJoints, nusystem->getTime() + 40, m_right_arm_angles, zeroarm, m_right_arm_gains);
}



