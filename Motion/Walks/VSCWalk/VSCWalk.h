/*! @file VSCWalk.h
    @brief Declaration of Virtual Slope Control walk class
 
    @class VSCWalk
    @brief A module to provide locomotion
 
    This module is based on the omnidirectional walk described in the following paper:
    Mingguo Zhao, "Humanoid Robot Gait Generation Based on Limit Cycle Stability", 2009.
 
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

#ifndef VSCWALK_H
#define VSCWALK_H

#include "Motion/NUWalk.h"
#include "NUPlatform/NUSensors/NUSensorsData.h"
#include "NUPlatform/NUActionators/NUActionatorsData.h"

#include <fstream>
using namespace std;

class VSCWalk : public NUWalk
{
public:
    VSCWalk();
    ~VSCWalk();
protected:
    void doWalk();
private:
    void calculateLeftArm();
    void calculateRightArm();
    void calculateArmAngles(float legphase, float armsign, vector<float>& angles);
    void calculateArmGains(float legphase, vector<float>& gains);
    
    void updateActionatorsData();

public:
protected:
private:
    float m_theta;              //!< toe-off angle (rad)
    float m_alpha;              //!< heel strik thigh angle (rad)
    float m_beta;               //!< swing leg thigh angle (rad)
    float m_period;             //!< the step period in seconds
    
    float m_psi;
    float m_gamma;
    float m_phi;
    
    float m_lambda;
    
    float m_t;                  //!< the step time in milliseconds
    float m_previous_t;         //!< the previous step time in milliseconds
    int m_step_leg;             //!< the side (left or right) of the current step. 0 if its a left, and 1 if its a right step
    
    vector<float> m_left_leg_angles;
    vector<float> m_right_leg_angles;
    vector<float> m_left_leg_gains;
    vector<float> m_right_leg_gains;
    
    vector<float> m_left_arm_angles;
    vector<float> m_right_arm_angles;
    vector<float> m_left_arm_gains;
    vector<float> m_right_arm_gains;
    
    ofstream m_pattern_debug;

};

#endif

