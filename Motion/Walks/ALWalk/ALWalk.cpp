/*! @file JWalk.cpp
    @brief Implementation of jwalk class

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

#include "ALWalk.h"

#include "NUPlatform/NUSystem.h"
#include "NUPlatform/Platforms/NAO/NUNAO.h"
#include "debug.h"
#include "debugverbositynumotion.h"

#include <albroker.h>
#include <time.h>

ALWalk::ALWalk()
{   
    m_al_motion = new ALMotionProxy(NUNAO::m_broker);
    m_al_config.arrayReserve(10);
    m_al_param.arraySetSize(2);
    
    // turn the foot protection on
    m_al_param[0] = "ENABLE_FOOT_CONTACT_PROTECTION";
    m_al_param[1] = true;
    m_al_config.arrayPush(m_al_param);
    // turn the low stiffness protection off
    m_al_param[0] = "ENABLE_STIFFNESS_PROTECTION";
    m_al_param[1] = false;
    m_al_config.arrayPush(m_al_param);
    m_al_motion->setMotionConfig(m_al_config);
    
    // load and init the walk parameters
    m_walk_parameters.load("ALWalkAldebaran");
    initALConfig();
}

/*! @brief Destructor for motion module
 */
ALWalk::~ALWalk()
{
    if (m_al_motion != NULL)
        delete m_al_motion;
}

/*! @brief Freezes the aldebaran walk engine
 */
void ALWalk::freeze()
{
    NUWalk::freeze();
    m_al_motion->killWalk();
}

/*! @brief Kill the aldebaran walk engine
 */
void ALWalk::kill()
{
    freeze();
    m_al_motion->setStiffnesses(string("Body"), 0.0f);
}

void ALWalk::enableWalk()
{
    m_al_motion->setStiffnesses(string("Body"), 0.5f);
    NUWalk::enableWalk();
}

void ALWalk::doWalk()
{      
    // give the target speed to the walk engine
    vector<float>& maxspeeds = m_walk_parameters.getMaxSpeeds();
    m_al_motion->setWalkTargetVelocity(m_speed_x/maxspeeds[0], m_speed_y/maxspeeds[1], m_speed_yaw/maxspeeds[2], 1);
    
    // handle the joint stiffnesses
    static vector<float> legnan(m_actions->getNumberOfJoints(NUActionatorsData::LeftLegJoints), NAN);
    static vector<float> armnan(m_actions->getNumberOfJoints(NUActionatorsData::LeftArmJoints), NAN);
    m_actions->addJointPositions(NUActionatorsData::LeftLegJoints, m_data->CurrentTime, legnan, legnan, m_walk_parameters.getLegGains()[0]);
    m_actions->addJointPositions(NUActionatorsData::RightLegJoints, m_data->CurrentTime, legnan, legnan, m_walk_parameters.getLegGains()[0]);
    if (m_larm_enabled)
        m_actions->addJointPositions(NUActionatorsData::LeftArmJoints, m_data->CurrentTime, armnan, armnan, m_walk_parameters.getArmGains()[0]);
    if (m_rarm_enabled)
        m_actions->addJointPositions(NUActionatorsData::RightArmJoints, m_data->CurrentTime, armnan, armnan, m_walk_parameters.getArmGains()[0]);
}

/*! @brief Sets whether the arms are allowed to be moved by the walk engine
 */
void ALWalk::setArmEnabled(bool leftarm, bool rightarm)
{
    m_larm_enabled = leftarm;
    m_rarm_enabled = rightarm;
    m_al_motion->setWalkArmsEnable(leftarm, rightarm);
}

void ALWalk::initALConfig()
{
    m_al_param[1] = 0;
    m_al_config.clear();
    
    m_al_param[0] = "WALK_STEP_MIN_PERIOD";
    m_al_config.arrayPush(m_al_param);
    m_al_param[0] = "WALK_MAX_STEP_X";
    m_al_config.arrayPush(m_al_param);
    m_al_param[0] = "WALK_MAX_STEP_Y";
    m_al_config.arrayPush(m_al_param);
    m_al_param[0] = "WALK_MAX_STEP_THETA";
    m_al_config.arrayPush(m_al_param);
    m_al_param[0] = "WALK_MAX_STEP_HEIGHT";
    m_al_config.arrayPush(m_al_param);
    m_al_param[0] = "WALK_MIN_TRAPEZOID";
    m_al_config.arrayPush(m_al_param);
    m_al_param[0] = "WALK_FOOT_ORIENTATION";
    m_al_config.arrayPush(m_al_param);
    m_al_param[0] = "WALK_TORSO_ORIENTATION_Y";
    m_al_config.arrayPush(m_al_param);
    m_al_param[0] = "WALK_TORSO_HEIGHT";
    m_al_config.arrayPush(m_al_param);
    
    vector<WalkParameters::Parameter>& parameters = m_walk_parameters.getParameters();
    if (m_al_config.getSize() != parameters.size() + 3)
        errorlog << "ALConfig and WalkParameter size mismatch detected. ALConfig: " << m_al_config.getSize() << " parameters: " << parameters.size() << endl;
    setWalkParameters(m_walk_parameters);
}

void ALWalk::setALConfig()
{
    vector<WalkParameters::Parameter>& parameters = m_walk_parameters.getParameters();
    vector<float>& maxspeeds = m_walk_parameters.getMaxSpeeds();
    
    m_al_config[0][1] = static_cast<int>(1000/(20*parameters[0].Value));      // "WALK_STEP_MIN_PERIOD";
    
    m_al_config[1][1] = maxspeeds[0]/(100*parameters[0].Value);               // "WALK_MAX_STEP_X";
    m_al_config[2][1] = maxspeeds[1]/(100*parameters[0].Value);               // "WALK_MAX_STEP_Y";
    m_al_config[3][1] = 180*maxspeeds[2]/(3.141*parameters[0].Value);         // "WALK_MAX_STEP_THETA";
    
    m_al_config[4][1] = parameters[1].Value/100.0;                            // "WALK_MAX_STEP_HEIGHT";
    m_al_config[5][1] = 180*parameters[2].Value/3.141;                        // "WALK_MIN_TRAPEZOID";
    m_al_config[6][1] = 180*parameters[3].Value/3.141;                        // "WALK_FOOT_ORIENTATION";
    m_al_config[7][1] = 180*parameters[4].Value/3.141;                        // "WALK_TORSO_ORIENTATION_Y"
    m_al_config[8][1] = parameters[5].Value/100;                              // "WALK_TORSO_HEIGHT";
}

void ALWalk::setWalkParameters(const WalkParameters& walkparameters)
{
    NUWalk::setWalkParameters(walkparameters);
    setALConfig();
    m_al_motion->setMotionConfig(m_al_config);
}
