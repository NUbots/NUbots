/*! @file QSRelax.cpp
    @brief Implementation of behaviour state class

    @author Jason Kulk
 
 Copyright (c) 2010 Jason Kulk
 
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

#include "QSRelax.h"
#include "QSCatch.h"
#include "QSBallisticController.h"

#include "Infrastructure/NUBlackboard.h"
#include "Infrastructure/NUSensorsData/NUSensorsData.h"
#include "Infrastructure/NUActionatorsData/NUActionatorsData.h"

#include "debug.h"
#include "debugverbositybehaviour.h"

#include <cmath>

QSRelax::QSRelax(const NUData::id_t& joint, const QSBallisticController* parent)
{
    #if DEBUG_BEHAVIOUR_VERBOSITY > 4
        debug << "QSRelax::QSRelax" << endl;
    #endif
    m_joint = joint;
    m_parent = parent;
    
    m_time_in_state = 0;
    m_previous_time = 0;
    
    if (m_joint.isLeft())
        m_target_calibration = 0.053;
    else
        m_target_calibration = 0.026;
}

QSRelax::~QSRelax()
{
}

void QSRelax::doState()
{
    #if DEBUG_BEHAVIOUR_VERBOSITY > 0
        debug << "QSRelax::doState" << endl;
    #endif
    float target = -0.025 + m_parent->getTargetEstimate();
    
    float gain = 0.07;              // with a slope of 5, I can't balance the robot without control with gain of 0.05;
    float meas_p, output;
    m_data->getPosition(m_joint, meas_p);
    output = gain*target - (gain-1)*meas_p;     
    m_actions->add(m_joint, 0, output, 100);
    
    if (m_previous_time != 0)
        m_time_in_state += m_data->CurrentTime - m_previous_time;

    m_previous_time = m_data->CurrentTime;
}

BehaviourState* QSRelax::nextState()
{
    #if DEBUG_BEHAVIOUR_VERBOSITY > 0
        debug << "QSRelax::nextState" << endl;
    #endif
    // I am fixing the time_in_state to be 0.64*catch_duration
    if (m_time_in_state > 400 and fabs(m_parent->getVelocity()) > QSBallisticController::VelocityThreshold)
    {
        m_previous_time = 0;
        m_time_in_state = 0;
        return m_parent->getCatch();
    }
    else
        return this;
}



