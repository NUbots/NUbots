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
#include <numeric>

QSRelax::QSRelax(const NUData::id_t& joint, const QSBallisticController* parent)
{
    #if DEBUG_BEHAVIOUR_VERBOSITY > 4
        debug << "QSRelax::QSRelax" << endl;
    #endif
    m_joint = joint;
    m_parent = parent;
    
    m_time_in_state = 0;
    m_previous_time = 0;
    
    m_target_estimate = 0;
    m_target_estimate_buffer = boost::circular_buffer<float>(10);
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
    float target = m_target_estimate;
    
    float gain = 0.1;              // with a slope of 5, I can't balance the robot without control with gain of 0.05;
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
    if (m_time_in_state > 180 and fabs(m_parent->getVelocity()) > QSBallisticController::VelocityThreshold)
    {
        m_previous_time = 0;
        m_time_in_state = 0;
        updateTargetEstimate();
        return m_parent->getCatch();
    }
    else
        return this;
}

/*! @brief Returns the current estimate of where the control variable needs to be to produce zero velocity */
float QSRelax::getTargetEstimate()
{
    return m_target_estimate;
}

/*! @brief Update the target esimtate */
void QSRelax::updateTargetEstimate()
{
    float offset = -0.02;           // the offset applied to the target to compensate for the slack in the joints (-0.02)
    float velocitygain = -2.0;     // a constant to control how much the velocity is used to estimate the target
    float currentestimate = m_parent->getPosition() + velocitygain*m_parent->getVelocity() + m_target_calibration + offset;
    m_target_estimate_buffer.push_back(currentestimate);
    
    m_target_estimate = accumulate(m_target_estimate_buffer.begin(), m_target_estimate_buffer.end(), 0.0f)/10;
}

