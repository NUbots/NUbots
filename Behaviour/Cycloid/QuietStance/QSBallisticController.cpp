/*! @file QSBallisticController.cpp
    @brief A simple ballistic controller
 
    @author Jason Kulk
 
  Copyright (c) 2011 Jason Kulk
 
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

#include "QSBallisticController.h"
#include "QSDelay.h"
#include "QSRelax.h"
#include "QSCatch.h"

#include "Infrastructure/NUBlackboard.h"
#include "Infrastructure/NUSensorsData/NUSensorsData.h"
#include "Infrastructure/NUActionatorsData/NUActionatorsData.h"

#include "debug.h"
#include "debugverbositybehaviour.h"

#include <cmath>
#include <numeric>

QSBallisticController::QSBallisticController(const NUData::id_t& joint)
{
    #if DEBUG_BEHAVIOUR_VERBOSITY > 4
        debug << "QSBallisticController::QSBallisticController" << std::endl;
    #endif
    m_joint = joint;
    m_delay = new QSDelay(joint, this);
    m_relax = new QSRelax(joint, this);
    m_catch = new QSCatch(joint, this);
    
    m_initialised = false;
    
    m_target_estimate = 0;
    m_target_estimate_buffer = boost::circular_buffer<float>(600, 0);        // assume the pitch starts at zero
    
    m_state = m_delay;
}

QSBallisticController::~QSBallisticController()
{
    #if DEBUG_BEHAVIOUR_VERBOSITY > 0
        debug << "QSBallisticController::~QSBallisticController" << std::endl;
    #endif
    delete m_delay;
    delete m_relax;
    delete m_catch;
}

void QSBallisticController::doStateCommons()
{
    if (not m_initialised)
    {   // we need a special initialisation so the m_position is set to the proper value the first time
        Blackboard->Sensors->getPosition(m_joint, m_position);
        m_velocity = 0;
        m_acceleration = 0;
        m_initialised = true;
    }
    else 
    {
        // so im going to need to use an alpha-beta-gamma filter because I need the acceleration too :(
        float beta = 0.015;
        float alpha = 2*sqrt(beta/2) - beta/2;              // coefficents taken from (Arcasoy, 1997) (Painter, 1990)
        float gamma = 0.3*alpha*beta/(2-alpha);              
        
        // Measure the position
        float measuredposition;
        m_data->getPosition(m_joint, measuredposition);
        double dt = (Blackboard->Sensors->CurrentTime - Blackboard->Sensors->PreviousTime)/1000;
        
        // Do the prediction
        float predictedposition = m_position + m_velocity*dt + 0.5*m_acceleration*pow(dt,2);
        float predictedvelocity = m_velocity + m_acceleration*dt;
        float predictedacceleration = m_acceleration;
        
        // Update the estimates
        float change = measuredposition - predictedposition;
        m_position = predictedposition + alpha*change;
        m_velocity = predictedvelocity + (beta/dt)*change;
        m_acceleration = predictedacceleration + (gamma/(2*pow(dt,2)))*change;
        
        updateTargetEstimate();
    }
}

/*! @brief Returns the relax state */
QSRelax* QSBallisticController::getRelax() const
{
    return m_relax;
}

/*! @brief Returns the catch state */
QSCatch* QSBallisticController::getCatch() const
{
    return m_catch;
}

/*! @brief Returns true if the controller is on the relax state, false otherwise */
bool QSBallisticController::relaxed() const
{
    return m_state == m_relax;
}

/*! @brief Returns the current estimate of where the controller needs to be to produce zero velocity */
float QSBallisticController::getTargetEstimate() const
{
    return 0;
    return m_target_estimate;
}

/*! @brief Returns the filtered angular position */
float QSBallisticController::getPosition() const
{
    return m_position;
}

/*! @brief Returns the filtered angular velocity */
float QSBallisticController::getVelocity() const
{
    return m_velocity;
}

/*! @brief Returns the filtered angular acceleration */
float QSBallisticController::getAcceleration() const
{
    return m_acceleration;
}

/*! @brief Returns the current guess of the torque */
float QSBallisticController::getTorque() const
{
    float torque;
    if (m_data->getTorque(m_joint, torque))
        return torque;
    else 
        return 0;
}

/*! @brief Update the target esimtate */
void QSBallisticController::updateTargetEstimate()
{
    float m = QSBallisticController::Mass;
    float g = 9.81;
    float h = QSBallisticController::Height;
    float o = -0.02;
    float b = QSBallisticController::FrictionConstant;
    float K = 1.0;          // K is a constant to compensate for calculation errors in the torque.
    
    float u, v, a, t;
    u = getPosition();
    v = getVelocity();
    a = getAcceleration();
    t = getTorque();
    
    float currentestimate = u - asin((m*pow(h,2)*a + b*v - K*t)/(m*g*h));
    m_target_estimate_buffer.push_back(currentestimate);
    
    m_target_estimate = accumulate(m_target_estimate_buffer.begin(), m_target_estimate_buffer.end(), 0.0f)/m_target_estimate_buffer.size();
}

