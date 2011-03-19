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

QSBallisticController::QSBallisticController(const NUData::id_t& joint)
{
    #if DEBUG_BEHAVIOUR_VERBOSITY > 4
        debug << "QSBallisticController::QSBallisticController" << endl;
    #endif
    m_joint = joint;
    m_delay = new QSDelay(joint, this);
    m_relax = new QSRelax(joint, this);
    m_catch = new QSCatch(joint, this);
    
    m_initialised = false;
    
    m_state = m_delay;
}

QSBallisticController::~QSBallisticController()
{
    #if DEBUG_BEHAVIOUR_VERBOSITY > 0
        debug << "QSBallisticController::~QSBallisticController" << endl;
    #endif
    delete m_delay;
    delete m_relax;
    delete m_catch;
}

void QSBallisticController::doStateCommons()
{
    if (not m_initialised)
    {
        Blackboard->Sensors->getPosition(m_joint, m_position);
        m_velocity = 0;
        m_initialised = true;
    }
    else 
    {
        float beta = 0.003;                     // this amount of filtering does introduce a significant time delay in the estimate
        float alpha = 2*sqrt(beta/2) - beta/2;  // calculate alpha to give Kalman optimal damping (Painter, 1990)
        
        float meas_p;
        m_data->getPosition(m_joint, meas_p);
        double dt = (Blackboard->Sensors->CurrentTime - Blackboard->Sensors->PreviousTime)/1000;
        
        float pred_p = m_position + dt*m_velocity;
        m_position = pred_p + alpha*(meas_p - pred_p);
        m_velocity = m_velocity + (beta/dt)*(meas_p - pred_p);
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
    return m_relax->getTargetEstimate();
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



