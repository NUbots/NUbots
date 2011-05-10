/*! @file QSDelay.h
    @brief Declaration of an extremely simple state which does NOTHING for a given period
 
    @class QSDelay
    @brief The delay state is just used to freeze the controller for a fixed period. In particular, during initialisation

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

#ifndef QSDelay_H
#define QSDelay_H

#include "Behaviour/BehaviourState.h"
#include "Infrastructure/NUSensorsData/NUSensorsData.h"

#include "QSRelax.h"
class QSBallisticController;

class QSDelay : public BehaviourState
{
public:
    QSDelay(const NUData::id_t& joint, const QSBallisticController* parent)
    {
        m_parent = parent;
        m_start_time = 0;
    };
    ~QSDelay() {};
    
protected:
    void doState()
    {
        if (m_start_time == 0)
            m_start_time = m_data->CurrentTime;
    };
    BehaviourState* nextState()
    {
        if (m_start_time != 0 and m_data->CurrentTime - m_start_time > 5000)
        {
            m_start_time = 0;
            return m_parent->getRelax();
        }
        else
            return this;
    };
private:
    const QSBallisticController* m_parent;
    double m_start_time;
};


#endif

