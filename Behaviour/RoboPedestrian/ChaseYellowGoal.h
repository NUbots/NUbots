/*! @file ChaseYellowGoal.h
    @brief A state to chase a field object

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

#ifndef CHASEYELLOWGOAL_H
#define CHASEYELLOWGOAL_H

#include "RoboPedestrianProvider.h"
#include "ChaseObject.h"

#include "Infrastructure/Jobs/JobList.h"
#include "Infrastructure/NUSensorsData/NUSensorsData.h"
#include "Infrastructure/FieldObjects/FieldObjects.h"

#include "debug.h"

class ChaseYellowGoal : public ChaseObject
{
public:
    ChaseYellowGoal(RoboPedestrianProvider* parent) : ChaseObject(parent, FieldObjects::FO_YELLOW_LEFT_GOALPOST, FieldObjects::FO_YELLOW_GOALPOST_UNKNOWN), m_goal_reached_count(0) {};
    virtual ~ChaseYellowGoal() {};
    virtual BehaviourState* nextState()
    {
        updateTarget();
        if (m_target.TimeSinceLastSeen() > 5000)
        {
            debug << "ChaseYellowGoal -> SearchYellowGoal. Goal lost." << std::endl;
            m_goal_reached_count = 0;
            return m_parent->m_search_yellow_goal;
        }
        else if (m_target.measuredDistance() < 100)
        {
            m_goal_reached_count++;
            if (m_goal_reached_count > 10)
            {
                m_goal_reached_count = 0;
                debug << "ChaseYellowGoal -> SearchBlueGoal. Goal reached." << std::endl;
                return m_parent->m_search_blue_goal;
            }
        }
        return this;
    };
private:
    int m_goal_reached_count;
};

#endif

