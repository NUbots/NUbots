/*! @file ChaseBlueGoal.h
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

#ifndef CHASEBLUEGOAL_H
#define CHASEBLUEGOAL_H

#include "WalkOptimisationProvider.h"
#include "ChaseObject.h"

#include "Behaviour/Jobs/JobList.h"
#include "NUPlatform/NUSensors/NUSensorsData.h"
#include "Vision/FieldObjects/FieldObjects.h"

#include "debug.h"

class ChaseBlueGoal : public ChaseObject
{
public:
    ChaseBlueGoal(WalkOptimisationProvider* parent) : ChaseObject(parent, FieldObjects::FO_BLUE_RIGHT_GOALPOST, FieldObjects::FO_BLUE_GOALPOST_UNKNOWN), m_goal_reached_count(0) {};
    virtual ~ChaseBlueGoal() {};
    virtual BehaviourState* nextState()
    {
        updateTarget();
        if (m_target.TimeSinceLastSeen() > 1000)
        {
            debug << "ChaseBlueGoal -> SearchBlueGoal. Goal lost" << endl;
            m_goal_reached_count = 0;
            return m_parent->m_search_blue_goal;
        }
        else if (m_target.measuredDistance() < 150)
        {
            m_goal_reached_count++;
            if (m_goal_reached_count > 10)
            {
                debug << "ChaseYellowGoal -> SearchYellowGoal. Goal Reached" << endl;
                m_goal_reached_count = 0;
                return m_parent->m_search_yellow_goal;
            }
        }
        return this;
    };
private:
    int m_goal_reached_count;
};

#endif

