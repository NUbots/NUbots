/*! @file GenerateWalkParametersState.h
    @brief A state to generate a set of walk parameters to test
 
    @class GenerateWalkParametersState
    @brief A WalkOptimisation state to generate a set of walk parameters to test and to
           move to the start position for the evalutation

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

#ifndef GENERATEWALKPARAMETERSSTATE_H
#define GENERATEWALKPARAMETERSSTATE_H

#include "WalkOptimisationProvider.h"
#include "WalkOptimisationState.h"
#include "Tools/Optimisation/Optimiser.h"
#include "Tools/Math/General.h"
#include "Motion/Tools/MotionFileTools.h"

#include "Infrastructure/NUActionatorsData/NUActionatorsData.h"
#include "Infrastructure/FieldObjects/FieldObjects.h"
#include "Infrastructure/Jobs/JobList.h"

#include "Infrastructure/Jobs/MotionJobs/WalkJob.h"
#include "Infrastructure/Jobs/MotionJobs/HeadTrackJob.h"
#include "Infrastructure/Jobs/MotionJobs/HeadPanJob.h"
#include "Behaviour/BehaviourPotentials.h"

#include "debug.h"
#include "debugverbositybehaviour.h"

#include <vector>
using namespace std;

class GenerateWalkParametersState : public WalkOptimisationState
{
public:
    GenerateWalkParametersState(WalkOptimisationProvider* parent) : WalkOptimisationState(parent)
    {
        #if DEBUG_BEHAVIOUR_VERBOSITY > 3
            debug << "GenerateWalkParametersState::GenerateWalkParametersState" << endl;
        #endif
        m_getting_up = false;
        m_previously_getting_up = false;
        m_time_not_getting_up = 0;
        m_time_in_state = 0;
        m_current_start_state = vector<float>(3);
    }
    
    ~GenerateWalkParametersState() {};
    BehaviourState* nextState()
    {   // progress to the evaluation state when we are in position AND lined up AND stopped
        vector<float> difference = m_field_objects->self.CalculateDifferenceFromFieldState(m_current_start_state);
        vector<float> speed;
        m_data->get(NUSensorsData::MotionWalkSpeed, speed);
        if (difference[0] < 10 and fabs(difference[2]) < 0.2)
            return m_parent->m_evaluate;
        else
            return this;
    }
    
    void doState()
    {
        #if DEBUG_BEHAVIOUR_VERBOSITY > 3
            debug << "GenerateWalkParametersState" << endl;
        #endif
        m_previously_getting_up = m_getting_up;
        m_data->get(NUSensorsData::MotionGetupActive, m_getting_up);
        
        if ((m_parent->stateChanged() and not m_parent->wasPreviousState(m_parent->m_paused)) or (not m_previously_getting_up and m_getting_up and m_time_not_getting_up > 3000) or m_time_in_state > 120000)
        {
            m_parent->tickOptimiser();
            #if DEBUG_BEHAVIOUR_VERBOSITY > 2
                debug << "GenerateWalkParametersState::doState(). Start Position: " << MotionFileTools::fromVector(m_current_start_state) << endl;
            #endif
            m_time_in_state = 0;
        }
        else
            m_time_in_state += m_data->CurrentTime - m_previous_time;
        m_previous_time = m_data->CurrentTime;
        if (m_getting_up)
            m_time_not_getting_up = 0;
        else
            m_time_not_getting_up += m_data->CurrentTime - m_previous_time;
        
        lookAtGoals();
        
        m_current_start_state = getStartState();
        vector<float> speed = BehaviourPotentials::goToFieldState(m_field_objects->self, m_current_start_state, 0, 2*m_parent->stoppingDistance(), 9000);
        m_jobs->addMotionJob(new WalkJob(speed[0], speed[1], speed[2]));
    }
private:
    vector<float> getStartState()
    {   
        vector<vector<float> >& points = m_parent->m_speed_points;
        vector<float> front = points.front();
        vector<float> back = front;
        back[0] = -back[0];
        back[1] = -back[1];
        back[2] += 3.14;
        
        vector<float> difference_from_front = m_field_objects->self.CalculateDifferenceFromFieldState(front);
        vector<float> difference_from_back = m_field_objects->self.CalculateDifferenceFromFieldState(back);
        
        vector<float> startpoint;
        if (difference_from_back[0] < difference_from_front[0])
            startpoint = back;
        else
            startpoint = front;
        
        return startpoint;
    }
    
    void lookAtGoals()
    {
        StationaryObject& yellow_left = m_field_objects->stationaryFieldObjects[FieldObjects::FO_YELLOW_LEFT_GOALPOST];
        StationaryObject& yellow_right = m_field_objects->stationaryFieldObjects[FieldObjects::FO_YELLOW_RIGHT_GOALPOST];
        StationaryObject& blue_left = m_field_objects->stationaryFieldObjects[FieldObjects::FO_BLUE_LEFT_GOALPOST];
        StationaryObject& blue_right = m_field_objects->stationaryFieldObjects[FieldObjects::FO_BLUE_RIGHT_GOALPOST];
        
        if (yellow_left.isObjectVisible() and yellow_right.isObjectVisible())
        {
            float bearing = (yellow_left.ScreenXTheta() + yellow_right.ScreenXTheta())/2;
            float elevation = (yellow_left.ScreenYTheta() + yellow_right.ScreenYTheta())/2;
            m_jobs->addMotionJob(new HeadTrackJob(elevation, bearing));
        }
        else if (blue_left.isObjectVisible() and blue_right.isObjectVisible())
        {
            float bearing = (blue_left.ScreenXTheta() + blue_right.ScreenXTheta())/2;
            float elevation = (blue_left.ScreenYTheta() + blue_right.ScreenYTheta())/2;
            m_jobs->addMotionJob(new HeadTrackJob(elevation, bearing));
        }
        else if (blue_left.isObjectVisible() and blue_right.isObjectVisible())
        {
            float bearing = (blue_left.ScreenXTheta() + blue_right.ScreenXTheta())/2;
            float elevation = (blue_left.ScreenYTheta() + blue_right.ScreenYTheta())/2;
            m_jobs->addMotionJob(new HeadTrackJob(elevation, bearing));
        }
        else if (yellow_left.isObjectVisible() and yellow_right.TimeSinceLastSeen() > 500)
        {
            m_jobs->addMotionJob(new HeadTrackJob(yellow_left));
        }
        else if (yellow_right.isObjectVisible() and yellow_left.TimeSinceLastSeen() > 500)
        {
            m_jobs->addMotionJob(new HeadTrackJob(yellow_right));
        }        
        else if (blue_left.isObjectVisible() and blue_right.TimeSinceLastSeen() > 500)
        {
            m_jobs->addMotionJob(new HeadTrackJob(blue_left));
        }
        else if (blue_right.isObjectVisible() and blue_left.TimeSinceLastSeen() > 500)
        {
            m_jobs->addMotionJob(new HeadTrackJob(blue_right));
        }
        else if (yellow_left.TimeSinceLastSeen() > 500 and yellow_right.TimeSinceLastSeen() > 500 and blue_left.TimeSinceLastSeen() > 500 and blue_right.TimeSinceLastSeen() > 500)
            m_jobs->addMotionJob(new HeadPanJob(HeadPanJob::Localisation, 700, 9000, -0.5, 0.5));
    }
private:
    vector<float> m_current_start_state;
    
    bool m_previously_getting_up;
    bool m_getting_up;
    float m_time_not_getting_up;
    float m_time_in_state;
    float m_previous_time;
};

#endif

