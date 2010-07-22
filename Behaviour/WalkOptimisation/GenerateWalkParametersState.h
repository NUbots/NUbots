/*! @file GenerateWalkParametersState.h
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

#ifndef GENERATEWALKPARAMETERSSTATE_H
#define GENERATEWALKPARAMETERSSTATE_H

#include "WalkOptimisationProvider.h"
#include "WalkOptimisationState.h"
#include "Tools/Optimisation/Optimiser.h"

#include "NUPlatform/NUActionators/NUActionatorsData.h"
#include "Vision/FieldObjects/FieldObjects.h"
#include "Behaviour/Jobs/JobList.h"

#include "Behaviour/Jobs/MotionJobs/WalkJob.h"
#include "Behaviour/Jobs/MotionJobs/HeadTrackJob.h"
#include "Behaviour/Jobs/MotionJobs/HeadPanJob.h"
#include "Behaviour/BehaviourPotentials.h"

#include "debug.h"
#include "debugverbositybehaviour.h"

#include <vector>
using namespace std;

class GenerateWalkParametersState : public WalkOptimisationState
{
public:
    GenerateWalkParametersState(WalkOptimisationProvider* parent) : WalkOptimisationState(parent), m_X_DISTANCE(300), m_Y_DISTANCE(140)
    {
        // initialise the starting position for the evaluation of new walk parameters
        vector<float> state(3,0);
        state[0] = m_X_DISTANCE/2.0;
        state[1] = m_Y_DISTANCE/2.0;
        state[2] = -3.1416;
        m_start_states.push_back(state);
        state[0] = -m_X_DISTANCE/2.0;
        state[1] = m_Y_DISTANCE/2.0;
        state[2] = 0;
        m_start_state_index = 0;
        m_start_states.push_back(state);
        m_current_start_state = m_start_states[0];
    }
    
    ~GenerateWalkParametersState() {};
    BehaviourState* nextState()
    {   // progress to the evaluation state when we are in position AND lined up
        vector<float> difference = m_field_objects->self.CalculateDifferenceFromFieldState(m_current_start_state);
        if (difference[0] < 5 and fabs(difference[2]) < 0.2)
            return m_parent->m_evaluate;
        else
            return this;
    }
    
    void doState()
    {
        #if DEBUG_BEHAVIOUR_VERBOSITY > 1
            debug << "GenerateWalkParametersState" << endl;
        #endif
        if (m_parent->stateChanged())
        {
            #if DEBUG_BEHAVIOUR_VERBOSITY > 0
                debug << "GenerateWalkParametersState. Generating next set of walk parameters" << endl;
            #endif
            vector<float>& parameters = m_parent->m_optimiser->nextParameters();
            m_current_start_state = getStartState();
            //!< TODO: make a walk parameters job and give it to the walk engine
        }
        
        // handle the head movements: For now we look at the yellow or blue goal if we can see them, otherwise pan
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
        else if (yellow_left.TimeSinceLastSeen() > 500 and yellow_right.TimeSinceLastSeen() > 500 and blue_left.TimeSinceLastSeen() > 500 and blue_right.TimeSinceLastSeen() > 500)
            m_jobs->addMotionJob(new HeadPanJob(HeadPanJob::Localisation));
        
        vector<float> speed = BehaviourPotentials::goToFieldState(m_field_objects->self, m_current_start_state, 5, 50, sqrt(pow(m_X_DISTANCE,2) + pow(m_Y_DISTANCE,2)));
        m_jobs->addMotionJob(new WalkJob(speed[0], speed[1], speed[2]));
    }
private:
    vector<float>& getStartState()
    {
        #if DEBUG_BEHAVIOUR_VERBOSITY > 0
            debug << "GenerateWalkParametersState::getStartState = [" << m_start_states[m_start_state_index][0] << "," << m_start_states[m_start_state_index][1] << "," << m_start_states[m_start_state_index][2] << "]" << endl;
        #endif
        m_start_state_index++;
        if (m_start_state_index >= m_start_states.size())
            m_start_state_index = 0;
        
        return m_start_states[m_start_state_index];
    }
private:
    const float m_X_DISTANCE;
    const float m_Y_DISTANCE;
    unsigned int m_start_state_index;
    vector<vector<float> > m_start_states;
    vector<float> m_current_start_state;
};

#endif

