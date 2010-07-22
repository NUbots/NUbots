/*! @file EvaluateWalkParametersState.h
    @brief A state to evaluate a set of walk parameters

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

#ifndef EVALUATEWALKPARAMETERSSTATE_H
#define EVALUATEWALKPARAMETERSSTATE_H

#include "WalkOptimisationProvider.h"
#include "WalkOptimisationState.h"

#include "Behaviour/Jobs/JobList.h"
#include "NUPlatform/NUActionators/NUActionatorsData.h"

#include "Behaviour/Jobs/MotionJobs/WalkJob.h"
#include "Behaviour/Jobs/MotionJobs/HeadJob.h"

#include "debug.h"

class EvaluateWalkParametersState : public WalkOptimisationState
{
public:
    EvaluateWalkParametersState(WalkOptimisationProvider* parent) : WalkOptimisationState(parent), m_X_DISTANCE(300), m_Y_DISTANCE(140) 
    {
        // initialise the targets to chase while evaluating a set of walk parameters
        vector<float> state(3,0);
        state[0] = -m_X_DISTANCE/2.0;
        state[1] = m_Y_DISTANCE/2.0;
        state[2] = 0;
        m_target_states.push_back(state);
        state[0] = m_X_DISTANCE/2.0;
        state[1] = m_Y_DISTANCE/2.0;
        state[2] = 0;
        m_target_states.push_back(state);
        m_target_state_index = 1;
        m_current_target_state = m_target_states[1];
    }
    virtual ~EvaluateWalkParametersState() {};
    virtual BehaviourState* nextState() 
    {
        bool targetbehind = fabs(m_field_objects->self.CalculateDifferenceFromFieldState(m_current_target_state)[1]) > 1.571;
        if (targetbehind)
            return m_parent->m_generate;
        else
            return this;
    }
    virtual void doState()
    {
        #if DEBUG_BEHAVIOUR_VERBOSITY > 1
            debug << "EvaluateWalkParametersState" << endl;
        #endif
        if (m_parent->stateChanged())
        {
            //!< TODO: make a walk parameters job and give it to the walk engine
            m_current_target_state = getTargetState();
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
        
        vector<float> difference = m_field_objects->self.CalculateDifferenceFromFieldState(m_current_target_state);
        if (fabs(difference[1]) > 1.571)
            m_jobs->addMotionJob(new WalkJob(0,0,0));
        else
        {
            vector<float> speed = BehaviourPotentials::goToFieldState(m_field_objects->self, m_current_target_state, 0, 0, 0);
            m_jobs->addMotionJob(new WalkJob(speed[0], speed[1], speed[2]));
        }
    }
private:
    vector<float>& getTargetState()
    {
        m_target_state_index++;
        if (m_target_state_index >= m_target_states.size())
            m_target_state_index = 0;
        
        return m_target_states[m_target_state_index];
    }
private:
    const float m_X_DISTANCE;
    const float m_Y_DISTANCE;
    unsigned int m_target_state_index;
    vector<vector<float> > m_target_states;
    vector<float> m_current_target_state;
};

#endif

