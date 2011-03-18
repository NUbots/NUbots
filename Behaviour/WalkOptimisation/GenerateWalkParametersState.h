/*! @file GenerateWalkParametersState.h
    @brief A state to generate a set of walk parameters to test
 
    @class GenerateWalkParametersState
    @brief A WalkOptimisation state to generate a set of walk parameters to test and to
           move to the start position for the evaluation.

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
#include "Tools/Math/StlVector.h"

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
    }
    
    ~GenerateWalkParametersState() {};
    BehaviourState* nextState()
    {   // progress to the evaluation state when we are in position AND lined up AND stopped
        vector<float> difference = m_field_objects->self.CalculateDifferenceFromFieldState(m_current_target_state);
        bool gettingup = false;
        m_data->get(NUSensorsData::MotionGetupActive, gettingup);

        if (difference[0] < 10 and fabs(difference[2]) < 0.2 and not gettingup)
        {
        	finish();
            return m_parent->m_evaluate;
        }
        else
            return this;
    }
    
    void doState()
    {
        #if DEBUG_BEHAVIOUR_VERBOSITY > 3
            debug << "GenerateWalkParametersState::doState" << endl;
        #endif
        m_previously_getting_up = m_getting_up;
        m_data->get(NUSensorsData::MotionGetupActive, m_getting_up);
        
        m_current_target_state = getStartPoint();
        if ((m_parent->stateChanged() and not m_parent->wasPreviousState(m_parent->m_paused)) or (not m_previously_getting_up and m_getting_up and m_time_not_getting_up > 500) or m_time_in_state > 45000)
        {
            m_parent->tickOptimiser();
            start();
            #if DEBUG_BEHAVIOUR_VERBOSITY > 2
                debug << "GenerateWalkParametersState::doState(). Start Position: " << m_current_target_state << endl;
            #endif
        }
        else
        	tick();

        if (m_getting_up)
            m_time_not_getting_up = 0;
        else
            m_time_not_getting_up += m_data->CurrentTime - m_data->PreviousTime;

        lookAtGoals();
        vector<float> speed = BehaviourPotentials::goToFieldState(m_field_objects->self, m_current_target_state, 0, 2*m_parent->stoppingDistance(), 9000);
        m_jobs->addMotionJob(new WalkJob(speed[0], speed[1], speed[2]));
		#if DEBUG_BEHAVIOUR_VERBOSITY > 4
            debug << "GenerateWalkParametersState::doState() - Completed." << endl;
        #endif
    }
private:
    bool m_previously_getting_up;
    bool m_getting_up;
    float m_time_not_getting_up;
};

#endif

