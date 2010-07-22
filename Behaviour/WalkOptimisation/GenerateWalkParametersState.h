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
#include "Tools/Math/General.h"
#include "Motion/Tools/MotionFileTools.h"

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
    GenerateWalkParametersState(WalkOptimisationProvider* parent) : WalkOptimisationState(parent)
    {
        #if DEBUG_BEHAVIOUR_VERBOSITY > 1
            debug << "GenerateWalkParametersState::GenerateWalkParametersState" << endl;
        #endif
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
            vector<float>& parameters = m_parent->m_optimiser->nextParameters();
            m_current_start_state = getStartState();
            #if DEBUG_BEHAVIOUR_VERBOSITY > 0
                debug << "GenerateWalkParametersState::doState(). Next parameters: " << MotionFileTools::fromVector(parameters) << endl;
                debug << "GenerateWalkParametersState::doState(). Start Position: " << MotionFileTools::fromVector(m_current_start_state) << endl;
            #endif
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
        
        vector<float> speed = BehaviourPotentials::goToFieldState(m_field_objects->self, m_current_start_state, 5, 50, 9000);
        m_jobs->addMotionJob(new WalkJob(speed[0], speed[1], speed[2]));
    }
private:
    vector<float>& getStartState()
    {   // we are only ever going to go back and forth between two states.
        // you want to pick the one we are not at.
        if (m_current_start_state.size() == 0)
            return m_parent->m_points.front();
        else if (mathGeneral::allEqual(m_current_start_state, m_parent->m_points.front()))
            return m_parent->m_points.back();
        else
            return m_parent->m_points.front();
    }
private:
    vector<float> m_current_start_state;
};

#endif

