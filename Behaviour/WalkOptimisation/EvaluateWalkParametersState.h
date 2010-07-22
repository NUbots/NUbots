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

#include "Motion/Tools/MotionFileTools.h"

#include "debug.h"
#include "debugverbositybehaviour.h"
#include "nubotdataconfig.h"

class EvaluateWalkParametersState : public WalkOptimisationState
{
public:
    EvaluateWalkParametersState(WalkOptimisationProvider* parent) : WalkOptimisationState(parent)
    {
        #if DEBUG_BEHAVIOUR_VERBOSITY > 1
            debug << "EvaluateWalkParametersState::EvaluateWalkParametersState" << endl;
        #endif
    }
    virtual ~EvaluateWalkParametersState() {};
    virtual BehaviourState* nextState() 
    {
        if (allPointsReached())
            return m_parent->m_generate;
        else
            return this;
    }
    virtual void doState()
    {
        #if DEBUG_BEHAVIOUR_VERBOSITY > 1
            debug << "EvaluateWalkParametersState::doState()" << endl;
        #endif
        if (m_parent->stateChanged())
        {
            m_current_target_state = getStartPoint();
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
            m_jobs->addMotionJob(new HeadPanJob(HeadPanJob::Localisation, 700, 9000, -0.5, 0.5));
        
        if (pointReached())
            m_current_target_state = getNextPoint();
        
        vector<float> speed = BehaviourPotentials::goToFieldState(m_field_objects->self, m_current_target_state, 0, 0, 0);
        m_jobs->addMotionJob(new WalkJob(speed[0], speed[1], speed[2]));
    }
private:
    vector<float>& getStartPoint()
    {
        vector<vector<float> >& points = m_parent->m_points;
        float distance_from_first = m_field_objects->self.CalculateDifferenceFromFieldState(points[0])[0];
        float distance_from_last = m_field_objects->self.CalculateDifferenceFromFieldState(points[points.size()-1])[0];
        if (distance_from_first < distance_from_last)
        {
            m_reverse_points = false;
            m_current_point_index = 0;
        }
        else
        {
            m_reverse_points = true;
            m_current_point_index = points.size()-1;
        }
        #if DEBUG_BEHAVIOUR_VERBOSITY > 0
            debug << "EvaluateWalkParametersState. Start point " << MotionFileTools::fromVector(points[m_current_point_index]) << endl;
        #endif
        return points[m_current_point_index];
    }
    bool pointReached()
    {
        vector<float> difference = m_field_objects->self.CalculateDifferenceFromFieldState(m_current_target_state);
        if (difference[0] < 10)
            return true;
        else
            return false;
    }
    vector<float>& getNextPoint()
    {
        if (not m_reverse_points)
        {
            if (m_current_point_index < m_parent->m_points.size()-1)
                m_current_point_index++;
        }
        else
        {
            if (m_current_point_index > 0)
                m_current_point_index--;
        }
        #if DEBUG_BEHAVIOUR_VERBOSITY > 0
            debug << "EvaluateWalkParametersState. Next point " << MotionFileTools::fromVector(m_parent->m_points[m_current_point_index]) << endl;
        #endif
        return m_parent->m_points[m_current_point_index];
    }
    bool allPointsReached()
    {
        if (not pointReached())
            return false;
        else
        {
            if (not m_reverse_points and m_current_point_index == m_parent->m_points.size()-1)
                return true;
            else if (m_reverse_points and m_current_point_index == 0)
                return true;
            else
                return false;
        }
    }
private:
    vector<float> m_current_target_state;
    bool m_reverse_points;
    unsigned int m_current_point_index;
};

#endif

