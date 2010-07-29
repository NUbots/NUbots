/*! @file EvaluateSpeedOfWalkParameters.cpp
    @brief A state to evaluate the speed and the efficiency of a set of walk parameters
 
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

#include "EvaluateSpeedOfWalkParametersState.h"
#include "WalkOptimisationProvider.h"
#include "EvaluateWalkParametersState.h"

#include "NUPlatform/NUActionators/NUActionatorsData.h"
#include "Vision/FieldObjects/FieldObjects.h"
#include "Behaviour/Jobs/JobList.h"
#include "Behaviour/Jobs/MotionJobs/WalkJob.h"
#include "Behaviour/Jobs/MotionJobs/HeadTrackJob.h"
#include "Behaviour/Jobs/MotionJobs/HeadPanJob.h"
#include "Behaviour/BehaviourPotentials.h"

#include "Motion/Tools/MotionFileTools.h"

#include "debug.h"
#include "debugverbositybehaviour.h"

/*! @brief Construct a evaluate speed of walk parameters state
    @param parent the parent EvaluateWalkParametersState
 */
EvaluateSpeedOfWalkParametersState::EvaluateSpeedOfWalkParametersState(EvaluateWalkParametersState* parent)
{
    #if DEBUG_BEHAVIOUR_VERBOSITY > 1
        debug << "EvaluateSpeedOfWalkParametersState::EvaluateSpeedOfWalkParametersState" << endl;
    #endif
    m_parent = parent;
    m_provider = parent->m_parent;
    m_points = m_provider->m_speed_points;
}

/*! @brief Returns the desired next state in the EvaluateWalkParameters state machine 
           The machine progresses to the stability evaluation once the speed path has been completed
 */
BehaviourState* EvaluateSpeedOfWalkParametersState::nextState() 
{   // when all of the points have been reached we progress to the stability evaluation
    if (allPointsReached())
    {
        finaliseEvaluation();
        return m_parent->m_evaluate_stability;
    }
    else
        return this;
}

/*! @brief Do the state behaviour */
void EvaluateSpeedOfWalkParametersState::doState()
{
    #if DEBUG_BEHAVIOUR_VERBOSITY > 1
        debug << "EvaluateSpeedOfWalkParametersState::doState()" << endl;
    #endif
    if (m_parent->stateChanged() or m_provider->stateChanged())
        m_current_target_state = getStartPoint();
    
    updateEvaluation();

    lookAtGoals();
    
    if (pointReached())
        m_current_target_state = getNextPoint();
    
    vector<float> speed = BehaviourPotentials::goToFieldState(m_field_objects->self, m_current_target_state, 0, 0, 9000);
    m_jobs->addMotionJob(new WalkJob(speed[0], speed[1], speed[2]));
}

/*! @brief Updates the evaluation */
void EvaluateSpeedOfWalkParametersState::updateEvaluation()
{
    updateSpeedEvaluation();
    updateEfficiencyEvaluation();
}

/*! @brief Updates the speed evaluation */
void EvaluateSpeedOfWalkParametersState::updateSpeedEvaluation()
{   // the speed calculation is distance/time, but i have no measure of distance travelled
    // distance is a constant, so the smaller the time the better
    // however, until the trial is finished the longer the time the better
    
}

/*! @brief Updates the efficiency evaluation */
void EvaluateSpeedOfWalkParametersState::updateEfficiencyEvaluation()
{   // the efficiency
    
}

/*! @brief Finalise the evaluation */
void EvaluateSpeedOfWalkParametersState::finaliseEvaluation()
{
    #if DEBUG_BEHAVIOUR_VERBOSITY > 0
        debug << "EvaluateSpeedOfWalkParametersState::finaliseEvaluation()" << endl;
    #endif
}
    
/*! @brief Returns the starting point for the evaluation of the speed */
vector<float> EvaluateSpeedOfWalkParametersState::getStartPoint()
{
    float distance_from_first = m_field_objects->self.CalculateDifferenceFromFieldState(m_points.front())[0];
    float distance_from_last = m_field_objects->self.CalculateDifferenceFromFieldState(m_points.back())[0];
    if (distance_from_first < distance_from_last)
    {
        m_reverse_points = false;
        m_current_point_index = 0;
    }
    else
    {
        m_reverse_points = true;
        m_current_point_index = m_points.size()-1;
    }
    #if DEBUG_BEHAVIOUR_VERBOSITY > 0
        debug << "EvaluateSpeedOfWalkParametersState. Start point " << MotionFileTools::fromVector(m_points[m_current_point_index]) << endl;
    #endif
    if (not m_reverse_points)
        return m_points[m_current_point_index];
    else
    {
        vector<float> point = m_points[m_current_point_index];
        point[2] += 3.1416;         // need to reverse the heading when going backwards
        return point;
    }
}
    
/*! @brief Returns true if the current point has been reached */
bool EvaluateSpeedOfWalkParametersState::pointReached()
{
    vector<float> difference = m_field_objects->self.CalculateDifferenceFromFieldState(m_current_target_state);
    if (difference[0] < 10)
        return true;
    else
        return false;
}
    
/*! @brief Returns the next point in the list of way points in the predefined path */
vector<float> EvaluateSpeedOfWalkParametersState::getNextPoint()
{
    if (not m_reverse_points)
    {
        if (m_current_point_index < m_points.size()-1)
            m_current_point_index++;
    }
    else
    {
        if (m_current_point_index > 0)
            m_current_point_index--;
    }
    #if DEBUG_BEHAVIOUR_VERBOSITY > 0
        debug << "EvaluateSpeedOfWalkParametersState. Next point " << MotionFileTools::fromVector(m_points[m_current_point_index]) << endl;
    #endif
    if (not m_reverse_points)
        return m_points[m_current_point_index];
    else
    {
        vector<float> point = m_points[m_current_point_index];
        point[2] += 3.1416;         // need to reverse the heading when going backwards
        return point;
    }
}
    
/*! @brief Returns true if the trial path has been completed */
bool EvaluateSpeedOfWalkParametersState::allPointsReached()
{
    if (not pointReached())
        return false;
    else
    {
        if (not m_reverse_points and m_current_point_index == m_points.size()-1)
            return true;
        else if (m_reverse_points and m_current_point_index == 0)
            return true;
        else
            return false;
    }
}
    
/*! @brief Moves the head to look at the goals */
void EvaluateSpeedOfWalkParametersState::lookAtGoals()
{
    StationaryObject& yellow_left = m_field_objects->stationaryFieldObjects[FieldObjects::FO_YELLOW_LEFT_GOALPOST];
    StationaryObject& yellow_right = m_field_objects->stationaryFieldObjects[FieldObjects::FO_YELLOW_RIGHT_GOALPOST];
    StationaryObject& blue_left = m_field_objects->stationaryFieldObjects[FieldObjects::FO_BLUE_LEFT_GOALPOST];
    StationaryObject& blue_right = m_field_objects->stationaryFieldObjects[FieldObjects::FO_BLUE_RIGHT_GOALPOST];
    AmbiguousObject yellow_unknown;
    AmbiguousObject blue_unknown;
    for (size_t i=0; i<m_field_objects->ambiguousFieldObjects.size(); i++)
    {
        int ambig_id = m_field_objects->ambiguousFieldObjects[i].getID();
        if (ambig_id == FieldObjects::FO_YELLOW_GOALPOST_UNKNOWN)
            yellow_unknown = m_field_objects->ambiguousFieldObjects[i];
        else if (ambig_id == FieldObjects::FO_BLUE_GOALPOST_UNKNOWN)
            blue_unknown = m_field_objects->ambiguousFieldObjects[i];
    }
    
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
    else if (yellow_unknown.getID() > 0 and yellow_left.TimeSinceLastSeen() > 500 and yellow_right.TimeSinceLastSeen() > 500)
    {
        m_jobs->addMotionJob(new HeadTrackJob(yellow_unknown));
    }
    else if (blue_unknown.getID() > 0 and blue_left.TimeSinceLastSeen() > 500 and blue_right.TimeSinceLastSeen() > 500)
    {
        m_jobs->addMotionJob(new HeadTrackJob(blue_unknown));
    }
    else if (yellow_left.TimeSinceLastSeen() > 500 and yellow_right.TimeSinceLastSeen() > 500 and blue_left.TimeSinceLastSeen() > 500 and blue_right.TimeSinceLastSeen() > 500)
        m_jobs->addMotionJob(new HeadPanJob(HeadPanJob::Localisation, 700, 9000, -0.75, 0.75));
}


