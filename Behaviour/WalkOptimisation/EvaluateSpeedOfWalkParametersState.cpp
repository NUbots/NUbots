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
    #if DEBUG_BEHAVIOUR_VERBOSITY > 3
        debug << "EvaluateSpeedOfWalkParametersState::EvaluateSpeedOfWalkParametersState" << endl;
    #endif
    m_parent = parent;
    m_provider = parent->m_parent;
    m_points = m_provider->m_speed_points;
    
    m_current_target_state = vector<float>(3,0);
}

/*! @brief Returns the desired next state in the EvaluateWalkParameters state machine 
           The machine progresses to the stability evaluation once the speed path has been completed
 */
BehaviourState* EvaluateSpeedOfWalkParametersState::nextState() 
{   // when all of the points have been reached we progress to the stability evaluation
    if (allPointsReached() or m_data->CurrentTime - m_trial_start_time > 180000)
    {
        finishEvaluation();
        return m_parent->m_evaluate_stability;
    }
    else
        return this;
}

/*! @brief Do the state behaviour */
void EvaluateSpeedOfWalkParametersState::doState()
{
    #if DEBUG_BEHAVIOUR_VERBOSITY > 3
        debug << "EvaluateSpeedOfWalkParametersState::doState() target:[" << m_current_target_state[0] << "," << m_current_target_state[1] << "," << m_current_target_state[2] << "]" << endl;
    #endif
    if (m_parent->stateChanged() or m_provider->stateChanged())
        startEvaluation();
    
    tickEvaluation();

    lookAtGoals();
    
    if (pointReached())
        m_current_target_state = getNextPoint();
    
    vector<float> speed = BehaviourPotentials::goToFieldState(m_field_objects->self, m_current_target_state, 0, m_provider->stoppingDistance(), 9000);
    m_jobs->addMotionJob(new WalkJob(speed[0], speed[1], speed[2]));
}

/*! @brief Starts the evaluation, ie resets all of the measurement variables */
void EvaluateSpeedOfWalkParametersState::startEvaluation()
{
    m_current_target_state = getStartPoint();
    m_trial_start_time = m_data->CurrentTime;
    m_energy_used = 0;
    m_previous_positions.clear();
    m_previous_time = 0;
    
    #if DEBUG_BEHAVIOUR_VERBOSITY > 3
        debug << "EvaluateSpeedOfWalkParametersState::startEvaluation() target:[" << m_current_target_state[0] << "," << m_current_target_state[1] << "," << m_current_target_state[2] << "]" << endl;
    #endif
}

/*! @brief Updates the evaluation */
void EvaluateSpeedOfWalkParametersState::tickEvaluation()
{
    m_provider->setDuration(m_data->CurrentTime - m_trial_start_time);
    updateEnergy();
}

/*! @brief Finalise the evaluation */
void EvaluateSpeedOfWalkParametersState::finishEvaluation()
{
    m_parent->markSpeedEvaluationCompleted();		// This is part of a hack to short-cut the stability test.
    m_provider->setEnergy(m_energy_used);
    #if DEBUG_BEHAVIOUR_VERBOSITY > 0
        debug << "EvaluateSpeedOfWalkParametersState::finishEvaluation() " << m_data->CurrentTime - m_trial_start_time << "ms " << m_energy_used << "J" << endl;
    #endif
}

/*! @brief Updates the energy used over the trial */
void EvaluateSpeedOfWalkParametersState::updateEnergy()
{
    // There are two ways to measure the energy used (a) using joint torques or (b) using battery currents (c) using joint currents
    vector<float> currents;
    vector<float> battery;
    vector<float> positions;
    vector<float> torques;
    
    bool batteryavaliable = m_data->getBatteryValues(battery);
    bool currentsavailable = m_data->getJointCurrents(NUSensorsData::BodyJoints, currents);
    
    m_data->getJointPositions(NUSensorsData::BodyJoints, positions);
    bool torquesavailable = m_data->getJointTorques(NUSensorsData::BodyJoints, torques);
    
    if (batteryavaliable)
    {
        // This code has never been tested, but should be OK on NAO
        float voltage = 3*(battery[2] + battery[3])/1000.0;        					// this has been hastily ported over from 2009!
        float current = -battery[1];
        if (m_previous_time != 0)
            m_energy_used += voltage*current*(m_data->CurrentTime - m_previous_time)/1000;
    }
    else if (currentsavailable)
    {
        /*float voltage = 3*(battery[2] + battery[3])/1000;        					// this has been hastily ported over from 2009!
        for (unsigned int i=0; i<currents.size(); i++)
            m_energy_used += fabs(currents[i]*voltage);
        m_energy_used += 21.0*(m_data->CurrentTime - m_previous_time);				// we assume for now the CPU etc draws 21W*/
    }
    else if (torquesavailable)
    {
        if (not m_previous_positions.empty())
        {
        	for (unsigned int i=0; i<positions.size(); i++)
            	m_energy_used += 2*fabs(torques[i]*(positions[i] - m_previous_positions[i]));			// the factor of two here model's the gearbox efficiency
            m_energy_used += 21.0*(m_data->CurrentTime - m_previous_time)/1000;							// we assume for now the CPU etc draws 21W
        }
        m_previous_positions = positions;
    }
    m_previous_time = m_data->CurrentTime;
}
    
/*! @brief Returns the starting point for the evaluation of the speed */
vector<float> EvaluateSpeedOfWalkParametersState::getStartPoint()
{
    float distance_from_forward = m_field_objects->self.CalculateDifferenceFromFieldState(m_points.front())[0];
    float distance_from_reverse = m_field_objects->self.CalculateDifferenceFromFieldState(m_points.back())[0];

    m_current_point_index = 0;
    if (distance_from_forward <= distance_from_reverse)
        m_reverse_points = false;
    else
        m_reverse_points = true;
    
    if (not m_reverse_points)
        return m_points[m_current_point_index];
    else
        return reversePoint(m_points[m_current_point_index]);
}
    
/*! @brief Returns true if the current point has been reached */
bool EvaluateSpeedOfWalkParametersState::pointReached()
{
    vector<float> difference = m_field_objects->self.CalculateDifferenceFromFieldState(m_current_target_state);
    if (difference[0] < 10 and fabs(difference[2]) < 0.2)
        return true;
    else
        return false;
}
    
/*! @brief Returns the next point in the list of way points in the predefined path */
vector<float> EvaluateSpeedOfWalkParametersState::getNextPoint()
{
    if (m_current_point_index < m_points.size()-1)
        m_current_point_index++;

    if (not m_reverse_points)
        return m_points[m_current_point_index];
    else
        return reversePoint(m_points[m_current_point_index]);
}
    
/*! @brief Returns true if the trial path has been completed */
bool EvaluateSpeedOfWalkParametersState::allPointsReached()
{
    if (not pointReached())
        return false;
    else
    {
        if (m_current_point_index == m_points.size()-1)
            return true;
        else
            return false;
    }
}

/*! @brief Returns the equivalent point on the reverse path */
vector<float> EvaluateSpeedOfWalkParametersState::reversePoint(const vector<float>& point)
{
    vector<float> reverse = m_points[(m_points.size()-1) - m_current_point_index];
    if (m_current_point_index == 0 or m_current_point_index == m_points.size()-1)
        return reverse;
    else
    {
        reverse[2] += 3.14;
        return reverse;
    }
    return reverse;
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


