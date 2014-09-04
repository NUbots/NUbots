/*! @file EvaluateWalkParametersState.cpp
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

#include "EvaluateWalkParametersState.h"

#include "WalkOptimisationProvider.h"

#include "Infrastructure/NUSensorsData/NUSensorsData.h"
#include "Infrastructure/Jobs/JobList.h"
#include "Infrastructure/Jobs/MotionJobs/WalkJob.h"
#include "Infrastructure/FieldObjects/FieldObjects.h"

#include "Behaviour/BehaviourPotentials.h"

#include "debug.h"
#include "debugverbositybehaviour.h"

/*! @brief Construct a evaluate walk parameters state
    @param parent the walk optimisation provider
 */
EvaluateWalkParametersState::EvaluateWalkParametersState(WalkOptimisationProvider* parent) : WalkOptimisationState(parent)
{
	#if DEBUG_BEHAVIOUR_VERBOSITY > 3
        debug << "EvaluateWalkParametersState::EvaluateWalkParametersState" << std::endl;
    #endif
}

/*! @brief Destroy the evaluate walk parameters state */
EvaluateWalkParametersState::~EvaluateWalkParametersState()
{
};

/*! @brief Returns the desired next state in the walk optimisation provider */
BehaviourState* EvaluateWalkParametersState::nextState()
{
	bool getting_up = false;
	m_data->get(NUSensorsData::MotionGetupActive, getting_up);
	if (allPointsReached())
	{
		finish();
		return m_parent->m_generate;
	}
	else if (m_time_in_state > 100000 or getting_up)
		return m_parent->m_generate;
	else
		return this;
}

/*! @brief Do the state behaviour */
void EvaluateWalkParametersState::doState()
{
    #if DEBUG_BEHAVIOUR_VERBOSITY > 3
        debug << "EvaluateWalkParametersState::doState() target: " << m_current_target_state << std::endl;
    #endif
    if (m_parent->stateChanged())
        start();
    else
    	tick();

    lookAtGoals();

    if (pointReached())
        m_current_target_state = getNextPoint();

    std::vector<float> speed = BehaviourPotentials::goToFieldState(m_field_objects->self, m_current_target_state, 0, 20, 5000);
    m_jobs->addMotionJob(new WalkJob(speed[0], speed[1], speed[2]));
}

/*! @brief Returns the starting point for the evaluation of the speed */
std::vector<float> EvaluateWalkParametersState::getStartPoint()
{
    float distance_from_forward = m_field_objects->self.CalculateDifferenceFromFieldState(m_way_points.front())[0];
    float distance_from_reverse = m_field_objects->self.CalculateDifferenceFromFieldState(m_way_points.back())[0];

    m_current_point_index = 0;
    if (distance_from_forward <= distance_from_reverse)
        m_reverse_points = false;
    else
        m_reverse_points = false;

    if (not m_reverse_points)
        return m_way_points[m_current_point_index];
    else
        return reversePoint(m_way_points[m_current_point_index]);
}

/*! @brief Returns true if the current point has been reached */
bool EvaluateWalkParametersState::pointReached()
{
    std::vector<float> difference = m_field_objects->self.CalculateDifferenceFromFieldState(m_current_target_state);
    if (difference[0] < 35 and fabs(difference[2]) < 0.4)
        return true;
    else
        return false;
}

/*! @brief Returns the next point in the std::list of way points in the predefined path */
std::vector<float> EvaluateWalkParametersState::getNextPoint()
{
    if (m_current_point_index < m_way_points.size()-1)
        m_current_point_index++;

    if (not m_reverse_points)
        return m_way_points[m_current_point_index];
    else
        return reversePoint(m_way_points[m_current_point_index]);
}

/*! @brief Returns true if the trial path has been completed */
bool EvaluateWalkParametersState::allPointsReached()
{
    if (not pointReached())
        return false;
    else
    {
        if (m_current_point_index == m_way_points.size()-1)
            return true;
        else
            return false;
    }
}

/*! @brief Returns the equivalent point on the reverse path */
std::vector<float> EvaluateWalkParametersState::reversePoint(const std::vector<float>& point)
{
    std::vector<float> reverse = m_way_points[(m_way_points.size()-1) - m_current_point_index];
    if (m_current_point_index == 0 or m_current_point_index == m_way_points.size()-1)
    {
        m_previous_reverse = reverse;
        return reverse;
    }
    else
    {
        reverse[0] = m_way_points[m_current_point_index-1][0];
        reverse[1] = m_way_points[m_current_point_index-1][1];
        reverse[2] += 3.14;
        m_previous_reverse = reverse;
        return reverse;
    }
    return reverse;
}

/*! @brief Returns the total distance travelled along the circuit so far
 * 				- This function will ignore travelling that is not on the path
 * 	@return the distance in cm
 */
float EvaluateWalkParametersState::distance()
{
	if(m_completed)
		return calculateCircuitLength();
	else if (m_current_point_index == 0)
		return 0;
	else
	{
		float distance = 0;
		for (size_t i=1; i<m_current_point_index; i++)
			distance += sqrt(pow(m_way_points[i][0] - m_way_points[i-1][0],2) + pow(m_way_points[i][1] - m_way_points[i-1][1],2));

		// because i'm lazy the distance for the leg on which we fell over is (the distance of the final leg) - (the distance from the target)
		float currentleg = sqrt(pow(m_current_target_state[0] - m_way_points[m_current_point_index-1][0],2) + pow(m_current_target_state[1] - m_way_points[m_current_point_index-1][1],2));
		float distancefromtarget = m_field_objects->self.CalculateDifferenceFromFieldState(m_current_target_state)[0];
		distance += currentleg - distancefromtarget;
		return distance;
	}
}

/*! @brief Returns the distance in cm of the path specified by m_points
 *  @return the distance of the evaluation path
 */
float EvaluateWalkParametersState::calculateCircuitLength()
{
    float distance = 0;
    for (size_t i=1; i<m_way_points.size(); i++)
        distance += sqrt(pow(m_way_points[i][0] - m_way_points[i-1][0],2) + pow(m_way_points[i][1] - m_way_points[i-1][1],2));
    return distance;
}
