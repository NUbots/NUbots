/*! @file EvaluateStabilityOfWalkParametersState.h
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

#include "EvaluateStabilityOfWalkParametersState.h"
#include "WalkOptimisationProvider.h"
#include "EvaluateWalkParametersState.h"

#include "NUPlatform/NUActionators/NUActionatorsData.h"
#include "Vision/FieldObjects/FieldObjects.h"
#include "Behaviour/Jobs/JobList.h"
#include "Behaviour/Jobs/MotionJobs/WalkJob.h"
#include "Behaviour/Jobs/MotionJobs/HeadTrackJob.h"
#include "Behaviour/Jobs/MotionJobs/HeadPanJob.h"
#include "Behaviour/Jobs/MotionJobs/WalkPerturbationJob.h"

#include "Behaviour/BehaviourPotentials.h"
#include "Motion/Tools/MotionFileTools.h"

#include "debug.h"
#include "debugverbositybehaviour.h"

// ----------------------------------------------------------------------------------------------------------------------- EvaluateStabilityOfWalkParametersState
/*! @brief Construct a evaluate stability of walk parameters state 
    @param parent the parent EvaluateWalkParametersState
 */
EvaluateStabilityOfWalkParametersState::EvaluateStabilityOfWalkParametersState(EvaluateWalkParametersState* parent)
{
    #if DEBUG_BEHAVIOUR_VERBOSITY > 1
        debug << "EvaluateStabilityOfWalkParametersState::EvaluateStabilityOfWalkParametersState" << endl;
    #endif
    m_parent = parent;
    m_provider = parent->m_parent;
    m_points = m_provider->m_stability_points;
    
    m_init = new EvaluateStabilityOfWalkParametersStartState(this);
    m_run = new EvaluateStabilityOfWalkParametersRunState(this);
    m_state = m_init;
}

/*! @brief Destroy a evaluate stability of walk parameter state */
EvaluateStabilityOfWalkParametersState::~EvaluateStabilityOfWalkParametersState()
{
    delete m_init;
    m_init = 0;
    delete m_run;
    m_run = 0;
}

/*! @brief Returns the desired next state in the EvaluateStability state machine */
BehaviourState* EvaluateStabilityOfWalkParametersState::nextStateCommons()
{
    if (m_parent->stateChanged())
        return m_init;
    else
        return m_state;
}

// ----------------------------------------------------------------------------------------------------------------------- EvaluateStabilityOfWalkParametersStartState
/*! @brief Construct a evaluate stability of walk parameters start state 
    @param parent the parent EvaluateStabilityOfWalkParametersState
 */
EvaluateStabilityOfWalkParametersStartState::EvaluateStabilityOfWalkParametersStartState(EvaluateStabilityOfWalkParametersState* parent): m_parent(parent), m_provider(parent->m_provider)
{
    m_current_start_state = vector<float>(3,0);
};

/*! @brief Returns the desired next state in the evaluate stability of walk parameters state machine */
BehaviourState* EvaluateStabilityOfWalkParametersStartState::nextState()
{// progress to the evaluation state when we are in position AND lined up
    vector<float> difference = m_field_objects->self.CalculateDifferenceFromFieldState(m_current_start_state);
    if (difference[0] < 5 and fabs(difference[2]) < 0.2)
        return m_parent->m_run;
    else
        return this;
}

/*! @brief Do the stability of walk parameters start state */
void EvaluateStabilityOfWalkParametersStartState::doState()
{
    #if DEBUG_BEHAVIOUR_VERBOSITY > 1
        debug << "EvaluateStabilityOfWalkParametersStartState::doState()" << endl;
    #endif
    if (m_parent->stateChanged() or m_parent->m_parent->stateChanged())
        m_current_start_state = getStartState();
    
    lookAtGoals();
    
    vector<float> speed = BehaviourPotentials::goToFieldState(m_field_objects->self, m_current_start_state, 5, 50, 9000);
    m_jobs->addMotionJob(new WalkJob(speed[0], speed[1], speed[2]));
}

/*! @brief Returns the start state for the stability test */
vector<float> EvaluateStabilityOfWalkParametersStartState::getStartState()
{
    #if DEBUG_BEHAVIOUR_VERBOSITY > 1
        debug << "EvaluateStabilityOfWalkParametersStartState::getStartState()" << endl;
    #endif
    vector<vector<float> >& points = m_parent->m_points;
    vector<float> front = points.front();
    vector<float> back = points.back();
    
    vector<float> difference_from_front = m_field_objects->self.CalculateDifferenceFromFieldState(front);
    vector<float> difference_from_back = m_field_objects->self.CalculateDifferenceFromFieldState(back);
    
    vector<float> startpoint;
    if (difference_from_back[0] < difference_from_front[0])
        startpoint = back;
    else
        startpoint = front;
    
    #if DEBUG_BEHAVIOUR_VERBOSITY > 0
        debug << "EvaluateStabilityOfWalkParametersStartState::getStartState(): " << MotionFileTools::fromVector(startpoint) << endl;
    #endif
    return startpoint;
}

/*! @brief Controls the head to look at the goal so that the robot localises nicely */
void EvaluateStabilityOfWalkParametersStartState::lookAtGoals()
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

// ----------------------------------------------------------------------------------------------------------------------- EvaluateStabilityOfWalkParametersStartState
/*! @brief Construct a evaluate stability of walk parameters state */
EvaluateStabilityOfWalkParametersRunState::EvaluateStabilityOfWalkParametersRunState(EvaluateStabilityOfWalkParametersState* parent): m_parent(parent), 
                                                                                                                                      m_provider(parent->m_provider), 
                                                                                                                                      m_PI(3.1416),
                                                                                                                                      m_INITIAL_PERTURBATION_MAG(10),
                                                                                                                                      m_PERTURBATION_MAG_INC(10),
                                                                                                                                      m_PERTURBATION_INTERVAL(3)
{
    reset();
}

/*! @brief Returns the desired next state in the EvaluateStability state machine */
BehaviourState* EvaluateStabilityOfWalkParametersRunState::nextState()
{
    return this;
}

void EvaluateStabilityOfWalkParametersRunState::doState()
{
    #if DEBUG_BEHAVIOUR_VERBOSITY > 1
        debug << "EvaluateStabilityOfWalkParametersRunState::doState() stepcount:" << m_step_count << endl;
    #endif
    if (m_parent->stateChanged())
        reset();
    float previousleft = m_left_impact_time;
    float previousright = m_right_impact_time;
    m_data->footImpact(NUSensorsData::LeftFoot, m_left_impact_time);
    m_data->footImpact(NUSensorsData::RightFoot, m_right_impact_time);
    
    if (m_left_impact_time != previousleft or m_right_impact_time != previousright)
        m_step_count++;
    
    if (m_step_count%m_PERTURBATION_INTERVAL == 0 and m_step_count > 3)
        doPerturbation();
    
    m_jobs->addMotionJob(new WalkJob(1, -0.785, 0));
}

/*! @brief Resets all of the trial variables */
void EvaluateStabilityOfWalkParametersRunState::reset()
{
    #if DEBUG_BEHAVIOUR_VERBOSITY > 0
        debug << "EvaluateStabilityOfWalkParametersRunState::doState(). Resetting." << endl;
    #endif
    m_perturbation_magnitude = m_INITIAL_PERTURBATION_MAG;
    m_perturbation_direction = 0;
    m_step_count = 0;
    m_perturbation_count = 0;
    m_left_impact_time = 0;
    m_right_impact_time = 0;
}

/*! @brief Generates a single perturbation 40ms after the last foot impact */
void EvaluateStabilityOfWalkParametersRunState::doPerturbation()
{
    float perturbationtime = max(m_left_impact_time, m_right_impact_time) + 40;
    if (m_data->CurrentTime - perturbationtime >= 0 and m_step_last_perturbed != m_step_count)
        generatePerturbation();
}

/*! @brief Adds a perturbation job to the joblist. Handles the gradual increase in magnitude and the direction sequence */
void EvaluateStabilityOfWalkParametersRunState::generatePerturbation()
{
    #if DEBUG_BEHAVIOUR_VERBOSITY > 0
        debug << "EvaluateStabilityOfWalkParametersRunState::generatePerturbation() " << m_perturbation_magnitude << " " << m_perturbation_direction << endl;
    #endif
    m_step_last_perturbed = m_step_count;
    m_perturbation_count++;
    
    m_jobs->addMotionJob(new WalkPerturbationJob(m_perturbation_magnitude, m_perturbation_direction));
    
    // ----------------------------------- Update the perturbation direction
    // the perturbation direction sequence is 0, PI/2, PI, -PI/2 (for both odd and even step intervals)
    m_perturbation_direction += m_PI/2;
    if (m_perturbation_direction > m_PI)
        m_perturbation_direction = -m_PI/2;
    
    // ----------------------------------- Update the perturbation magnitude
    if (m_perturbation_direction == 0)
        m_perturbation_magnitude += m_PERTURBATION_MAG_INC;
}


