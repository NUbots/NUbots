/*! @file WalkOptimisationState.h
    @brief Walk Optimisation State

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

#ifndef WALKOPTIMISATIONSTATE_H
#define WALKOPTIMISATIONSTATE_H

#include "Behaviour/BehaviourState.h"
#include "WalkOptimisationProvider.h"

#include "Infrastructure/NUSensorsData/NUSensorsData.h"
#include "Infrastructure/FieldObjects/FieldObjects.h"
#include "Infrastructure/Jobs/JobList.h"

#include "nubotconfig.h"
#include "debug.h"
#include "debugverbositybehaviour.h"

class WalkOptimisationState : public BehaviourState
{
public:
    WalkOptimisationState(WalkOptimisationProvider* parent)
    {
    	m_parent = parent;
    	m_way_points = m_parent->m_way_points;

    	m_current_time = 0;
    	m_previous_time = 0;
    	m_current_target_state = vector<float>(3,0);
    	m_starting_position = m_current_target_state;
    	m_current_position = m_current_target_state;
    	m_completed = false;
    	m_time_in_state = 0;
    	m_energy_used = 0;
    };
    virtual ~WalkOptimisationState() {};
    virtual BehaviourState* nextState() = 0;
    virtual void doState() = 0;

    /*! @brief Returns the distance travelled in this state
	 * 	@return distance travelled in cm
	 */
	virtual float distance()
	{
		float distance = sqrt(pow(m_current_position[0] - m_starting_position[0],2) + pow(m_current_position[1] - m_starting_position[1],2));
		return distance;
	}

	/*! @brief Returns the time spent in this state
	 * 	@ return the time spend in this state in ms
	 */
	float duration()
	{
		return m_time_in_state;
	}

	/*! @brief Returns the energy consumed in this state
	 * 	@return the energy in Joules used
	 */
	float energy()
	{
		return m_energy_used;
	}

	/*! @brief Returns true if the state was completed successfully on the last run
	 *  @return true if success, false if fail
	 */
	bool success()
	{
		return m_completed;
	}

protected:
	/*! @brief Start the walk optimisations state */
    virtual void start()
    {
		#if DEBUG_BEHAVIOUR_VERBOSITY > 0
			debug << "WalkOptimisationState::start()" << endl;
		#endif
		m_current_time = m_data->CurrentTime;
		m_previous_time = 0;
    	m_current_target_state = getStartPoint();
    	m_starting_position = m_field_objects->self.wmState();
    	m_current_position = m_starting_position;
    	m_completed = false;

    	m_time_in_state = 0;
    	m_energy_used = 0;
    	m_previous_positions.clear();
    }

    /*! @brief Tick the walk optimisation state */
    virtual void tick()
    {
    	m_current_time = m_data->CurrentTime;
    	m_current_position = m_field_objects->self.wmState();
    	updateTime();
		updateEnergy();
		#if DEBUG_BEHAVIOUR_VERBOSITY > 1
			debug << "WalkOptimisationState::tick(). position: " << m_current_position << " time: " << m_time_in_state << " energy: " << m_energy_used << endl;
		#endif
		m_previous_time = m_current_time;
    }

    /*! @brief Finish the walk optimisation state */
    virtual void finish()
    {
    	m_completed = true;
		#if DEBUG_BEHAVIOUR_VERBOSITY > 0
			debug << "WalkOptimisationState::finish(). distance: " << distance() << " duration(): " << duration() << " energy(): " << energy() << endl;
		#endif
    }

    /* @brief Returns the start point for the optimisation state */
    virtual vector<float> getStartPoint()
	{
		vector<float> front = m_way_points.front();
		vector<float> back = m_way_points.back();

		vector<float> difference_from_front = m_field_objects->self.CalculateDifferenceFromFieldState(front);
		vector<float> difference_from_back = m_field_objects->self.CalculateDifferenceFromFieldState(back);

		vector<float> startpoint;
		if (difference_from_back[0] < difference_from_front[0])
			startpoint = back;
		else
			startpoint = front;

		return startpoint;
	}

    /*! @brief Updates the time in this state */
    void updateTime()
    {
    	bool gettingup = false;
		m_data->get(NUSensorsData::MotionGetupActive, gettingup);
		if (not gettingup and m_previous_time != 0)
			m_time_in_state += m_current_time - m_previous_time;
    }

    /* @brief Updated the energy consumed in this state */
    void updateEnergy()
	{
    	bool gettingup = false;
    	m_data->get(NUSensorsData::MotionGetupActive, gettingup);
    	if (gettingup)
		{
    		m_previous_positions.clear();
    		return;
    	}

		// There are three ways to measure the energy used (a) using joint torques or (b) using battery currents (c) using joint currents
		vector<float> currents;
		float batteryvoltage, batterycurrent;
		vector<float> positions;
		vector<float> torques;

		bool batteryavaliable = m_data->getBatteryVoltage(batteryvoltage);
		batteryavaliable &= m_data->getBatteryCurrent(batterycurrent);
		bool currentsavailable = m_data->getCurrent(NUSensorsData::Body, currents);

		bool positionsavailable = m_data->getPosition(NUSensorsData::Body, positions);
		bool torquesavailable = m_data->getTorque(NUSensorsData::Body, torques);

		if (false and batteryavaliable)
		{
			float voltage = batteryvoltage;
			float current = -batterycurrent;
			if (m_previous_time != 0)
				m_energy_used += voltage*current*(m_current_time - m_previous_time)/1000;
		}
		else if (currentsavailable)
		{
			if (m_previous_time != 0)
			{
				for (unsigned int i=0; i<currents.size(); i++)
					m_energy_used += 0.69*fabs(currents[i]*batteryvoltage)*(m_current_time - m_previous_time)/1000;       // the 0.69 is a fudge factor to make the current sum match the battery current
				m_energy_used += 17.3*(m_current_time - m_previous_time)/1000;				// we assume for now the CPU etc draws 17.3W

			}
		}
		else if (torquesavailable)
		{
			if (not m_previous_positions.empty())
			{
				for (unsigned int i=0; i<positions.size(); i++)
					m_energy_used += 2*fabs(torques[i]*(positions[i] - m_previous_positions[i]));			// the factor of two here model's the gearbox efficiency
				m_energy_used += 21.0*(m_current_time - m_previous_time)/1000;								// we assume for now the CPU etc draws 21W
			}
			m_previous_positions = positions;
		}
	}

    /*! @brief Moves the head to look at the goals
     * 		   Depending on what is visible there are several different ways to look at a 'goal'
     * 				- If both goals are seen, then we look between them so we can see the whole goal
     * 				- If only a single known post is visible then we look at that
     * 				- If only an unknown post is visible then we just look at that
     * 				- Finally if no posts are visible then we just do a Localisation pan
     * */
    void lookAtGoals()
    {
		#ifdef USE_VISION		// only try to look at the goals if we are using vision
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
		#endif
    }
protected:
    WalkOptimisationProvider* m_parent;

    vector<vector<float> > m_way_points;		//!< the list of way points along the circuit used for optimisation

    float m_current_time;						//!< the current time in ms
    float m_previous_time;						//!< the previous time this state was run in ms
    vector<float> m_current_target_state;		//!< the current field location we are walking to [x,y,theta]
    vector<float> m_starting_position;			//!< the position [x,y,theta] at which the this state was entered
    vector<float> m_current_position;			//!< the current position [x,y,theta], when leaving this state, this will become the final position
    bool m_completed;							//!< true if the last time we were in this state, it completed successfully.

    float m_time_in_state;						//!< the amount of time in ms spent in this state
    float m_energy_used;						//!< the amount of energy in Joules used in this state
    vector<float> m_previous_positions;			//!< the previous joint positions (used to calculate energy used via joint torques)
};

#endif

