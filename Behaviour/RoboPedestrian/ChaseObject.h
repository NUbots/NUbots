/*! @file ChaseObject.h
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

#ifndef CHASEOBJECT_H
#define CHASEOBJECT_H

#include "RoboPedestrianProvider.h"
#include "RoboPedestrianState.h"

#include "Infrastructure/Jobs/JobList.h"
#include "Infrastructure/NUSensorsData/NUSensorsData.h"
#include "Infrastructure/FieldObjects/FieldObjects.h"

#include "Infrastructure/Jobs/MotionJobs/WalkJob.h"
#include "Infrastructure/Jobs/MotionJobs/HeadTrackJob.h"
#include "Infrastructure/Jobs/MotionJobs/HeadPanJob.h"

#include "Behaviour/BehaviourPotentials.h"

#include "debug.h"

class ChaseObject : public RoboPedestrianState
{
public:
    ChaseObject(RoboPedestrianProvider* parent, int fieldobjectid, int ambiguousfieldobjectid) : RoboPedestrianState(parent)
    {
        m_id = fieldobjectid;
        m_ambiguous_id = ambiguousfieldobjectid;
	lastBallTrack = 0;
	timeSinceTrack = 0;
    };
    virtual ~ChaseObject() {};
    virtual BehaviourState* nextState() = 0;
    virtual void doState()
    {
        // track the object
        if (m_target.isObjectVisible())
         //   m_jobs->addMotionJob(new HeadTrackJob(m_target));
		targetBearing = m_target.measuredBearing();

        if(m_field_objects->mobileFieldObjects[FieldObjects::FO_BALL].TimeSeen() < 3000
		&& timeSinceTrack > 5000
		&& m_field_objects->mobileFieldObjects[FieldObjects::FO_BALL].TimeSinceLastSeen() < 500
		//&& m_field_objects->mobileFieldObjects[FieldObjects::FO_BALL].isObjectVisible()
	        && m_field_objects->mobileFieldObjects[FieldObjects::FO_BALL].measuredDistance() > 75
		&& m_field_objects->mobileFieldObjects[FieldObjects::FO_BALL].measuredDistance() < 400
	)
	{
		if(m_field_objects->mobileFieldObjects[FieldObjects::FO_BALL].isObjectVisible())
			m_jobs->addMotionJob(new HeadTrackJob(m_field_objects->mobileFieldObjects[FieldObjects::FO_BALL],0.25,0));
		
		m_jobs->addMotionJob(new WalkJob(0.2, targetBearing, (targetBearing)/2.0));
		lastBallTrack = m_data->CurrentTime;
	}
	else
	{
		m_jobs->addMotionJob(new HeadPanJob(HeadPanJob::BallAndLocalisation,100, 300, -0.6, 0.6)); //Min Distance should be 5cm not 400
		timeSinceTrack = m_data->CurrentTime - lastBallTrack;
		vector<float> speed(3,0);
		vector<float> result(3,0);
		speed[0] = 1;
		speed[1] = targetBearing;
		speed[2] = targetBearing/2;
		result = BehaviourPotentials::sensorAvoidObjects(speed, m_data, 40, 100);
            
		m_jobs->addMotionJob(new WalkJob(result[0], result[1], result[2]));
	}
	
        // walk torward the object
        //m_jobs->addMotionJob(new WalkJob(1, m_target.measuredBearing(), (m_target.measuredBearing())/2.0));
	//m_jobs->addMotionJob(new WalkJob(1, targetBearing, (targetBearing)/2.0));

    };
protected:
    void updateTarget()
    {
        if (m_field_objects == NULL)
            return;
        else if (m_id >= 0 and m_field_objects->stationaryFieldObjects[m_id].isObjectVisible())
            m_target = m_field_objects->stationaryFieldObjects[m_id];
        else if (not m_field_objects->ambiguousFieldObjects.empty())
        {
            for (size_t i=0; i<m_field_objects->ambiguousFieldObjects.size(); i++)
            {
                int ambig_id = m_field_objects->ambiguousFieldObjects[i].getID();
                if (ambig_id == m_ambiguous_id)
                {
                    m_target = m_field_objects->ambiguousFieldObjects[i];
                    break;
                }
            }
        }
    };
protected:
    int m_id;               //!< the id of the field object you want to chase
    int m_ambiguous_id;     //!< the ambiguous id of the field object you want to chase
    Object m_target;
    float targetBearing;
    float timeSinceTrack;
    float lastBallTrack;
};

#endif

