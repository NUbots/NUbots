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
        m_tracking_ball = false;
        m_tracking_time = 0;
        m_chasing_time = 0;
        m_previous_time = 0;
    };
    
    virtual ~ChaseObject() {};
    virtual BehaviourState* nextState() = 0;
        
    virtual void doState()
    {
        MobileObject& ball = m_field_objects->mobileFieldObjects[FieldObjects::FO_BALL];
        if (m_previous_time == 0 or m_data->CurrentTime - m_previous_time > 500)
        {   // check if this is the first time this state has been run in awhile, if so: reset
            m_previous_time = m_data->CurrentTime;
            m_tracking_ball = false;
            m_tracking_time = 0;
            m_chasing_time = 0;
        }
        
        vector<float> speed(3,0);
        vector<float> result(3,0);
        speed[0] = 1;
        speed[1] = m_target.measuredBearing();
        speed[2] = m_target.measuredBearing()/2;
        result = BehaviourPotentials::sensorAvoidObjects(speed, m_data, 40, 100);
        
        if (not m_tracking_ball)
        {   // if we are chasing the object

            m_jobs->addMotionJob(new HeadPanJob(HeadPanJob::BallAndLocalisation, 75, 300, -0.6, 0.6)); //Min Distance should be 5cm not 400
            
            m_chasing_time += m_data->CurrentTime - m_previous_time;
            
            if (m_chasing_time > 5000 and ball.isObjectVisible() and ball.measuredDistance() > 75 and ball.measuredDistance() < 180)
            {
                m_tracking_ball = true;
                m_tracking_time = 0;
            }
        }
        else
        {   // if we are distracted by the ball
            if(ball.isObjectVisible())
            {
                float offset = mathGeneral::sign(ball.measuredBearing())*0.3;
                m_jobs->addMotionJob(new HeadTrackJob(ball, 0.15, offset));
            }
            
            result[0] *= 0.5;
            
            m_tracking_time += m_data->CurrentTime - m_previous_time;
            
            if (m_tracking_time > 3000)
            {
                m_tracking_ball = false;
                m_chasing_time = 0;
            }
        }
        m_jobs->addMotionJob(new WalkJob(result[0], result[1], result[2]));

        m_previous_time = m_data->CurrentTime;
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
    int m_id;                   //!< the id of the field object you want to chase
    int m_ambiguous_id;         //!< the ambiguous id of the field object you want to chase
    Object m_target;            //!< the object we are 'chasing'
    bool m_tracking_ball;       //!< a flag to indicate whether we are tracking the distraction
    float m_chasing_time;       //!< the time spent chasing the target
    float m_tracking_time;      //!< the time spent tracking the ball

    double m_previous_time;     //!< the previous time this tate was run
};

#endif

