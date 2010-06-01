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

#include "WalkOptimisationProvider.h"
#include "WalkOptimisationState.h"

#include "Behaviour/Jobs/JobList.h"
#include "NUPlatform/NUSensors/NUSensorsData.h"
#include "Vision/FieldObjects/FieldObjects.h"

#include "Behaviour/Jobs/MotionJobs/WalkJob.h"
#include "Behaviour/Jobs/MotionJobs/HeadTrackJob.h"

#include "debug.h"

class ChaseObject : public WalkOptimisationState
{
public:
    ChaseObject(WalkOptimisationProvider* parent, int fieldobjectid, int ambiguousfieldobjectid) : WalkOptimisationState(parent)
    {
        m_id = fieldobjectid;
        m_ambiguous_id = ambiguousfieldobjectid;
    };
    virtual ~ChaseObject() {};
    virtual BehaviourState* nextState() = 0;
    virtual void doState()
    {
        // track the object
        HeadTrackJob* job = new HeadTrackJob(m_target);
        m_jobs->addMotionJob(job);
        
        // walk torward the object
        float headyaw;
        m_data->getJointPosition(NUSensorsData::HeadYaw, headyaw);
        WalkJob* walkjob;
        walkjob = new WalkJob(1, m_target.measuredBearing(), (m_target.measuredBearing())/2.0);
        m_jobs->addMotionJob(walkjob);
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
};

#endif

