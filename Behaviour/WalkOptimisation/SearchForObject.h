/*! @file SearchForObject.h
    @brief Search for a field object

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

#ifndef SEARCHFOROBJECT_H
#define SEARCHFOROBJECT_H

#include "Behaviour/BehaviourState.h"
#include "WalkOptimisationProvider.h"
#include "WalkOptimisationState.h"

#include "Behaviour/Jobs/JobList.h"
#include "NUPlatform/NUSensors/NUSensorsData.h"
#include "Vision/FieldObjects/FieldObjects.h"

#include "Behaviour/Jobs/MotionJobs/WalkJob.h"
#include "Behaviour/Jobs/MotionJobs/HeadNodJob.h"

#include "debug.h"

class SearchForObject : public WalkOptimisationState
{
public:
    SearchForObject(WalkOptimisationProvider* parent, int fieldobjectid, int ambiguousfieldobjectid = -1) : WalkOptimisationState(parent)
    {
        m_id = fieldobjectid;
        m_ambiguous_id = ambiguousfieldobjectid;
    };
    virtual ~SearchForObject() {};
    virtual BehaviourState* nextState() = 0;
    virtual void doState()
    {
        m_parent->m_jobs->addMotionJob(new WalkJob(0,0,0.3));
        m_parent->m_jobs->addMotionJob(new HeadNodJob(HeadNodJob::Localisation, 0.3));
    };
protected:
    bool isTargetVisible()
    {
        if (m_id >= 0 and m_parent->m_field_objects->stationaryFieldObjects[m_id].isObjectVisible())
        {   // if the non-ambiguous field object is seen
            return true;
        }
        else if (m_ambiguous_id >= 0 and (not m_parent->m_field_objects->ambiguousFieldObjects.empty()))
        {   // if there are ambiguous objects, check if one of them is the ambiguous version of this object 
            for (size_t i=0; i<m_parent->m_field_objects->ambiguousFieldObjects.size(); i++)
            {
                int ambig_id = m_parent->m_field_objects->ambiguousFieldObjects[i].getID();
                if (ambig_id == m_ambiguous_id)
                    return true;
            }
            // if non were the ambiguous version
            return false;
        }
        else
            return false;
    };
protected:
    int m_id;               //!< the id of the field object you want to chase
    int m_ambiguous_id;     //!< the ambiguous id of the field object you want to chase
};

#endif

