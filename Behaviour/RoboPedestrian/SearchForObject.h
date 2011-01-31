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

#include "RoboPedestrianProvider.h"
#include "RoboPedestrianState.h"

#include "Infrastructure/Jobs/JobList.h"
#include "Infrastructure/NUSensorsData/NUSensorsData.h"
#include "Infrastructure/FieldObjects/FieldObjects.h"

#include "Infrastructure/Jobs/MotionJobs/WalkJob.h"
#include "Infrastructure/Jobs/MotionJobs/HeadNodJob.h"

#include "debug.h"

class SearchForObject : public RoboPedestrianState
{
public:
    SearchForObject(RoboPedestrianProvider* parent, int fieldobjectid, int ambiguousfieldobjectid = -1) : RoboPedestrianState(parent)
    {
        m_id = fieldobjectid;
        m_ambiguous_id = ambiguousfieldobjectid;
    };
    virtual ~SearchForObject() {};
    virtual BehaviourState* nextState() = 0;
    virtual void doState() = 0;
    
protected:
    bool isTargetVisible()
    {
        if (m_field_objects == NULL)
        {   // if there are no field objects then we definitely cant see the target
            return false;
        }
        else if (m_id >= 0 and m_field_objects->stationaryFieldObjects[m_id].isObjectVisible())
        {   // if the non-ambiguous field object is seen
            return true;
        }
        else if (m_ambiguous_id >= 0 and (not m_field_objects->ambiguousFieldObjects.empty()))
        {   // if there are ambiguous objects, check if one of them is the ambiguous version of this object 
            for (size_t i=0; i<m_field_objects->ambiguousFieldObjects.size(); i++)
            {
                int ambig_id = m_field_objects->ambiguousFieldObjects[i].getID();
                if (ambig_id == m_ambiguous_id)
                    return true;
            }
            // if none were the ambiguous version
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

