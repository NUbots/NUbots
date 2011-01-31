/*! @file SearchForBlueGoal.h
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

#ifndef SEARCHFORBLUEGOAL_H
#define SEARCHFORBLUEGOAL_H

#include "Behaviour/BehaviourState.h"
#include "RoboPedestrianProvider.h"
#include "SearchForObject.h"

#include "Infrastructure/Jobs/JobList.h"
#include "Infrastructure/NUSensorsData/NUSensorsData.h"
#include "Infrastructure/FieldObjects/FieldObjects.h"

#include "Infrastructure/Jobs/MotionJobs/WalkJob.h"
#include "Infrastructure/Jobs/MotionJobs/HeadNodJob.h"

#include "debug.h"

class SearchForBlueGoal : public SearchForObject
{
public:
    SearchForBlueGoal(RoboPedestrianProvider* parent) : SearchForObject(parent, FieldObjects::FO_BLUE_RIGHT_GOALPOST, -1) {};
    virtual ~SearchForBlueGoal() {};
    virtual BehaviourState* nextState()
    {
        if (isTargetVisible())
        {
            debug << "SearchForBlueGoal -> ChaseBlueGoal" << endl;
            return m_parent->m_chase_blue_goal;
        }
        else
            return this;
    };
    virtual void doState()
    {
        m_jobs->addMotionJob(new WalkJob(0,0,-0.3));
        m_jobs->addMotionJob(new HeadNodJob(HeadNodJob::Localisation, 0.3));
	//m_jobs->addMotionJob(new HeadPanJob(HeadPanJob::BallAndLocalisation, 40, 90000, -0.8, 0.8));
    };
};

#endif

