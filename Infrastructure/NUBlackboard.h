/*! @file NUBlackboard.h
    @brief The blackboard class.
 
    @class NUBlackboard
    @brief A blackboard for storing data that is required by multiple modules.
           The blackboard contains the following containers
                - Sensors; which contains all of the sensor data
                - Actions; which contains all of the actions for the robot's hardware
                - Objects; which contains all of the information about the location of landmarks in the environment
                - Jobs; which contains all of the pending jobs for the robot
                - GameInfo; which contains all of the information about the state of the 'game'
                - TeamInfo; which contains all of the information about the team mates' state
 
    @note Adding a new type of object to the Blackboard is considered a major change, and should be avoided.
 
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

#ifndef NUBLACKBOARD_H
#define NUBLACKBOARD_H

class NUSensorsData;
class NUActionatorsData;
class NUImage;
class FieldObjects;
class JobList;
class GameInformation;
class TeamInformation;
class NUPlatform;

class NUBlackboard
{
public:
    NUBlackboard();
    ~NUBlackboard();
    
    void add(NUSensorsData* sensorsdata);
    void add(NUActionatorsData* actionsdata);
    void add(NUImage* image);
    void add(FieldObjects* objects);
    void add(JobList* joblist);
    void add(GameInformation* gameinfo);
    void add(TeamInformation* teaminfo);
    
public:
    NUSensorsData* Sensors;
    NUActionatorsData* Actions;
    NUImage* Image;
    FieldObjects* Objects;
    JobList* Jobs;
    GameInformation* GameInfo;
    TeamInformation* TeamInfo;
    bool lookForBall;
};

extern NUBlackboard* Blackboard;

#endif

