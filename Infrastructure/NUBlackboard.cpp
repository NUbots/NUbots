/*! @file NUBlackboard.cpp
    @brief The implementation of the blackboard class.
 
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

#include "NUBlackboard.h"

#include "Infrastructure/NUSensorsData/NUSensorsData.h"
#include "Infrastructure/NUActionatorsData/NUActionatorsData.h"
#include "Infrastructure/NUImage/NUImage.h"
#include "Infrastructure/FieldObjects/FieldObjects.h"
#include "Infrastructure/Jobs/Jobs.h"
#include "Infrastructure/GameInformation/GameInformation.h"
#include "Infrastructure/TeamInformation/TeamInformation.h"

NUBlackboard* Blackboard = 0;

/*! @brief Construct the NUBlackboard --- the global data store. 
*/
NUBlackboard::NUBlackboard()
{
    Blackboard = this;
    Sensors = 0;
    Actions = 0;
    Image = 0;
    Objects = 0;
    Jobs = 0;
    GameInfo = 0;
    TeamInfo = 0;
}

NUBlackboard::~NUBlackboard()
{
    delete Sensors;
    Sensors = 0;
    delete Actions;
    Actions = 0;
    delete Image;
    Image = 0;
    delete Objects;
    Objects = 0;
    delete Jobs;
    Jobs = 0;
    delete GameInfo;
    GameInfo = 0;
    delete TeamInfo;
    TeamInfo = 0;
}

/*! @brief Adds a NUSensorsData object to the blackboard. Note that ownership of the object is now with the Blackboard. 
    @param sensorsdata a pointer to the new sensors data
 */
void NUBlackboard::add(NUSensorsData* sensorsdata)
{
    NUSensorsData* oldsensors = Sensors;
    Sensors = sensorsdata;
    delete oldsensors;
}

/*! @brief Adds a NUActionatorsData object to the blackboard. Note that ownership of the object is now with the Blackboard. 
    @param actionsdata a pointer to the new actions data
 */
void NUBlackboard::add(NUActionatorsData* actionsdata)
{
    NUActionatorsData* oldactions = Actions;
    Actions = actionsdata;
    delete oldactions;
}

/*! @brief Adds a NUImage object to the blackboard. Note that ownership of the object is now with the Blackboard. 
    @param image a pointer to the new image
 */
void NUBlackboard::add(NUImage* image)
{
    NUImage* oldimage = Image;
    Image = image;
    delete oldimage;
}

/*! @brief Adds a FieldObjects object to the blackboard. Note that ownership of the object is now with the Blackboard. 
    @param objects a pointer to the new field objects
 */
void NUBlackboard::add(FieldObjects* objects)
{
    FieldObjects* oldobjects = Objects;
    Objects = objects;
    delete oldobjects;
}

/*! @brief Adds a JobList object to the blackboard. Note that ownership of the object is now with the Blackboard. 
    @param joblist a pointer to the new job list
 */
void NUBlackboard::add(JobList* joblist)
{
    JobList* oldjobs = Jobs;
    Jobs = joblist;
    delete oldjobs;
}

/*! @brief Adds a GameInformation object to the blackboard. Note that ownership of the object is now with the Blackboard. 
    @param gameinfo a pointer to the new game information object
 */
void NUBlackboard::add(GameInformation* gameinfo)
{
    GameInformation* oldgame = GameInfo;
    GameInfo = gameinfo;
    delete oldgame;
}

/*! @brief Adds a TeamInformation object to the blackboard. Note that ownership of the object is now with the Blackboard. 
    @param teaminfo a pointer to the new team information data
 */
void NUBlackboard::add(TeamInformation* teaminfo)
{
    TeamInformation* oldteam = TeamInfo;
    TeamInfo = teaminfo;
    delete oldteam;
}


