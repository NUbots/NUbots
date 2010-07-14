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
#include "Infrastructure/FieldObjects/FieldObjects.h"
#include "Infrastructure/Jobs/Jobs.h"
#include "Infrastructure/GameInformation/GameInformation.h"
#include "Infrastructure/TeamInformation/TeamInformation.h"

#include "NUPlatform/NUSystem.h"
#include "debug.h"

NUBlackboard* Blackboard = 0;

/*! @brief Construct the NUBlackboard --- the global data store. 
    The NUSystem and NUPlatform need to be created before the NUBlackboard
*/
NUBlackboard::NUBlackboard()
{
    Blackboard = this;
    Sensors = new NUSensorsData();
    Actions = new NUActionatorsData();
    Objects = new FieldObjects();
    Jobs = new JobList();
    GameInfo = new GameInformation(System->getPlayerNumber(), System->getTeamNumber());
    TeamInfo = new TeamInformation(System->getPlayerNumber(), System->getTeamNumber());
    
    // create the locks protecting the objects, gameinfo and teaminfo pointers
    // These objects are potentially behaviour dependent, and we can switch behaviours on the fly
    // Thus, we need to provider a safe way for behaviour to swap the pointers to the behaviour specific ones
    int err;
    err = pthread_mutex_init(&m_objects_pointer_lock, NULL);
    if (err != 0)
        errorlog << "NUBlackboard::NUBlackboard() Failed to create m_objects_pointer_lock." << endl;
    
    err = pthread_mutex_init(&m_gameinfo_pointer_lock, NULL);
    if (err != 0)
        errorlog << "NUBlackboard::NUBlackboard() Failed to create m_game_info_pointer_lock." << endl;
    
    err = pthread_mutex_init(&m_teaminfo_pointer_lock, NULL);
    if (err != 0)
        errorlog << "NUBlackboard::NUBlackboard() Failed to create m_team_info_pointer_lock." << endl;
}

NUBlackboard::~NUBlackboard()
{
    delete Sensors;
    Sensors = 0;
    delete Actions;
    Actions = 0;
    setObjects(0);      // thread-safe delete of Objects
    delete Jobs;
    Jobs = 0;
    setGameInfo(0);     // thread-safe delete of GameInfo
    setTeamInfo(0);     // thread-safe delete of TeamInfo
}

/*! @brief Sets the blackboard's Objects pointer to the given one 
    
    The Blackboard now owns objects and it should not be free'd.
    @param objects a pointer to the new Objects container.
 */
void NUBlackboard::setObjects(FieldObjects* objects)
{
    pthread_mutex_lock(&m_objects_pointer_lock);
    delete Objects;
    Objects = objects;
    pthread_mutex_unlock(&m_objects_pointer_lock);
}

/*! @brief Sets the blackboard's GameInfo pointer to the given one 
 
    The Blackboard now owns gameinfo and it should not be free'd.
    @param gameinfo a pointer to the new GameInfo container.
 */
void NUBlackboard::setGameInfo(GameInformation* gameinfo)
{
    pthread_mutex_lock(&m_gameinfo_lock);
    delete GameInfo;
    GameInfo = gameinfo;
    pthread_mutex_unlock(&m_gameinfo_lock);
}

/*! @brief Sets the blackboard's TeamInfo pointer to the given one 
 
    The Blackboard now owns teaminfo and it should not be free'd.
    @param teaminfo a pointer to the new TeamInfo container.
 */
void NUBlackboard::setTeamInfo(TeamInformation* teaminfo)
{
    pthread_mutex_lock(&m_teaminfo_lock);
    delete TeamInfo;
    TeamInfo = teaminfo;
    pthread_mutex_unlock(&m_teaminfo_lock);
}
