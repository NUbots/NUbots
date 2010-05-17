/*! @file ChaseBallProvider.h
    @brief Declaration of simple chase ball behaviour for testing and demonstration purposes 
 
    @class ChaseBallProvider
    @brief A simple chase ball behaviour for testing and demonstration purposes 

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

#ifndef CHASEBALLBEHAVIOUR_H
#define CHASEBALLBEHAVIOUR_H

#include "../BehaviourProvider.h"

class JobList;
class NUSensorsData;
class NUActionatorsData;
class FieldObjects;
class GameInformation;
class TeamInformation;

#include <vector>
#include <string>

class ChaseBallProvider : public BehaviourProvider
{
public:
    ChaseBallProvider(Behaviour* manager, bool pauseable = true);
    ~ChaseBallProvider();
    
protected:
    void doBehaviour();
private:
    void TrackPoint(float sensoryaw, float sensorpitch, float elevation, float bearing, float centreelevation = 0, float centrebearing = 0);
    void Pan();
    
    bool m_pauseable;
    bool m_paused;
};


#endif

