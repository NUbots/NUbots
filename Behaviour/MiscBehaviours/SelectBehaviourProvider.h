/*! @file SelectBehaviourProvider.h
    @brief Declaration of simple behaviour selection class
 
    @class SelectBehaviourProvider
    @brief A simple behaviour selection class. Behaviours can be changed via the left (up) and right (down) buttons,
           and then selected using the chest (middle)

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

#ifndef SELECTBEHAVIOURPROVIDER_H
#define SELECTBEHAVIOURPROVIDER_H

#include "../BehaviourProvider.h"

class JobList;
class NUSensorsData;
class NUActionatorsData;
class FieldObjects;
class GameInformation;
class TeamInformation;

#include <vector>
#include <string>

class SelectBehaviourProvider : public BehaviourProvider
{
public:
    SelectBehaviourProvider(Behaviour* manager);
    ~SelectBehaviourProvider();
protected:
    void doBehaviour();
private:
    void doIntroduction();
    void voiceCurrentSelection();
private:
    bool m_introduction_done;
    int m_selection_index;
    std::vector<std::string> m_available_behaviours;
};


#endif

