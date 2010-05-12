/*! @file Behaviour.h
    @brief Declaration of top-level behaviour class
 
    @class Behaviour
    @brief The top-level behaviour class

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

#ifndef BEHAVIOUR_H
#define BEHAVIOUR_H

#include "AbstractBehaviour.h"

class JobList;
class NUSensorsData;
class NUActionatorsData;
class FieldObjects;
class GameInformation;
class TeamInformation;

#include <vector>
#include <string>

class Behaviour : public AbstractBehaviour 
{
public:
    Behaviour();
    ~Behaviour();
    
protected:
    void doBehaviour();
private:
    void doIntroduction();
    void voiceCurrentSelection();
    
private:
    bool m_introduction_done;               //!< true if the introduction has been played
    int m_selection_index;                  //!< the index into m_available_behaviours for the currently 'selected' behaviour
    static std::vector<std::string> m_avaliable_behaviours;     //!< a list containing the avaiable behaviours
};


#endif

