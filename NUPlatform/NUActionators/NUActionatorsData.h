/*! @file NUActionatorsData.h
    @brief Declaration of a actionator data class to store actionator data in a platform independent way
    @author Jason Kulk
 
    @class NUActionatorsData
    @brief A actionator class to store actionator data in a platform independent way
 
    @author Jason Kulk
 
  Copyright (c) 2009 Jason Kulk
 
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

#ifndef NUACTIONATORSDATA_H
#define NUACTIONATORSDATA_H

#include "actionator_t.h"

#include <vector>
#include <string>
using namespace std;

class NUActionatorsData
{
public:
    NUActionatorsData();
    ~NUActionatorsData();

    void setAvaliableActionators(const vector<string>& actionators);
private:
    void addActionator(actionator_t** p_actionator, string actionatorname, actionator_id_t actionatorid);
    
public:
    // NAMED ACTIONATORS
    actionator_t* JointPositions;
    actionator_t* JointVelocities;
    actionator_t* CameraControl;
    actionator_t* LedLEar;
    actionator_t* LedREar;
    actionator_t* LedLEye;
    actionator_t* LedREye;
    actionator_t* LedLFoot;
    actionator_t* LedRFoot;
    actionator_t* Sound;
    
private:
    vector<actionator_t*> m_actionators;
    
};

#endif

