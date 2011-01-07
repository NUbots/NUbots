/*! @file BearWalk.h
    @brief Declaration of the Bear's walk class
 
    @class BearWalk
    @brief A module to provide locomotion for the Bear
 
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

#ifndef BEARWALK_H
#define BEARWALK_H

#include "Motion/NUWalk.h"
#include "Infrastructure/NUSensorsData/NUSensorsData.h"
#include "Infrastructure/NUActionatorsData/NUActionatorsData.h"

#include <fstream>
using namespace std;

class BearWalk : public NUWalk
{
public:
    BearWalk(NUSensorsData* data, NUActionatorsData* actions);
    ~BearWalk();
protected:
    void doWalk();
private:
    void calculateGaitPhase();
    
    void updateActionatorsData();
public:
protected:
private:
    float m_gait_phase;                     //!< a variable between 0 and 1 representing the fraction completed of the current gait cycle
    double m_previous_time;                 //!< the previous time doWalk was called in ms
    
    // Front legs
    vector<float> m_left_front_angles;
    vector<float> m_right_front_angles;
    
    // Back legs
    vector<float> m_left_back_angles;
    vector<float> m_right_back_angles;
    
    // Torso
    vector<float> m_torso_angles;
};

#endif

