/*! @file GoalKeeperProvider.h
    @brief Declaration of a behaviour provider for testing the goal keeper behaviour
 
    @class GoalKeeperProvider
    @brief A special behaviour for developing the goal keeper
 

    @author Jason Kulk
 
  Copyright (c) 2011 Jason Kulk
 
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

#ifndef SLIPTEST_PROVIDER_H
#define SLIPTEST_PROVIDER_H

#include "../BehaviourProvider.h"
#include "Motion/Tools/MotionScript.h"

class SlipTestProvider : public BehaviourProvider
{
public:
    SlipTestProvider(Behaviour* manager);
    ~SlipTestProvider();
protected:
    void doBehaviour();

private:
    
    float m_trans_speed,m_trans_direction,m_turn_speed,m_start_x,m_start_y,m_start_heading,m_odomX,m_odomY,m_odomB,m_av_count;
    float m_slip_matrix[3][3];
    bool m_return_to_start;
    double m_run_start;
    int samplePointIndex;
    std::vector<float> hist_odomX,hist_odomY,hist_odomB,hist_gpsX,hist_gpsY,hist_gpsB;
};


#endif

