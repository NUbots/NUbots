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

#ifndef TESTKEEPER_PROVIDER_H
#define TESTKEEPER_PROVIDER_H

#include "../BehaviourProvider.h"
#include "Motion/Tools/MotionScript.h"

class TestKeeperProvider : public BehaviourProvider
{
public:
    TestKeeperProvider(Behaviour* manager);
    ~TestKeeperProvider();
protected:
    void doBehaviour();

private:
    
    float m_block_time;
    MotionScript m_script;
    bool doSave(float maxInterceptTime=7,float interceptErrorFraction=0.5);
};


#endif

