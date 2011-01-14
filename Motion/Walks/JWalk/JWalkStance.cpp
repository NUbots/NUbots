/*! @file JWalkStance.cpp
    @brief Implementation of a jwalk's stance state
 
    @author Jason Kulk
 
  Copyright (c) 2010, 2011 Jason Kulk
 
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

#include "JWalkStance.h"

JWalkStance::JWalkStance(const NUData::id_t& leg) : JWalkState("Stance", leg)
{
    
}

JWalkStance::~JWalkStance()
{
    
}

void JWalkStance::doIt()
{
    // what do we do in the stance phase
    // (a) balance
    //			maintain CoP under the ankle
    //			maintain angleX and angleY to be zero
    // (b) bring foot to push position
    
    static bool first = true;
    static double firsttime = 0;
    if (first)
    {	// this is a hack to get the NAO on one leg real quick
        firsttime = Blackboard->Actions->CurrentTime;
        first = false;
        vector<float> position(6,0);
        position[0] = -0.20;
        position[4] = 0.20;
        Blackboard->Actions->add(NUData::RLeg, Blackboard->Actions->CurrentTime + 300, position);
        Blackboard->Actions->add(NUData::LLeg, Blackboard->Actions->CurrentTime + 300, position);
        position[0] = -0.15;
        position[4] = 0.30;
        Blackboard->Actions->add(NUData::LLeg, Blackboard->Actions->CurrentTime + 600, position);
    }
    
    if (Blackboard->Actions->CurrentTime < firsttime + 700)
        return;
    
    // Lets try some balance
    vector<float> cop;
    if (Blackboard->Sensors->getCoP(Leg, cop))
    {
        // a simple proportional controller, has to be relative to the current position
        // this doesn't work well, and it does not work at all when standing on two legs.
        // the problem is dynamic and steady-state relationships between the cop and ankle angles are opposite.
        float errorx = 0 - cop[0];
        float errory = 0 - cop[1];
        float rgain = 0.00025;		// 0.001 in webots
        float pgain = 0.00025;
        
        // a platform independent way to get the ankle roll and ankle pitch??
        float ankleroll = 0;
        float anklepitch = 0;
        if (Leg == NUData::LLeg)
        {
            Blackboard->Sensors->getTarget(NUData::LAnkleRoll, ankleroll);
            Blackboard->Sensors->getTarget(NUData::LAnklePitch, anklepitch);
            Blackboard->Actions->add(NUData::LAnkleRoll, JWalkBlackboard->CurrentTime, ankleroll + rgain*errory);
            Blackboard->Actions->add(NUData::LAnklePitch, JWalkBlackboard->CurrentTime, anklepitch - pgain*errorx);
            cout << JWalkBlackboard->CurrentTime << " Left ex:" << errorx << " ey:" << errory << " ar:" << ankleroll << " ap:" << anklepitch << endl;
        }
        else
        {
            Blackboard->Sensors->getTarget(NUData::RAnkleRoll, ankleroll);
            Blackboard->Sensors->getTarget(NUData::RAnklePitch, anklepitch);
            //Blackboard->Actions->add(NUData::RAnkleRoll, JWalkBlackboard->CurrentTime, ankleroll + rgain*errory);
            //Blackboard->Actions->add(NUData::RAnklePitch, JWalkBlackboard->CurrentTime, anklepitch - pgain*errorx);
            //cout << JWalkBlackboard->CurrentTime << " Right ex:" << errorx << " ey:" << errory << " ar:" << ankleroll << " ap:" << anklepitch << endl;
        }
            
    }
    vector<float> orientation;
    if (Blackboard->Sensors->getOrientation(orientation))
    {
        // do something
    }
    
}

JWalkStance* JWalkStance::next()
{
    return this;
}

