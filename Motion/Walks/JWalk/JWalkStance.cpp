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

#include "Motion/Tools/PIDController.h"

JWalkStance::JWalkStance(const NUData::id_t& leg) : JWalkState("Stance", leg)
{
    // A note about gains. The gains can be much higher in webots, and the gains depending on the hardness of the surface.
    // webots->hard 0.23
    																									// roll: 0.41, 0.09, 0.44   0.24, 0.014, 0.32     pitch: 0.22, meh, 0.25
    m_lroll_pid = new PIDController("JWalkStance.lroll", 0.018, 12.2e-5, 2.5, 4, -0.78, 0.78);			// webots: 0.018, 12.2e-5, 2.5, 4, -0.78, 0.78 on soft ground: 0.007, 3.0e-6, 1.11, 4, -0.78, 0.78 on hard ground: 0.004, 0.5e-6, 0.79, 4, -0.78, 0.78
    m_lpitch_pid = new PIDController("JWalkStance.lpitch", 0.008, 6.7e-5, 0.6, 4, -0.78, 0.78);			// webots: 0.009, 6.7e-5, 0.6, 4, -0.78, 0.78 on hard ground: 0.002, 0.5e-6, 0.15, 4, -0.78, 0.78
    
    m_l_hiproll_pid = new PIDController("JWalkStance.lhroll", 0.035, 5.1e-5, 9, 4, -0.78, 0.78);		// webots: 0.05, 1.35e-4, 3.3, 4, -0.78, 0.78 on hard ground:
    m_l_hippitch_pid = new PIDController("JWalkStance.lhpitch", 0.025, 1.0e-4, 1.4, 4, -0.78, 0.78);	// webots: 0.025, 1.0e-4, 1.4, 4, -0.78, 0.78
    
    m_rroll_pid = new PIDController("JWalkStance.rroll", 0.004, 0.5e-6, 0.79, 4, -0.78, 0.78);
    m_rpitch_pid = new PIDController("JWalkStance.rpitch", 0.002, 0.5e-6, 0.15, 4, -0.78, 0.78);
}

JWalkStance::~JWalkStance()
{
    delete m_lroll_pid;
    m_lroll_pid = 0;
    delete m_lpitch_pid;
    m_lpitch_pid = 0;
    delete m_rroll_pid;
    m_rroll_pid = 0;
    delete m_rpitch_pid;
    m_rpitch_pid = 0;
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
    static bool lolwut = true;
    static float ankleroll, anklepitch, hiproll, hippitch;
    if (first)
    {	// this is a hack to get the NAO on one leg real quick
        firsttime = Blackboard->Actions->CurrentTime;
        first = false;
        vector<float> position(6,0);
        position[0] = -0.25;
        position[4] = 0.25;
        position[5] = -0.1;
        Blackboard->Actions->add(NUData::RLeg, Blackboard->Actions->CurrentTime + 400, position);
        Blackboard->Actions->add(NUData::LLeg, Blackboard->Actions->CurrentTime + 400, position);
        //position[4] = 0.40;
        Blackboard->Actions->add(NUData::LLeg, Blackboard->Actions->CurrentTime + 700, position);
        position[1] = -0.5;
        position[3] = 1.0;
        position[5] = -0.5;
        Blackboard->Actions->add(NUData::RLeg, Blackboard->Actions->CurrentTime + 700, position);
    }
    
    if (Blackboard->Actions->CurrentTime < firsttime + 1000)
        return;
    
    // Lets try some balance, lets be honest, and say that this isn't really going to work, because it isn't fast enough
    // I can't do it using feedback alone, and this setup can't be used as an offset
    vector<float> cop;
    vector<float> othercop;
    if (Blackboard->Sensors->getCoP(Leg, cop))
    {
        if (lolwut)
        {
            Blackboard->Sensors->getTarget(NUData::LAnkleRoll, ankleroll);
            Blackboard->Sensors->getTarget(NUData::LAnklePitch, anklepitch);
            Blackboard->Sensors->getTarget(NUData::LHipRoll, hiproll);
            Blackboard->Sensors->getTarget(NUData::LHipPitch, hippitch);
            lolwut = false;
        }
        
        if (Leg == NUData::LLeg)
        {
            m_lroll_pid->set(0);
            m_lpitch_pid->set(0);
            m_l_hiproll_pid->set(0);
            m_l_hippitch_pid->set(0);
            Blackboard->Actions->add(NUData::LAnkleRoll, JWalkBlackboard->CurrentTime, ankleroll + m_lroll_pid->doIt(Blackboard->Actions->CurrentTime, cop[1]));
            //Blackboard->Actions->add(NUData::LAnklePitch, JWalkBlackboard->CurrentTime, m_lpitch_pid->doIt(Blackboard->Actions->CurrentTime, -cop[0], anklepitch));
            Blackboard->Actions->add(NUData::LHipRoll, JWalkBlackboard->CurrentTime, hiproll + m_l_hiproll_pid->doIt(Blackboard->Actions->CurrentTime, -cop[1]));
            //Blackboard->Actions->add(NUData::LHipPitch, JWalkBlackboard->CurrentTime, m_l_hippitch_pid->doIt(Blackboard->Actions->CurrentTime, -cop[0], hippitch));
            cout << JWalkBlackboard->CurrentTime << " Left ex:" << cop[0] << " ey:" << cop[1] << " ar:" << ankleroll << " hr:" << hiproll << endl;
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

