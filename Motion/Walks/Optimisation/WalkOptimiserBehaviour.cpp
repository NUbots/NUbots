/*! @file WalkOptimiserBehaviour.cpp
    @brief Implementation of WalkOptimiserBehaviour class

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

#include "WalkOptimiserBehaviour.h"

#include "NUPlatform/NUSystem.h"
#include "Tools/debug.h"

#include <math.h>

//! @todo TODO: put M_PI and NORMALISE somewhere else
#ifndef M_PI
    #define M_PI 3.1415926535
#endif

template <class T>
inline T NORMALISE(T theta){
    return atan2(sin(theta), cos(theta));
}

/*! @brief Constructs a walk optimiser
    @param p_platform I need this to access the player number and team number. I think that should be moved into sensorsdata at some stage
 */
WalkOptimiserBehaviour::WalkOptimiserBehaviour(NUPlatform* p_platform, NUWalk* p_walk)
{
    // get initial walk parameters from the walk engine itself.
    m_walk = p_walk;
    m_walk->getWalkParameters(m_walk_parameters);
    m_optimiser = new WalkOptimiser(m_walk_parameters, false);
    
    // specify respawn location based on the player and team number
    m_respawn_x = -270;
    m_respawn_bearing = 0;
    int playernum, teamnum;
    p_platform->getNumber(playernum);
    p_platform->getTeamNumber(teamnum);
    if (teamnum == 0)
    {
        if (playernum == 1)
        {
            m_respawn_y = 150;
            m_max_target_speed = 3;
        }
        else
        {
            m_respawn_y = 50;
            m_max_target_speed = 6;
        }
    }
    else 
    {
        if (playernum == 1)
        {
            m_respawn_y = -50;
            m_max_target_speed = 9;
        }
        else
        {
            m_respawn_y = -150;
            m_max_target_speed = 12;
        }
    }
    
    // start the behaviour in the initial state
    m_state = Initial;
    m_previous_state = m_state;
}

WalkOptimiserBehaviour::~WalkOptimiserBehaviour()
{
    if (m_optimiser != NULL)
        delete m_optimiser;
}


void WalkOptimiserBehaviour::process(NUSensorsData* data, NUActionatorsData* actions)
{   
    if (data == NULL || actions == NULL)
        return;
    m_data = data;
    m_actions = actions;
    
    if (m_state == Trial)
        runTrial();
}

void WalkOptimiserBehaviour::process(JobList& joblist)
{
    if (m_data == NULL || m_actions == NULL)
        return;
    
    m_previous_state = m_state;
    if (m_data->isFallen())
    {
        if (m_state == Trial)
            finishTrial();
        else
            respawn();
    }
    else 
    {
        if (m_state == Initial)         // wait until 'playing'
        {
            if (m_data->CurrentTime > 15000)
                respawn();
        }
        else if (m_state == Start)      // accelerate up to 'target' speed
        {
            static vector<float> speed(3,0);
            static WalkJob* walkjob = new WalkJob(speed);
            joblist.addMotionJob(walkjob);
            if (m_target_speed < m_max_target_speed)
                m_target_speed += 0.03;
            else
                startTrial();

            speed[0] = m_target_speed;
            walkjob->setSpeed(speed);
        }
        else if (m_state == Trial)        // wait until we have enough data then tick the optimiser
        {
            if ((m_data->CurrentTime - m_trial_start_time) > 20000)
                finishTrial();
        }
    }
}

void WalkOptimiserBehaviour::respawn()
{
    m_target_speed = 0;
    m_state = Start;
    m_actions->addTeleportation(m_data->CurrentTime, m_respawn_x, m_respawn_y, m_respawn_bearing);
    m_optimiser->getNewParameters(m_walk_parameters);
    m_walk->setWalkParameters(m_walk_parameters);
}

void WalkOptimiserBehaviour::startTrial()
{
    m_state = Trial;
    m_trial_start_time = m_data->CurrentTime;
    static vector<float> gps(3,0);
    m_data->getGPSValues(gps);
    m_trial_start_x = gps[0];
    m_trial_start_y = gps[1];
    m_trial_energy_used = 0;
    cout << "Starting Trial at [" << gps[0] << "," << gps[1] << "] " << m_trial_start_time << endl;
}

void WalkOptimiserBehaviour::runTrial()
{
    static vector<float> previouspositions;
    static vector<float> positions;
    static vector<float> torques;
    m_data->getJointPositions(NUSensorsData::BodyJoints, positions);
    m_data->getJointTorques(NUSensorsData::BodyJoints, torques);
    
    if (previouspositions.size() != 0)
    {
        for (int i=0; i<positions.size(); i++)
            m_trial_energy_used += fabs(torques[i]*(positions[i] - previouspositions[i]));
    }
    
    previouspositions = positions;
}

void WalkOptimiserBehaviour::finishTrial()
{
    m_state = Start;
    static vector<float> gps(3,0);
    m_data->getGPSValues(gps);
    float distance = sqrt(pow(gps[0] - m_trial_start_x,2) + pow(gps[1] - m_trial_start_y,2));
    float time = (m_data->CurrentTime - m_trial_start_time)/1000.0;
    if (time < 15)
        time = 15;
    float speed = distance/time;
    float cost = (2*m_trial_energy_used+20*time)*9.81*4.8/(distance*100);       // assume CPU draws 20W and the motor gears are 50% efficient
    m_optimiser->tick(cost, m_walk_parameters);
    respawn();
    cout << "Finished Trial with speed: " << speed << " cost " << cost << endl;
}
