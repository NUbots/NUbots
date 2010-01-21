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
#include <iostream>
#include <fstream>
#include <sstream>
using namespace std;

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
    m_optimiser = new WalkOptimiser(m_walk_parameters);

    int playernum, teamnum;
    p_platform->getNumber(playernum);
    p_platform->getTeamNumber(teamnum);
    m_optimiser = new WalkOptimiser(m_walk_parameters);
    // check to see if there is an existing saved optimiser. I do this by trying to open it
    stringstream filenamestream;
    filenamestream << "optimiser" << teamnum << playernum << ".log";
    m_saved_optimiser_filename = filenamestream.str();
    loadOptimiser();
    
    // specify respawn location based on the player and team number
    m_respawn_x = -270;
    m_respawn_bearing = 0;
    if (teamnum == 0)
    {
        if (playernum == 1)
            m_respawn_y = 150;
        else
            m_respawn_y = 50;
    }
    else 
    {
        if (playernum == 1)
            m_respawn_y = -50;
        else
            m_respawn_y = -150;
    }
    
    // start the behaviour in the initial state
    m_state = Initial;
    m_previous_state = m_state;
    m_target_trial_duration = 20000;
}

WalkOptimiserBehaviour::~WalkOptimiserBehaviour()
{
    if (m_optimiser != NULL)
        delete m_optimiser;
}

/*! @brief Processes the sensor data, and produces actions (if necessary)
    @param data the sensorsdata
    @param actions the actionatorsdata
 */
void WalkOptimiserBehaviour::process(NUSensorsData* data, NUActionatorsData* actions)
{   
    if (data == NULL || actions == NULL)
        return;
    m_data = data;
    m_actions = actions;
    
    if (m_state == Trial)
        runTrial();
}

/*! @brief Jobs created by the WalkOptimiserBehaviour are added here.
    @param joblist the list of jobs to which job(s) will be added
 */
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
            if (m_target_speed < m_walk_parameters[0])
                m_target_speed += 0.03;
            else
                startTrial();

            speed[0] = m_target_speed;
            walkjob->setSpeed(speed);
        }
        else if (m_state == Trial)        // wait until we have enough data then tick the optimiser
        {
            if ((m_data->CurrentTime - m_trial_start_time) > m_target_trial_duration)
                finishTrial();
        }
    }
}

/*! @brief Teleports the robot back to its starting position. The robot is always placed upright
 */
void WalkOptimiserBehaviour::respawn()
{
    m_target_speed = 0;
    m_state = Start;
    m_actions->addTeleportation(m_data->CurrentTime, m_respawn_x, m_respawn_y, m_respawn_bearing);
    m_optimiser->getNewParameters(m_walk_parameters);
    m_walk->setWalkParameters(m_walk_parameters);
}

/*! @brief Start the trial
 */
void WalkOptimiserBehaviour::startTrial()
{
    m_state = Trial;
    m_trial_start_time = m_data->CurrentTime;
    static vector<float> gps(3,0);
    m_data->getGPSValues(gps);
    m_trial_start_x = gps[0];
    m_trial_start_y = gps[1];
    m_trial_energy_used = 0;
}

/*! @brief Run the trial.
 */
void WalkOptimiserBehaviour::runTrial()
{
    static vector<float> previouspositions;
    static vector<float> positions;
    static vector<float> torques;
    perturbRobot();
    m_data->getJointPositions(NUSensorsData::BodyJoints, positions);
    m_data->getJointTorques(NUSensorsData::BodyJoints, torques);
    
    if (previouspositions.size() != 0)
    {
        for (int i=0; i<positions.size(); i++)
            m_trial_energy_used += fabs(torques[i]*(positions[i] - previouspositions[i]));
    }
    
    previouspositions = positions;
}

/*! @brief Perturbs the robot while it is walking
 
    It seems that the head does not exert any force when it is accelerating on the torso,
    so the only effect of moving the head is a shift in the centre of mass.
 
    I might be able to perturb the robot by adding small signals into the ankle or hips.
    I probably only want to perturb the feet that are on the ground.
 
    So, what sort of signals should I add to determine robustness?
        - short strong ones to simulate pushes.
        - longer ones to simulate running into something.
 
    Do I want to try and increase the size of the pushes until it falls over! This is fine in the simulator,
    but not fine in hardware! Well I could make it fine in hardware, by changing the definition of a fall!
 */
void WalkOptimiserBehaviour::perturbRobot()
{
    // so lets start with something easy:
    //      a simple impulse to the ankle roll

    const double duration = 200;
    const float magnitude = 1.0;
    static int stepcount = 0;
    static float target = 0;
    static float stiffness = 0;
    static float leftforce, rightforce;
    static float leftimpacttime, rightimpacttime;
    m_data->getFootForce(NUSensorsData::LeftFoot, leftforce);
    m_data->getFootForce(NUSensorsData::RightFoot, rightforce);
    
    if (m_data->footImpact(NUSensorsData::LeftFoot, leftimpacttime) || m_data->footImpact(NUSensorsData::RightFoot, rightimpacttime))
        stepcount++;
    
    if (stepcount%3 == 0)           // perturb the robot on every third step
    {   
        cout << "Robot will be perturbed on this step" << endl;
        float steptime = fabs(leftimpacttime - rightimpacttime);
        // estimate the time to perturb the robot
        float perturbationtime = 0;
        if (leftimpacttime > rightimpacttime)       // if the we are on the left foot
            perturbationtime = leftimpacttime + 0.5*steptime;
        else
            perturbationtime = rightimpacttime + 0.5*steptime;
        
        if (m_data->CurrentTime - perturbationtime > 0 && m_data->CurrentTime - perturbationtime < duration)
        {
            cout << "The robot is being perturbed NOW" << endl;
            if (leftimpacttime > rightimpacttime) 
            {
                cout << "Perturbing the left leg" << endl;
                m_data->getJointTarget(NUSensorsData::LAnkleRoll, target);
                m_data->getJointStiffness(NUSensorsData::LAnkleRoll, stiffness);
                m_actions->addJointPosition(NUSensorsData::LAnkleRoll, 0, target - (100*magnitude/stiffness), 0, stiffness);
                m_data->getJointTarget(NUSensorsData::LHipRoll, target);
                m_data->getJointStiffness(NUSensorsData::LHipRoll, stiffness);
                //m_actions->addJointPosition(NUSensorsData::LHipRoll, 0, target + (100*magnitude/stiffness), 0, stiffness);
            }
            else
            {
                cout << "Perturbing the right leg" << endl;
                m_data->getJointTarget(NUSensorsData::RAnkleRoll, target);
                m_data->getJointStiffness(NUSensorsData::RAnkleRoll, stiffness);
                m_actions->addJointPosition(NUSensorsData::RAnkleRoll, 0, target - (100*magnitude/stiffness), 0, stiffness);
                m_data->getJointTarget(NUSensorsData::RHipRoll, target);
                m_data->getJointStiffness(NUSensorsData::RHipRoll, stiffness);
                //m_actions->addJointPosition(NUSensorsData::RHipRoll, 0, target + (100*magnitude/stiffness), 0, stiffness);
            }
                
        }
    }
}

/*! @brief Finish the trial, that is calculate the performance, tick the optimiser, save the optimiser and respawn the robot
 */
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
    saveOptimiser();
    respawn();
}

/*! @brief Loads a saved WalkOptimiser from a file. If there is no file then we start from scratch
 */
void WalkOptimiserBehaviour::loadOptimiser()
{
    ifstream savedoptimiser(m_saved_optimiser_filename.c_str(), ios_base::in);
    if (savedoptimiser.is_open())
    {   // if it exists start from where we left off
        cout << "WalkOptimiserBehaviour::WalkOptimiserBehaviour. Loading previous WalkOptimiser from file." << endl;
        savedoptimiser >> (*m_optimiser);
        m_optimiser->getNewParameters(m_walk_parameters);
        m_walk->setWalkParameters(m_walk_parameters);
        m_optimiser->summaryTo(cout);
    }
}

/*! @brief Saves the WalkOptimiser to a file so that we can continue later
 */
void WalkOptimiserBehaviour::saveOptimiser()
{
    ofstream savedoptimiser(m_saved_optimiser_filename.c_str(), ios_base::trunc);
    if (savedoptimiser.is_open())
        savedoptimiser << (*m_optimiser);
    m_optimiser->summaryTo(cout);
}