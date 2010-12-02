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
#include "debug.h"
#include "debugverbositynumotion.h"

#include <math.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics.hpp>
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
    m_metric_type = Cost;                                                              //<<<<<<<<<-------------------- Don't forget to set this line to the right metric!!
    if (m_metric_type == Speed || m_metric_type == SpeedAndPushes)
        m_optimiser = new WalkOptimiser(m_walk_parameters, false);
    else
        m_optimiser = new WalkOptimiser(m_walk_parameters);

    int playernum, teamnum;
    p_platform->getNumber(playernum);
    p_platform->getTeamNumber(teamnum);
    // firstly we look to see if there are any sets of parameters that need assessing!
    stringstream assessparameters_filenamestream;
    assessparameters_filenamestream << "../assess_this" << teamnum << playernum << ".log";
    m_assessparameters_filename = assessparameters_filenamestream.str();
    loadAssessor();
    // check to see if there is an existing saved optimiser. I do this by trying to open it
    stringstream savedoptimiser_filenamestream;
    savedoptimiser_filenamestream << "../previous_optimiser" << teamnum << playernum << ".log";
    m_saved_optimiser_filename = savedoptimiser_filenamestream.str();
    loadOptimiser();
    // init log files. I always want to write, and I always want to append to the files
    stringstream parameter_filenamestream;
    parameter_filenamestream << "../parameters" << teamnum << playernum << ".log";
    m_parameter_log.open(parameter_filenamestream.str().c_str(), ios_base::out | ios_base::app);
    stringstream performance_filenamestream;
    performance_filenamestream << "../performance" << teamnum << playernum << ".log";
    m_performance_log.open(performance_filenamestream.str().c_str(), ios_base::out | ios_base::app);
    
    
    m_current_time = 0;
    m_previous_time = 0;
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
    m_target_speed = 0.0;
    
    m_trial_out_of_field = false;
    m_trial_energy_used = 0;
    m_trial_perturbation_mag = 0;
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
    
    if (m_state == MeasureCost)
        measureCost();
    else if (m_state == MeasureRobust)
        measureRobust();
}

/*! @brief Jobs created by the WalkOptimiserBehaviour are added here.
 
    The behaviour has 4 states:
        1. Initial where we are just waiting for the simulator to go into 'playing'
        3. MeasureCost where we actually perform the measurement of the cost
        5. MeasureRobust where we actually perform the measurement of the robustness
        6. Teleport where the robot is teleported back to its initial position
 
    @param joblist the list of jobs to which job(s) will be added
 */
void WalkOptimiserBehaviour::process(JobList& joblist)
{
    static int fallencount = 0;
    static vector<float> speed(3,0);
    static WalkJob* job = new WalkJob(speed);
    
    if (m_data == NULL || m_actions == NULL)
        return;

    m_previous_state = m_state;
    m_previous_time = m_current_time;
    m_current_time = m_data->CurrentTime;
    
    if (m_state == Initial)
    {   // In the initial state we wait until the simulator puts the game in 'playing' and I have time to score a goal!
        if (m_data->CurrentTime > 15000)
        {
            m_state = MeasureCost;
            teleport();
        }
    }
    else if (m_state == Teleport)
    {   // In the teleport state we wait until the robot has stopped
        // We then respawn, and then wait another 0.5s before proceeding to the next state
        
        // handle the deacceleration (now done in the walk engine itself)
        m_target_speed = 0;
        
        // wait until the robot comes to rest
        static vector<float> speed(3,0);
        static double stoppedtime = 0;
        static bool stopped = false;
        m_walk->getCurrentSpeed(speed);
        if (stopped == false && speed[0] == 0)
        {
            stoppedtime = m_current_time;
            stopped = true;
        }
            
        // handle the respawn call (being careful to only call it once because it really slows webots down)
        static bool respawn_called = false;
        if (respawn_called == false && stopped == true && (m_current_time - stoppedtime) > 500)
        {
            respawn();
            respawn_called = true;
        }
        
        // handle the timely progress to the next state
        if (respawn_called == true && stopped == true && (m_current_time - stoppedtime) > 1000)
        {
            m_state = m_next_state;
            m_target_speed = 0;
            fallencount = 0;
            respawn_called = false;
            stopped = false;
            if (m_trial_out_of_field == false)
            {
                if (m_state == MeasureCost)
                    startCostTrial();
                else if (m_state == MeasureRobust)
                    startRobustTrial();
            }
            else
                m_trial_out_of_field = false;
        }
    }
    else if (m_data->isFallen())
    {   // In the fallen state, we wait to make sure we are actually fallen, and then finish the current trial
        fallencount++;
        if (fallencount > 9)
        {
            if (m_state == MeasureCost)
                finishMeasureCost();
            else if (m_state == MeasureRobust)
                finishMeasureRobust();
        }
    }
    else 
    {
        fallencount = 0;                // reset the fallen count when we are not falling
        
        // handle accelerating walks up to maximum speed (now done in the walk engine itself)
        m_target_speed = m_walk_parameters[0];
        
        // look for terminating distances while measuring walk performance
        static vector<float> gps(3,0);
        m_data->getGPSValues(gps);
        float totaldistance = sqrt(pow(gps[0] - m_respawn_x,2) + pow(gps[1] - m_respawn_y,2));
        if (m_state == MeasureCost)
        {
            if (totaldistance > 300.0)      // if walked more than 300cm begin to stop the robot
            {
                m_target_speed = 0;
                static vector<float> speeds(3,0);
                m_walk->getCurrentSpeed(speeds);
                if (speeds[0] == 0)         // once the robot has stopped finish the cost measurement
                    finishMeasureCost();
            }
        }
        else if (m_state == MeasureRobust)
        {
            if (totaldistance > 450.0)
            {
                m_trial_out_of_field = true;
                teleport();
            }
        }
    }
    speed[0] = m_target_speed;
    job->setSpeed(speed);
    joblist.addMotionJob(job);
}

/*! @brief Teleports the robot back to its starting position cleanly.
 
    This function makes sure that the robot is walking at a speed of 0 before placing the robot on the ground.
 */
void WalkOptimiserBehaviour::teleport()
{
    m_next_state = m_state;                 // I copy the desired state to next state, so when teleport finishes I know which state to go into
    m_state = Teleport;
}

/*! @brief Moves the robot back to its starting position. The robot is always placed upright
 */
void WalkOptimiserBehaviour::respawn()
{
    m_actions->addTeleportation(m_data->CurrentTime, m_respawn_x, m_respawn_y, m_respawn_bearing);
}

/*! @brief Start the cost trial
 */
void WalkOptimiserBehaviour::startCostTrial()
{
    m_trial_start_time = m_data->CurrentTime;
    m_trial_energy_used = 0;
    m_state = MeasureCost;
}

/*! @brief Start the robust trial
 */
void WalkOptimiserBehaviour::startRobustTrial()
{
    m_trial_perturbation_mag = 0.02;
    m_perturbation_direction = -1;
    m_state = MeasureRobust;
}

/*! @brief Measures the cost of transport of the robot while it is walking
 
    m_trial_energy_used is incremented by the amount of energy used in this cycle
 */
void WalkOptimiserBehaviour::measureCost()
{
    static vector<float> previouspositions;
    static vector<float> positions;
    static vector<float> torques;
    static double previoustime = 0;
    m_data->getJointPositions(NUSensorsData::BodyJoints, positions);
    m_data->getJointTorques(NUSensorsData::BodyJoints, torques);
    
    if (previouspositions.size() != 0 && (m_data->CurrentTime - previoustime) < 1000)
    {
        for (unsigned int i=0; i<positions.size(); i++)
            m_trial_energy_used += fabs(torques[i]*(positions[i] - previouspositions[i]));
    }
    previouspositions = positions;
    previoustime = m_data->CurrentTime;
}

/*! @brief Finish the cost measurement
 
    m_measured_speed and m_measured_cost are updated and the robot is respawned. The optimiser is not updated at this stage.
 */
void WalkOptimiserBehaviour::finishMeasureCost()
{
    static vector<float> gps(3,0);
    m_data->getGPSValues(gps);
    float distance = fabs(gps[0] - m_respawn_x);                                // only count the forward distance travelled
    float totaldistance = sqrt(pow(gps[0] - m_respawn_x,2) + pow(gps[1] - m_respawn_y,2));
    float time = (m_data->CurrentTime - m_trial_start_time)/1000.0;
    if (m_data->isFallen() || totaldistance < 300)
    {   // if we fall then add the time and energy it takes to get up
        time += 10;                                     // approx 10s to getup
        m_trial_energy_used += (9.81*4.8*0.3)*3;        // approx 42J to getup
    }
    
    m_trial_energy_used = 2*m_trial_energy_used;                                // the factor of two is placed here to model the motor's gearbox efficiency
    m_trial_energy_used = 21*time;                                              // we assume the CPU etc draws 21W
    m_measured_speed = distance/time;
    m_measured_cost = m_trial_energy_used*100/(9.81*4.8*distance);          
    
    //cout << "finishMeasureCost(). m_trial_energy_used: " << m_trial_energy_used << " time: " << time << " totaldistance:" << totaldistance << endl;
    
    m_state = MeasureRobust;
    teleport();
}

/*! @brief Measure the robustness of the walk
 */
void WalkOptimiserBehaviour::measureRobust()
{
    perturbRobot();
}

/*! @brief Finish the robustness measurement
 */
void WalkOptimiserBehaviour::finishMeasureRobust()
{
    m_measured_robustness = m_trial_perturbation_mag;
    
    if (m_metric_type == Speed)
        m_measured_metric = m_measured_speed;
    else if (m_metric_type == Cost)
        m_measured_metric = m_measured_cost;
    else if (m_metric_type == SpeedAndPushes)
        m_measured_metric = m_measured_speed;
    else
        m_measured_metric = m_measured_cost;
 
    if (m_assessor_running == true)
        tickAssessor();
    else
        tickOptimiser(m_measured_metric);
    
    m_state = MeasureCost;
    teleport();
}

/*! @brief Perturbs the robot while it is walking
 
    We are going to be 'perturbing' the robot by applying an additional torque to the ankles and/or hips.
 */
void WalkOptimiserBehaviour::perturbRobot()
{
    static int stepcount = 1;
    
    if (m_target_speed < m_walk_parameters[0])
    {   // be careful not to perturb the robot will it is accelerating
        stepcount = 1;
        return;
    }
    
    // increment the step count every time there is a foot impact
    if (m_data->footImpact(NUSensorsData::LeftFoot, m_left_impact_time) || m_data->footImpact(NUSensorsData::RightFoot, m_right_impact_time))
    {
        stepcount++;
        if (stepcount%3 == 0)
        {
            m_perturbation_direction = (m_perturbation_direction + 1) % 4;
            if (m_perturbation_direction == 0)
                m_trial_perturbation_mag += 0.02;
        }
    }
    
    // perturb the robot on every third step
    if (stepcount%3 == 0)
    {   
        // estimate the time to perturb the robot (assume midstep is half of the previous step period)
        float steptime = fabs(m_left_impact_time - m_right_impact_time);
        float perturbationtime = 0;
        if (m_left_impact_time > m_right_impact_time)       // if the we are on the left foot
            perturbationtime = m_left_impact_time + 0.20*steptime;
        else
            perturbationtime = m_right_impact_time + 0.20*steptime;
        
        if (m_data->CurrentTime - perturbationtime > 0 && m_data->CurrentTime - perturbationtime < 0.80*steptime)
        {
            if (m_perturbation_direction == 0)
                pushInward();
            else if (m_perturbation_direction == 1)
                pushOutward();
            else if (m_perturbation_direction == 2)
                pushForward();
            else
                pushBackward();
        }
    }
}

/*! @brief Simulates an imaginary push inwards, that is a push toward the foot thats off the ground
 */
void WalkOptimiserBehaviour::pushInward()
{
    if (m_left_impact_time > m_right_impact_time) 
    {
        pushJoint(NUSensorsData::LAnkleRoll, -m_trial_perturbation_mag);
        pushJoint(NUSensorsData::LHipRoll, +m_trial_perturbation_mag);
        pushJoint(NUSensorsData::RAnkleRoll, -m_trial_perturbation_mag);
        pushJoint(NUSensorsData::RHipRoll, +m_trial_perturbation_mag);
    }
    else
    {
        pushJoint(NUSensorsData::LAnkleRoll, +m_trial_perturbation_mag);
        pushJoint(NUSensorsData::LHipRoll, -m_trial_perturbation_mag);
        pushJoint(NUSensorsData::RAnkleRoll, +m_trial_perturbation_mag);
        pushJoint(NUSensorsData::RHipRoll, -m_trial_perturbation_mag);
    }
}

/*! @brief Simulates an imaginary push outwards, that is a push toward the foot thats on the ground
 */
void WalkOptimiserBehaviour::pushOutward()
{
    if (m_left_impact_time > m_right_impact_time) 
    {
        pushJoint(NUSensorsData::LAnkleRoll, +m_trial_perturbation_mag);
        pushJoint(NUSensorsData::LHipRoll, -m_trial_perturbation_mag);
        pushJoint(NUSensorsData::RAnkleRoll, +m_trial_perturbation_mag);
        pushJoint(NUSensorsData::RHipRoll, -m_trial_perturbation_mag);
    }
    else
    {
        pushJoint(NUSensorsData::LAnkleRoll, -m_trial_perturbation_mag);
        pushJoint(NUSensorsData::LHipRoll, +m_trial_perturbation_mag);
        pushJoint(NUSensorsData::RAnkleRoll, -m_trial_perturbation_mag);
        pushJoint(NUSensorsData::RHipRoll, +m_trial_perturbation_mag);
    }
}

/*! @brief Simulates an imaginary push forwards, that is a push from behind
 */
void WalkOptimiserBehaviour::pushForward()
{
    float magnitude = 0.5*m_trial_perturbation_mag;
    if (m_left_impact_time > m_right_impact_time) 
        pushJoint(NUSensorsData::LAnklePitch, -m_trial_perturbation_mag);
    else
        pushJoint(NUSensorsData::RAnklePitch, -m_trial_perturbation_mag);
}

/*! @brief Simulates a push from the front
 */
void WalkOptimiserBehaviour::pushBackward()
{
    float magnitude = 0.5*m_trial_perturbation_mag;
    if (m_left_impact_time > m_right_impact_time) 
        pushJoint(NUSensorsData::LAnklePitch, +m_trial_perturbation_mag);
    else
        pushJoint(NUSensorsData::RAnklePitch, +m_trial_perturbation_mag);
}

/*! @brief Adds an imaginary push to the specified joint of the specified offset
 @param id the id of the joint to attack
 @param offset the offset in radians applied to the joint (low stiffnesses will automatically be compensated for)
 */
void WalkOptimiserBehaviour::pushJoint(NUSensorsData::id_t id, float offset)
{
    static double time = 0;
    static float target = 0;
    static float velocity = 0;
    static float stiffness = 0;
    m_actions->getLastJointPosition(id, time, target, velocity, stiffness);
    m_actions->addJointPosition(id, time, target + (100*offset/stiffness), velocity, stiffness);
}

/*! @brief Ticks the optimiser
 */
void WalkOptimiserBehaviour::tickOptimiser(float metric)
{
    //cout << "Ticking Optimiser" << endl;
    m_optimiser->tick(metric, m_walk_parameters);
    m_walk->setWalkParameters(m_walk_parameters);
    
    saveOptimiser();
    m_optimiser->csvTo(m_parameter_log);
    csvTo(m_performance_log);
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
}

/*! @brief Ticks the internal walk parameter assessor
 */
void WalkOptimiserBehaviour::tickAssessor()
{
    using namespace boost::accumulators;
    static accumulator_set<float, stats<tag::mean, tag::variance> > speed_accumulator;
    static accumulator_set<float, stats<tag::mean, tag::variance> > cost_accumulator;
    static accumulator_set<float, stats<tag::mean, tag::variance> > robust_accumulator;
    
    speed_accumulator(m_measured_speed);
    cost_accumulator(m_measured_cost);
    robust_accumulator(m_measured_robustness);
    
    cout << "Speed: " << mean(speed_accumulator) << " sd:" << sqrt(variance(speed_accumulator)) << endl;
    cout << "Cost: " << mean(cost_accumulator) << " sd:" << sqrt(variance(cost_accumulator)) << endl;
    cout << "Robustness: " << mean(robust_accumulator) << " sd:" << sqrt(variance(robust_accumulator)) << endl;
}

void WalkOptimiserBehaviour::loadAssessor()
{
    ifstream assess_parameters_log;
    assess_parameters_log.open(m_assessparameters_filename.c_str());
    if (assess_parameters_log.is_open())
    {
        m_walk_parameters.csvFrom(assess_parameters_log);
        m_walk->setWalkParameters(m_walk_parameters);
        m_assessor_running = true;
        cout << "Loading walk parameters for assessment: " << endl;
        m_walk_parameters.summaryTo(cout);
    }
    else
        m_assessor_running = false;
}

/*! @brief Prints a human readable summary of the walk behaviour's state
 */
void WalkOptimiserBehaviour::summaryTo(ostream& output)
{
    output << "WalkOptimiserBehaviour: Optimising: ";
    if (m_metric_type == Speed)
        output << "Speed" << endl;
    else if (m_metric_type == Cost)
        output << "Cost" << endl;
    else if (m_metric_type == SpeedAndPushes)
        output << "SpeedAndPushes" << endl;
    else if (m_metric_type == CostAndPushes)
        output << "CostAndPushes" << endl;
    output << "Measured Speed: " << m_measured_speed << " Cost: " << m_measured_cost << "Robustness: " << m_measured_robustness << endl;
}

void WalkOptimiserBehaviour::csvTo(ostream& output)
{
    output << m_optimiser->getIterationCount() << ", " << m_optimiser->getBestPerformance() << ", " << m_measured_metric << ", " << m_measured_speed << ", " << m_measured_cost << ", " << m_measured_robustness << endl;
}


