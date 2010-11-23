/*! @file NBWalk.cpp
    @brief Implementation of NBWalk class

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

#include "NBWalk.h"
using namespace Kinematics;

#include "NUPlatform/NUPlatform.h"
#include "Infrastructure/NUSensorsData/NUSensorsData.h"
#include "Infrastructure/NUActionatorsData/NUActionatorsData.h"

#include "debug.h"
#include "debugverbositynumotion.h"


#include <math.h>
#include <list>

//! @todo TODO: put M_PI, NORMALISE, NUM_JOINTS somewhere else
#ifndef M_PI
    #define M_PI 3.1415926535
#endif

template <class T>
inline T NORMALISE(T theta){
    return atan2(sin(theta), cos(theta));
}

/*! Creates a module to walk using NB's walk engine
 */
NBWalk::NBWalk(NUSensorsData* data, NUActionatorsData* actions) :  NUWalk(data, actions),
                                                                   nb_sensors(new Sensors()),     // Because we have our own way of storing sensors, I will keep a copy of NB's sensors class in here and copy over the data on each iteration
                                                                   walkProvider(nb_sensors),
                                                                   nextJoints(vector<float>(Kinematics::NUM_JOINTS,0.0f))
{
    //Allow safe access to the next joints
    pthread_mutex_init(&next_joints_mutex, NULL);
    
    float larm[] = {0.1, 1.57, 0.15, -1.57};
    float rarm[] = {-0.1, 1.57, 0.15, 1.57};
    m_initial_larm = vector<float>(larm, larm + sizeof(larm)/sizeof(*larm));
    m_initial_rarm = vector<float>(rarm, rarm + sizeof(rarm)/sizeof(*rarm));
    
    float lleg[] = {0, -0.44, 0, 0.92, 0, -0.52};
    m_initial_lleg = vector<float>(lleg, lleg + sizeof(lleg)/sizeof(*lleg));
    m_initial_rleg = vector<float>(lleg, lleg + sizeof(lleg)/sizeof(*lleg));
    
    // Set the gait to the default one
    m_walk_parameters.load("NBWalkDefault");
    m_gait = boost::shared_ptr<Gait>(new Gait(DEFAULT_GAIT));
    setGait();
}

/*! @brief Destructor for walk module
 */
NBWalk::~NBWalk()
{
    pthread_mutex_destroy(&next_joints_mutex);
}

/*! @brief Kills the walk
 */
void NBWalk::kill()
{
    NUWalk::kill();
    walkProvider.hardReset();
}

void NBWalk::doWalk()
{
    #if DEBUG_NUMOTION_VERBOSITY > 4
        debug << "NBWalk::doWalk()" << endl;
    #endif
    updateNBSensors();
    sendWalkCommand();
    
    processJoints();
    
    updateActionatorsData();
}

void NBWalk::sendWalkCommand()
{
    static WalkCommand* command = new WalkCommand(0, 0, 0);
    command->x_mms = m_speed_x*10;
    command->y_mms = m_speed_y*10;
    command->theta_rads = m_speed_yaw;

    #if DEBUG_NUMOTION_VERBOSITY > 4
        debug << "NBWalk::sendWalkCommand() command: " << *command << endl;
    #endif
    
    walkProvider.setCommand(command);
}

void NBWalk::processJoints()
{
#if DEBUG_NUMOTION_VERBOSITY > 4
    debug << "NBWalk::processJoints()" << endl;
#endif
    if (walkProvider.isActive())
    {
		// Request new joints
		walkProvider.calculateNextJointsAndStiffnesses();
		const vector <float > llegJoints = walkProvider.getChainJoints(LLEG_CHAIN);
		const vector <float > rlegJoints = walkProvider.getChainJoints(RLEG_CHAIN);
		const vector <float > rarmJoints = walkProvider.getChainJoints(RARM_CHAIN);
        
		const vector <float > larmJoints = walkProvider.getChainJoints(LARM_CHAIN);
        
		// Copy the new values into place, and wait to be signaled.
		pthread_mutex_lock(&next_joints_mutex);
        
        
        for(unsigned int i = 0; i < LEG_JOINTS; i ++){
            nextJoints[L_HIP_YAW_PITCH + i] = llegJoints.at(i);
        }
        for(unsigned int i = 0; i < LEG_JOINTS; i ++){
            nextJoints[R_HIP_YAW_PITCH + i] = rlegJoints.at(i);
        }
        for(unsigned int i = 0; i < ARM_JOINTS; i ++){
            nextJoints[L_SHOULDER_PITCH + i] = larmJoints.at(i);
        }
        for(unsigned int i = 0; i < ARM_JOINTS; i ++){
            nextJoints[R_SHOULDER_PITCH + i] = rarmJoints.at(i);
        }
        pthread_mutex_unlock(&next_joints_mutex);
    }
}

/*! @brief A function to copy necessary data from NUSensorsData (m_data) to nb_sensors
 */
void NBWalk::updateNBSensors()
{
    #if DEBUG_NUMOTION_VERBOSITY > 4
        debug << "NBWalk::updateNBSensors()" << endl;
    #endif
    
    // I'll to the joints first. All I need to do is get the order right
    // Positions
    vector<float> nu_jointpositions;
    m_data->getPosition(NUSensorsData::All, nu_jointpositions);
    vector<float> nb_jointpositions(nu_jointpositions.size(), 0);
    nuToNBJointOrder(nu_jointpositions, nb_jointpositions);
    nb_sensors->setBodyAngles(nb_jointpositions);
    // Temperatures
    vector<float> nu_jointtemperatures;
    m_data->getTemperature(NUSensorsData::All, nu_jointtemperatures);
    vector<float> nb_jointtemperatures(nu_jointtemperatures.size(), 0);
    nuToNBJointOrder(nu_jointtemperatures, nb_jointtemperatures);
    nb_sensors->setBodyTemperatures(nb_jointtemperatures);
    
    // Now to the other sensors!
    static vector<float> accelvalues(3,0);
    static vector<float> gyrovalues(2,0);
    static vector<float> orientation(2,0);
    static vector<float> lfootvalues(4,0);
    static vector<float> rfootvalues(4,0);
    m_data->getAccelerometer(accelvalues);
    m_data->getGyro(gyrovalues);
    m_data->getOrientation(orientation);
    m_data->get(NUSensorsData::LFootTouch, lfootvalues);
    m_data->get(NUSensorsData::RFootTouch, rfootvalues);

    float angleX = 0;
    if (orientation.size() > 0)
        angleX = -orientation[0];           // NUbot convention has positive roll to the left, NB has positive roll to the right
    float angleY = 0;
    if (orientation.size() > 1)
        angleY = orientation[1];
    
    nb_sensors->setMotionSensors(FSR(lfootvalues[0], lfootvalues[1], lfootvalues[2], lfootvalues[3]),
                                 FSR(rfootvalues[0], rfootvalues[1], rfootvalues[2], rfootvalues[3]),
                                 0,                                                             // no button in webots
                                 Inertial(-accelvalues[0]/100.0, -accelvalues[1]/100.0, -accelvalues[2]/100.0,
                                          gyrovalues[0]/100.0, gyrovalues[1]/100.0, angleX, angleY),
                                 Inertial(-accelvalues[0]/100.0, -accelvalues[1]/100.0, -accelvalues[2]/100.0,
                                          gyrovalues[0]/100.0, gyrovalues[1]/100.0, angleX, angleY));
    
    nb_sensors->setMotionBodyAngles(nb_sensors->getBodyAngles());
}

void NBWalk::setWalkParameters(const WalkParameters& walkparameters)
{
    NUWalk::setWalkParameters(walkparameters);
    setGait();
}

void NBWalk::setGait()
{
    // copy the parameters from m_walk_parameters into m_gait
    vector<Parameter>& parameters = m_walk_parameters.getParameters();
    m_gait->step[0] = 1/parameters[0].get();        // step frequency
    m_gait->step[2] = 10*parameters[1].get();       // step height
    m_gait->zmp[1] = parameters[2].get();           // zmp static fraction
    m_gait->zmp[2] = 10*parameters[3].get();        // zmp offset
    m_gait->zmp[3] = 10*parameters[3].get();        // zmp offset
    m_gait->sensor[1] = parameters[4].get();        // sensor angle x gamma
    m_gait->sensor[2] = parameters[5].get();        // sensor angle y gamma
    m_gait->sensor[3] = parameters[6].get();        // sensor x spring constant
    m_gait->sensor[4] = parameters[7].get();        // sensor y spring constant
    m_gait->step[1] = parameters[8].get();          // double support time
    m_gait->hack[0] = parameters[9].get();          // hip roll hack
    m_gait->hack[1] = parameters[9].get();          // hip roll hack
    m_gait->step[3] = parameters[10].get();          // foot lift angle
    m_gait->stance[3] = parameters[11].get();        // forward lean
    m_gait->stance[0] = 10*parameters[12].get();     // torso height
    
    vector<float>& maxspeeds = m_walk_parameters.getMaxSpeeds();
    m_gait->step[4] = 10*maxspeeds[0];
    m_gait->step[5] = -10*maxspeeds[0];
    m_gait->step[6] = 10*maxspeeds[1];
    m_gait->step[7] = 2*maxspeeds[2];
    
    vector<float>& maxaccelerations = m_walk_parameters.getMaxAccelerations();
    m_gait->step[8] = 10*maxaccelerations[0];
    m_gait->step[9] = 10*maxaccelerations[1];
    m_gait->step[10] = maxaccelerations[2];
    
    walkProvider.setCommand(m_gait);
}

void NBWalk::nuToNBJointOrder(const vector<float>& nujoints, vector<float>& nbjoints)
{
    // NB order: HeadYaw, HeadPitch, LShoulderPitch, LShoulderRoll, LElbowYaw, LElbowRoll, LHipYawPitch, LHipRoll, LHipPitch, LKneePitch, LAnklePitch, LAnkleRoll, LHipYawPitch, RHipRoll, RHipPitch, RKneePitch, RAnklePitch, RAnkleRoll, RShoulderPitch, RShoulderRoll, RElbowYaw, RElbowRoll 
    // NU order: 0.HeadPitch, 1.HeadYaw, 2.LShoulderRoll, 3.LShoulderPitch, 4.LElbowRoll, 5.LElbowYaw, 6.RShoulderRoll, 7.RShoulderPitch, 8.RElbowRoll, 9.RElbowYaw, 10.LHipRoll, 11.LHipPitch, 12.LHipYawPitch, 13.LKneePitch, 14.LAnkleRoll, 15.LAnklePitch, 16.RHipRoll, 17.RHipPitch, 18.RHipYawPitch, 19.RKneePitch, 20.RAnkleRoll, 21.RAnklePitch
    // Clearly the order is mostly different, so there is only one way to do this!
    if (nbjoints.size() < 22 || nujoints.size() < 22)
        return;
    nbjoints[0] = nujoints[1];      // HeadYaw
    nbjoints[1] = nujoints[0];      // HeadPitch
    nbjoints[2] = nujoints[3];      // LShoulderPitch
    nbjoints[3] = nujoints[2];      // LShoulderRoll
    nbjoints[4] = nujoints[5];      // LElbowYaw
    nbjoints[5] = nujoints[4];      // LElbowRoll       
    nbjoints[6] = nujoints[12];     // LHipYawPitch
    nbjoints[7] = nujoints[10];     // LHipRoll
    nbjoints[8] = nujoints[11];     // LHipPitch
    nbjoints[9] = nujoints[13];     // LKneePitch
    nbjoints[10] = nujoints[15];    // LAnklePitch
    nbjoints[11] = nujoints[14];    // LAnkleRoll
    nbjoints[12] = nujoints[18];    // RHipYawPitch
    nbjoints[13] = nujoints[16];    // RHipRoll
    nbjoints[14] = nujoints[17];    // RHipPitch
    nbjoints[15] = nujoints[19];    // RKneePitch
    nbjoints[16] = nujoints[21];    // RAnklePitch
    nbjoints[17] = nujoints[20];    // RAnkleRoll
    nbjoints[18] = nujoints[7];     // RShoulderPitch
    nbjoints[19] = nujoints[6];     // RShoulderRoll
    nbjoints[20] = nujoints[9];     // RElbowYaw
    nbjoints[21] = nujoints[8];     // RElbowRoll
}

void NBWalk::nbToNUJointOrder(const vector<float>& nbjoints, vector<float>& nujoints)
{
    // NB order: 0.HeadYaw, 1.HeadPitch, 2.LShoulderPitch, 3.LShoulderRoll, 4.LElbowYaw, 5.LElbowRoll, 6.LHipYawPitch, 7.LHipRoll, 8.LHipPitch, 9.LKneePitch, 10.LAnklePitch, 11.LAnkleRoll, 12.RHipYawPitch, 13.RHipRoll, 14.RHipPitch, 15.RKneePitch, 16.RAnklePitch, 17.RAnkleRoll, 18.RShoulderPitch, 19.RShoulderRoll, 20.RElbowYaw, 21.RElbowRoll 
    // NU order: 0.HeadPitch, 1.HeadYaw, 2.LShoulderRoll, 3.LShoulderPitch, 4.LElbowRoll, 5.LElbowYaw, 6.RShoulderRoll, 7.RShoulderPitch, 8.RElbowRoll, 9.RElbowYaw, 10.LHipRoll, 11.LHipPitch, 12.LHipYawPitch, 13.LKneePitch, 14.LAnkleRoll, 15.LAnklePitch, 16.RHipRoll, 17.RHipPitch, 18.RHipYawPitch, 19.RKneePitch, 20.RAnkleRoll, 21.RAnklePitch
    // Clearly the order is mostly different, so there is only one way to do this!
    if (nbjoints.size() < 22 || nujoints.size() < 22)
        return;
    nujoints[0] = nbjoints[1];      // HeadPitch
    nujoints[1] = nbjoints[0];      // HeadYaw
    nujoints[2] = nbjoints[3];      // LShoulderRoll
    nujoints[3] = nbjoints[2];      // LShoulderPitch
    nujoints[4] = nbjoints[5];      // LElbowRoll
    nujoints[5] = nbjoints[4];      // LElbowYaw       
    nujoints[6] = nbjoints[19];     // RShoulderRoll
    nujoints[7] = nbjoints[18];     // RShoulderPitch
    nujoints[8] = nbjoints[21];     // RElbowRoll
    nujoints[9] = nbjoints[20];     // RElbowYaw
    nujoints[10] = nbjoints[7];     // LHipRoll
    nujoints[11] = nbjoints[8];     // LHipPitch
    nujoints[12] = nbjoints[6];     // LHipYawPitch
    nujoints[13] = nbjoints[9];     // LKneePitch
    nujoints[14] = nbjoints[11];    // LAnkleRoll
    nujoints[15] = nbjoints[10];    // LAnklePitch
    nujoints[16] = nbjoints[13];    // RHipRoll
    nujoints[17] = nbjoints[14];    // RHipPitch
    nujoints[18] = nbjoints[12];     // RHipYawPitch
    nujoints[19] = nbjoints[15];    // RKneePitch
    nujoints[20] = nbjoints[17];    // RAnkleRoll
    nujoints[21] = nbjoints[16];    // RAnklePitch
}

void NBWalk::nbToNULeftLegJointOrder(const vector<float>& nbjoints, vector<float>& nuleftlegjoints)
{
    if (nbjoints.size() < 22 || nuleftlegjoints.size() < 6)
        return;
    nuleftlegjoints[0] = nbjoints[7];     // LHipRoll
    nuleftlegjoints[1] = nbjoints[8];     // LHipPitch
    nuleftlegjoints[2] = nbjoints[6];     // LHipYawPitch
    nuleftlegjoints[3] = nbjoints[9];     // LKneePitch
    nuleftlegjoints[4] = nbjoints[11];    // LAnkleRoll
    nuleftlegjoints[5] = nbjoints[10];    // LAnklePitch
}

void NBWalk::nbToNURightLegJointOrder(const vector<float>& nbjoints, vector<float>& nurightlegjoints)
{
    if (nbjoints.size() < 22 || nurightlegjoints.size() < 6)
        return;
    nurightlegjoints[0] = nbjoints[13];    // RHipRoll
    nurightlegjoints[1] = nbjoints[14];    // RHipPitch
    nurightlegjoints[2] = nbjoints[12];     // RHipYawPitch
    nurightlegjoints[3] = nbjoints[15];    // RKneePitch
    nurightlegjoints[4] = nbjoints[17];    // RAnkleRoll
    nurightlegjoints[5] = nbjoints[16];    // RAnklePitch
}

void NBWalk::nbToNULeftArmJointOrder(const vector<float>& nbjoints, vector<float>& nuleftarmjoints)
{
    if (nbjoints.size() < 22 || nuleftarmjoints.size() < 4)
        return;
    nuleftarmjoints[0] = nbjoints[3];      // LShoulderRoll
    nuleftarmjoints[1] = nbjoints[2];      // LShoulderPitch
    nuleftarmjoints[2] = nbjoints[5];      // LElbowRoll
    nuleftarmjoints[3] = nbjoints[4];      // LElbowYaw       
}

void NBWalk::nbToNURightArmJointOrder(const vector<float>& nbjoints, vector<float>& nurightarmjoints)
{
    if (nbjoints.size() < 22 || nurightarmjoints.size() < 4)
        return;
    nurightarmjoints[0] = nbjoints[19];     // RShoulderRoll
    nurightarmjoints[1] = nbjoints[18];     // RShoulderPitch
    nurightarmjoints[2] = nbjoints[21];     // RElbowRoll
    nurightarmjoints[3] = nbjoints[20];     // RElbowYaw
}

/*! @brief A function to nextJoints and nextStiffnesses to NUActionatorsData (m_actions)
 */
void NBWalk::updateActionatorsData()
{
    static vector<float> nu_nextLeftLegJoints(m_actions->getSize(NUActionatorsData::LLeg), 0);
    static vector<float> nu_nextRightLegJoints(m_actions->getSize(NUActionatorsData::RLeg), 0);
    static vector<float> nu_nextLeftArmJoints(m_actions->getSize(NUActionatorsData::LArm), 0);
    static vector<float> nu_nextRightArmJoints(m_actions->getSize(NUActionatorsData::RArm), 0);
    
    vector<vector<float> >& armgains = m_walk_parameters.getArmGains();
    vector<vector<float> >& leggains = m_walk_parameters.getLegGains();
    
    nbToNULeftLegJointOrder(nextJoints, nu_nextLeftLegJoints);
    nbToNURightLegJointOrder(nextJoints, nu_nextRightLegJoints);
    nbToNULeftArmJointOrder(nextJoints, nu_nextLeftArmJoints);
    nbToNURightArmJointOrder(nextJoints, nu_nextRightArmJoints);
    
    applyPerturbation(nu_nextLeftLegJoints, leggains[0], nu_nextRightLegJoints, leggains[0]);
    
    m_actions->add(NUActionatorsData::LLeg, Platform->getTime(), nu_nextLeftLegJoints, leggains[0]);
    m_actions->add(NUActionatorsData::RLeg, Platform->getTime(), nu_nextRightLegJoints, leggains[0]);
    if (m_larm_enabled)
        m_actions->add(NUActionatorsData::LArm, Platform->getTime(), nu_nextLeftArmJoints, armgains[0]);
    if (m_rarm_enabled)
        m_actions->add(NUActionatorsData::RArm, Platform->getTime(), nu_nextRightArmJoints, armgains[0]);
}

