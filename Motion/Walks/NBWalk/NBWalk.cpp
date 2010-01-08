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

#include "NUPlatform/NUSystem.h"
#include "Tools/debug.h"

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
NBWalk::NBWalk() :  nb_sensors(new Sensors()),     // Because we have our own way of storing sensors, I will keep a copy of NB's sensors class in here and copy over the data on each iteration
                    walkProvider(nb_sensors),
                    nullBodyProvider(nb_sensors),
                    curProvider(&nullBodyProvider),
                    nextProvider(&nullBodyProvider),
                    nextJoints(vector<float>(Kinematics::NUM_JOINTS,0.0f)),
                    nextStiffnesses(vector<float>(Kinematics::NUM_JOINTS,0.0f)),
                    newJoints(false),
                    readyToSend(false),
                    noWalkTransitionCommand(true)
{
    //Allow safe access to the next joints
    pthread_mutex_init(&next_joints_mutex, NULL);
    pthread_mutex_init(&next_provider_mutex, NULL);
    pthread_mutex_init(&stiffness_mutex, NULL);
    
    // Initialise the body to be frozen
    //boost::shared_ptr<FreezeCommand> paralyze = boost::shared_ptr<FreezeCommand>(new FreezeCommand());
    //nullBodyProvider.setCommand(paralyze);
    
    // Set the gait to the default one
    boost::shared_ptr<Gait>  defaultGait(new Gait(WEBOTS_GAIT));
    walkProvider.setCommand(defaultGait);
}

/*! @brief Destructor for walk module
 */
NBWalk::~NBWalk()
{
    pthread_mutex_destroy(&next_joints_mutex);
    pthread_mutex_destroy(&next_provider_mutex);
    pthread_mutex_destroy(&stiffness_mutex);
}

void NBWalk::doWalk()
{
    #if DEBUG_NUMOTION_VERBOSITY > 4
        debug << "NBWalk::doWalk()" << endl;
    #endif
    updateNBSensors();
    sendWalkCommand();
    
    preProcess();
    processJoints();
    processStiffness();
    bool active  = postProcess();

    if(active)
        readyToSend = true;
    
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
    
    pthread_mutex_lock(&next_provider_mutex);
    nextProvider = &walkProvider;
    walkProvider.setCommand(command);
    pthread_mutex_unlock(&next_provider_mutex);
}

void NBWalk::preProcess()
{
#if DEBUG_NUMOTION_VERBOSITY > 4
    debug << "NBWalk::preProcess()" << endl;
#endif
    pthread_mutex_lock(&next_provider_mutex);
    
    if (curProvider != &nullBodyProvider && nextProvider == &nullBodyProvider)
        walkProvider.hardReset();
    
    // determine the curProvider, and do any necessary swapping
	if (curProvider != nextProvider && !curProvider->isActive())
        swapBodyProvider();

	if (curProvider != nextProvider && !curProvider->isStopping())
    {
		debug << "Requesting stop on "<< *curProvider <<endl;
        curProvider->requestStop();
    }

    pthread_mutex_unlock(&next_provider_mutex);
}

void NBWalk::processJoints()
{
#if DEBUG_NUMOTION_VERBOSITY > 4
    debug << "NBWalk::processJoints()" << endl;
#endif
    if (curProvider->isActive())
    {
		// Request new joints
		curProvider->calculateNextJointsAndStiffnesses();
		const vector <float > llegJoints = curProvider->getChainJoints(LLEG_CHAIN);
		const vector <float > rlegJoints = curProvider->getChainJoints(RLEG_CHAIN);
		const vector <float > rarmJoints = curProvider->getChainJoints(RARM_CHAIN);
        
		const vector <float > larmJoints = curProvider->getChainJoints(LARM_CHAIN);
        
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

/*! @brief Method to process remaining stiffness requests
    Technically this could be handled by another provider, but there isn't
    too much too it:
 */ 
void NBWalk::processStiffness()
{
#if DEBUG_NUMOTION_VERBOSITY > 4
    debug << "NBWalk::processStiffness()" << endl;
#endif
    try
    {
        if(curProvider->isActive())
        {
            const vector <float > llegStiffnesses = curProvider->getChainStiffnesses(LLEG_CHAIN);
            const vector <float > rlegStiffnesses = curProvider->getChainStiffnesses(RLEG_CHAIN);
            const vector <float > rarmStiffnesses = curProvider->getChainStiffnesses(RARM_CHAIN);
            const vector <float > larmStiffnesses = curProvider->getChainStiffnesses(LARM_CHAIN);
            
            pthread_mutex_lock(&stiffness_mutex);
            for(unsigned int i = 0; i < LEG_JOINTS; i ++){
                nextStiffnesses[L_HIP_YAW_PITCH + i] = 100*llegStiffnesses.at(i);
            }
            for(unsigned int i = 0; i < LEG_JOINTS; i ++){
                nextStiffnesses[R_HIP_YAW_PITCH + i] = 100*rlegStiffnesses.at(i);
            }
            for(unsigned int i = 0; i < ARM_JOINTS; i ++){
                nextStiffnesses[L_SHOULDER_PITCH + i] = 100*larmStiffnesses.at(i);
            }
            for(unsigned int i = 0; i < ARM_JOINTS; i ++){
                nextStiffnesses[R_SHOULDER_PITCH + i] = 100*rarmStiffnesses.at(i);
            }
            pthread_mutex_unlock(&stiffness_mutex);
        }
    }
    catch(std::out_of_range & e)
    {
        cout << "Out of range exception caught in processStiffness"<< e.what() <<endl;
        exit(0);
    }
}

bool NBWalk::postProcess()
{
#if DEBUG_NUMOTION_VERBOSITY > 4
    debug << "NBWalk::postProcess()" << endl;
#endif
    pthread_mutex_lock(&next_provider_mutex);
    
    newJoints = true;
    
    //Make sure that if the current provider just became inactive,
    //and we have the next provider ready, then we want to swap to ensure
    //that we never have an inactive provider when an active one is potentially
    //ready to take over:
	if (curProvider != nextProvider && !curProvider->isActive()) 
        swapBodyProvider();
    
    pthread_mutex_unlock(&next_provider_mutex);
    
    return curProvider->isActive();
}

/*! @brief Method handles switching providers. Also handles any special action
    required when switching between providers
 */
void NBWalk::swapBodyProvider()
{
#if DEBUG_NUMOTION_VERBOSITY > 4
    debug << "NBWalk::swapBodyProvider()" << endl;
#endif
    std::vector<BodyJointCommand *> gaitSwitches;
    std::string old_provider = curProvider->getName();
    
    switch(nextProvider->getType()){
        case WALK_PROVIDER:
            curProvider = nextProvider;
            break;
        case NULL_PROVIDER:
        case SCRIPTED_PROVIDER:
        case HEAD_PROVIDER:
        default:
            noWalkTransitionCommand = true;
            curProvider = nextProvider;
    }
}

/*! @brief A function to copy necessary data from NUSensorsData (m_data) to nb_sensors
 */
void NBWalk::updateNBSensors()
{
    static vector<float> nu_jointpositions(m_data->getNumberOfJoints(NUSensorsData::AllJoints), 0);
    static vector<float> nu_jointtemperatures(nu_jointpositions.size(), 0);
    static vector<float> nb_jointpositions(nu_jointpositions.size(), 0);
    static vector<float> nb_jointtemperatures(nb_jointtemperatures.size(), 0);
    
    static list<float> anglex_buffer;
    static list<float> angley_buffer;
    static list<float>::iterator it;
    
#if DEBUG_NUMOTION_VERBOSITY > 4
    debug << "NBWalk::updateNBSensors()" << endl;
#endif
    
    // I'll to the joints first. All I need to do is get the order right
    // Positions
    m_data->getJointPositions(NUSensorsData::AllJoints, nu_jointpositions);
    nuToNBJointOrder(nu_jointpositions, nb_jointpositions);
    nb_sensors->setBodyAngles(nb_jointpositions);
    // Temperatures
    m_data->getJointTemperatures(NUSensorsData::AllJoints, nu_jointtemperatures);
    nuToNBJointOrder(nu_jointtemperatures, nb_jointtemperatures);
    nb_sensors->setBodyTemperatures(nb_jointtemperatures);
    
    // Now to the other sensors!
    static vector<float> accelvalues;
    static vector<float> gyrovalues;
    static vector<float> footvalues;
    static vector<float> buttonvalues;
    m_data->getAccelerometerValues(accelvalues);
    m_data->getGyroValues(gyrovalues);
    
    // it is clear that the walk engine is very sensitive to a good measurement for orientation
    /*anglex_buffer.push_front(atan2(-accelvalues[1],-accelvalues[2]));        // angleX is roll for NB. Right is positive
    angley_buffer.push_front(atan2(accelvalues[0],-accelvalues[2]));         // angleY is the pitch for NB. Forwards is positive
    if (anglex_buffer.size() > 2)
    {
        anglex_buffer.pop_back();
        angley_buffer.pop_back();
    }
    float xsum = 0;
    float ysum = 0;
    for ( it=anglex_buffer.begin() ; it != anglex_buffer.end(); it++ )
        xsum += *it;
    for ( it=angley_buffer.begin() ; it != angley_buffer.end(); it++ )
        ysum += *it;*/
    float angleX = atan2(-accelvalues[1],-accelvalues[2]);//xsum/anglex_buffer.size();
    float angleY = atan2(accelvalues[0],-accelvalues[2]);//ysum/angley_buffer.size();
    
    m_data->getFootSoleValues(NUSensorsData::AllFeet, footvalues);
    m_data->getButtonValues(NUSensorsData::MainButton, buttonvalues);
    
    nb_sensors->setMotionSensors(FSR(footvalues[0], footvalues[1], footvalues[2], footvalues[3]),
                                 FSR(footvalues[4], footvalues[5], footvalues[6], footvalues[7]),
                                 0,                                                             // no button in webots
                                 Inertial(-accelvalues[0]/100.0, -accelvalues[1]/100.0, -accelvalues[2]/100.0,
                                          gyrovalues[0]/100.0, gyrovalues[1]/100.0, angleX, angleY),
                                 Inertial(-accelvalues[0]/100.0, -accelvalues[1]/100.0, -accelvalues[2]/100.0,
                                          gyrovalues[0]/100.0, gyrovalues[1]/100.0, angleX, angleY));
    
    nb_sensors->setMotionBodyAngles(nb_sensors->getBodyAngles());
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

/*! @brief A function to nextJoints and nextStiffnesses to NUActionatorsData (m_actions)
 */
void NBWalk::updateActionatorsData()
{
    static vector<float> zerovel(m_data->getNumberOfJoints(NUSensorsData::AllJoints), 0);
    static vector<float> nu_nextJoints(m_data->getNumberOfJoints(NUSensorsData::AllJoints), 0);
    static vector<float> nu_nextStiffnesses(m_data->getNumberOfJoints(NUSensorsData::AllJoints), 100);
    nbToNUJointOrder(nextJoints, nu_nextJoints);
    //nbToNUJointOrder(nextStiffnesses, nu_nextStiffnesses);
    debug << "nu_nextStiffnesses: ";
    for (int i=0; i<nu_nextStiffnesses.size(); i++)
        debug << nu_nextStiffnesses[i] << ", ";
    debug << endl;
    m_actions->addJointPositions(NUActionatorsData::AllJoints, nusystem->getTime() + 40, nu_nextJoints, zerovel, nu_nextStiffnesses);
}

