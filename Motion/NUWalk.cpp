/*! @file NUWalk.cpp
    @brief Implementation of nuwalk class

    @author Jason Kulk
 
 Copyright (c) 2009, 2010 Jason Kulk
 
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

#include "NUWalk.h"

#include "Infrastructure/NUSensorsData/NUSensorsData.h"
#include "Infrastructure/NUActionatorsData/NUActionatorsData.h"
#include "Infrastructure/Jobs/MotionJobs/WalkJob.h"
#include "Infrastructure/Jobs/MotionJobs/WalkToPointJob.h"
#include "Infrastructure/Jobs/MotionJobs/WalkParametersJob.h"
#include "Infrastructure/Jobs/MotionJobs/WalkPerturbationJob.h"

#include "walkconfig.h"
#ifdef USE_JWALK
    #include "Walks/JWalk/JWalk.h"
#endif
#ifdef USE_BWALK
    #include "Walks/BWalk/WalkingEngine.h"
#endif
#ifdef USE_JUPPWALK
    #include "Walks/JuppWalk/JuppWalk.h"
#endif
#ifdef USE_NBWALK
    #include "Walks/NBWalk/NBWalk.h"
#endif
#ifdef USE_VSCWALK
    #include "Walks/VSCWalk/VSCWalk.h"
#endif
#ifdef USE_ALWALK
    #include "Walks/ALWalk/ALWalk.h"
#endif
#ifdef USE_BEARWALK
    #include "Walks/BearWalk/BearWalk.h"
#endif
#ifdef USE_DARWINWALK
    #include "Walks/DarwinWalk/DarwinWalk.h"
#endif
#include "Walks/BlankWalk.h"

#include "debug.h"
#include "debugverbositynumotion.h"

#include <math.h>
using namespace std;
#include "Tools/Math/General.h"
using namespace mathGeneral;

class NUInverseKinematics;

NUWalk* NUWalk::getWalkEngine(NUSensorsData* data, NUActionatorsData* actions, NUInverseKinematics* ik)
{
    #ifdef USE_JWALK
        return new JWalk();
    #endif

    #ifdef USE_BWALK
        return new WalkingEngine(data, actions, ik);
    #endif
    
    #ifdef USE_JUPPWALK
        return new JuppWalk(data, actions);
    #endif
    
    #ifdef USE_NBWALK
        return new NBWalk(data, actions);
    #endif
    
    #ifdef USE_ALWALK
        return new ALWalk(data, actions);
    #endif
    
    #ifdef USE_VSCWALK
        return new VSCWalk(data, actions);
    #endif
                
    #ifdef USE_BEARWALK
        return new BearWalk(data, actions);
    #endif
    
	#ifdef USE_DARWINWALK
        return new DarwinWalk(data, actions);
    #endif

    return new BlankWalk(data, actions);
}

NUWalk::NUWalk(NUSensorsData* data, NUActionatorsData* actions) : NUMotionProvider("NUWalk", data, actions)
{
    m_current_time = 0;
    m_previous_time = 0;
    
    m_target_speed_x = 0;                         //!< the current target x speed cm/s
    m_target_speed_y = 0;                         //!< the current target y speed cm/s
    m_target_speed_yaw = 0;                       //!< the current target yaw speed rad/s
    
    m_speed_x = 0;                                //!< the current x speed in cm/s
    m_speed_y = 0;                                //!< the current y speed in cm/s
    m_speed_yaw = 0;                              //!< the current rotation speed in rad/s
    
    m_walk_enabled = false;
    m_larm_enabled = true;
    m_rarm_enabled = true;
    
    m_perturbation_start_time = -1000;
    m_perturbation_magnitude = 0;
    m_perturbation_direction = 0;
}

/*! @brief Destructor for motion module
 */
NUWalk::~NUWalk()
{
#if DEBUG_NUMOTION_VERBOSITY > 0
    debug << "NUWalk::~NUWalk()" << endl;
#endif  
    kill();
}

/*! @brief Enables the walk
 */
void NUWalk::enableWalk()
{
    #if DEBUG_NUMOTION_VERBOSITY > 3
        debug << "NUWalk::enableWalk()" << endl;
    #endif
    if (m_data == NULL or m_actions == NULL)
        m_walk_enabled = false;
    else if (not inInitialPosition())
        moveToInitialPosition();
    else
    {
        m_walk_enabled = true;
        setArmEnabled(true, true);
    }
}

/*! @brief Freezes the walk engine and (eventually) frees the joint actionators to be used elsewhere
 */
void NUWalk::stop()
{
    #if DEBUG_NUMOTION_VERBOSITY > 3
        debug << "NUWalk::stop()" << endl;
    #endif
    stopLegs();
}


void NUWalk::stopArms()
{
    #if DEBUG_NUMOTION_VERBOSITY > 3
        debug << "NUWalk::stopArms()" << endl;
    #endif
    setArmEnabled(false, false);
}

void NUWalk::stopLegs()
{
    #if DEBUG_NUMOTION_VERBOSITY > 3
        debug << "NUWalk::stopLegs()" << endl;
    #endif
    m_target_speed_x = 0;
    m_target_speed_y = 0;
    m_target_speed_yaw = 0;
    if (not isActive())
        kill();
}

/*! @brief Kills the walk engine
 */
void NUWalk::kill()
{
    #if DEBUG_NUMOTION_VERBOSITY > 3
        debug << "NUWalk::kill()" << endl;
    #endif
    m_walk_enabled = false;
}

/*! @brief Returns true if the walk is active */
bool NUWalk::isActive()
{   
    if (not m_walk_enabled)
        return false;
    else if (m_speed_x != 0 or m_speed_y != 0 or m_speed_yaw != 0)
        return true;
    else 
    {
        vector<float> jointvelocities;
        float jointvelocitysum = 0.0f;
        if(m_data->getVelocity(NUSensorsData::Body, jointvelocities))
        {
            for (unsigned int i = 0; i < jointvelocities.size(); i++)
                jointvelocitysum += fabs(jointvelocities[i]);
        }
        if (jointvelocitysum > 0.4)
            return true;
        else
            return false;
    }

}

/*! @brief Returns false */
bool NUWalk::isUsingHead()
{
    return false;
}

/*! @brief Returns true if the walk engine is using the arms */
bool NUWalk::isUsingArms()
{
    if (isActive() and (m_larm_enabled or m_rarm_enabled))
        return true;
    else
        return false;
}

/*! @brief Returns true if the walk engine is using the legs */
bool NUWalk::isUsingLegs()
{
    return isActive();
}

/*! @brief Process new sensor data, and produce actionator commands
 
    @param data a pointer to the most recent sensor data storage class
    @param actions a pointer to the actionators data storage class. 
*/
void NUWalk::process(NUSensorsData* data, NUActionatorsData* actions)
{
    #if DEBUG_NUMOTION_VERBOSITY > 2
        debug << "NUWalk::process(" << data << ", " << actions << ") requiresArms: " << requiresArms() << " isUsingArms: " << isUsingArms() << " requiresLegs: " << requiresLegs() << " isUsingLegs: " << isUsingLegs() << endl;
    #endif
    if (actions == NULL || data == NULL)
        return;
    m_data = data;
    m_actions = actions;
    m_current_time = m_data->CurrentTime;
    
    if (m_current_time - m_previous_time > 200)
        m_walk_enabled = false;
    
    if (m_walk_enabled)
    {
        calculateCurrentSpeed();
        doWalk();
    }
    m_previous_time = m_current_time;
}

/*! @brief Process a walk speed job
    @param job the walk job to be processed
    @param currentprovider true if the walk is the current provider, and therefore has permission to use the joints
 */
void NUWalk::process(WalkJob* job, bool currentprovider)
{
    if (not currentprovider)
        return;
    
	#if DEBUG_NUMOTION_VERBOSITY > 2
        debug << "NUWalk::process ";
        job->summaryTo(debug);
    #endif
    
    float t,d,y;
    t = job->getTranslationSpeed();
    d = job->getDirection();
    y = job->getRotationSpeed();
    if (not m_walk_enabled /*and (t != 0 or y != 0)*/)
        enableWalk();
    setTargetSpeed(t, d, y);
}

/*! @brief Process a walk to point job
    @param job the walk to point job to be processed
    @param currentprovider true if the walk is the current provider, and therefore has permission to use the joints
 */
void NUWalk::process(WalkToPointJob* job, bool currentprovider)
{
    errorlog << "NUWalk::process(WalkToPointJob) is no longer supported. Your job has been discarded!" << endl;
}

/*! @brief Process a walk parameters job
    @param job the walk parameter job to be processed
 */
void NUWalk::process(WalkParametersJob* job)
{
    #if DEBUG_NUMOTION_VERBOSITY > 2
        debug << "NUWalk::process ";
        job->summaryTo(debug);
    #endif
    WalkParameters parameters;
    job->getWalkParameters(parameters);                
    setWalkParameters(parameters);
}

/*! @brief Process a walk perturbation job
    @param job the walk perturbation job to be processed
 */
void NUWalk::process(WalkPerturbationJob* job)
{
    m_perturbation_start_time = m_current_time;
    m_perturbation_magnitude = job->getMagnitude();
    m_perturbation_direction = job->getDirection();
}

/*! @brief Sets m_target_speed_x, m_target_speed_y and m_target_speed_yaw.
    @param trans_speed the translation speed factor
    @param trans_direction the translation direction in rad
    @param rot_speed the rotation speed in radians per second
 */
void NUWalk::setTargetSpeed(float trans_speed, float trans_direction, float rot_speed)
{
    #if DEBUG_NUMOTION_VERBOSITY > 3
        debug << "NUWalk::setTargetSpeed(" << trans_speed << "," << trans_direction << "," << rot_speed << ")";
    #endif
    vector<float>& maxspeeds = m_walk_parameters.getMaxSpeeds();
    
    if (isnan(trans_speed))
        trans_speed = 1.0;
    if (isnan(trans_direction) or isnan(rot_speed))
        return;
    
    // clip translational speed to be a fraction
    if (trans_speed < -1)
        trans_speed = -1;
    else if (trans_speed > 1)
        trans_speed = 1;
    
    // clip the rotation first
    if (fabs(rot_speed) > maxspeeds[2])
        rot_speed = sign(rot_speed)*maxspeeds[2];
    
    float rot_frac = fabs(rot_speed)/maxspeeds[2];
    const float clip_threshold = 0.25;                  // rotation speeds above this percentage will result in a reduction of translational velocity
    const float min_forward_trans = 0.3;                // the minimum percentage of translational speed at maximum clipping rate
    const float min_backward_trans = 0.1;               // the minimum percentage of translational speed at maximum clipping rate
    if (rot_frac > clip_threshold)
    {   // if the rotation speed is high then clip the trans_speed
        if (trans_speed > 0)
            trans_speed = trans_speed*(1 + ((min_forward_trans - 1)/(1 - clip_threshold))*(rot_frac - clip_threshold));
        else
            trans_speed = trans_speed*(1 + ((min_backward_trans - 1)/(1 - clip_threshold))*(rot_frac - clip_threshold));
    }
    
    float x = trans_speed*maxspeeds[0]*cos(trans_direction);               // compute desired x
    float y = trans_speed*maxspeeds[0]*sin(trans_direction);               // compute desired y
    
    if (x < -0.5*maxspeeds[0])
    {   // if walking backwards clip the x first
        x = -0.5*maxspeeds[0];
        y = x*tan(trans_direction);
    }
    
    if (fabs(y) > maxspeeds[1])
    {   // we assume the sidewards direction is always the slowest, if it needs clipping, clip the x to achieve the desired direction
        y = sign(y)*maxspeeds[1];
        x = y/tan(trans_direction);
    }
    
    m_target_speed_x = x;
    m_target_speed_y = y;
    m_target_speed_yaw = rot_speed;
    
    #if DEBUG_NUMOTION_VERBOSITY > 3
        debug << "->(" << x << "," << y << "," << rot_speed << ")" << endl;
    #endif
}

/*! @brief Sets m_speed_x, m_speed_y and m_speed_yaw; they are smoothed to satisify acceleration constraints
 */
void NUWalk::calculateCurrentSpeed()
{
    if (m_data == NULL)
        return;
    float timestep = (m_current_time - m_previous_time)/1000.0;
    float x, y, yaw;
    
    // calculate the accelerations required to go to the target speed in the next timestep
    if (timestep != 0)
    {
        x = (m_target_speed_x - m_speed_x)/timestep;
        y = (m_target_speed_y - m_speed_y)/timestep;
        yaw = (m_target_speed_yaw - m_speed_yaw)/timestep;
    }
    else 
    {
        x = 0;
        y = 0;
        yaw = 0;
    }
    
    // clip the accelerations to the max values (if the max values exist)
    vector<float>& maxaccels = m_walk_parameters.getMaxAccelerations();
    if (maxaccels.size() > 0 && fabs(x) > fabs(maxaccels[0]))      // if clipping is available, and the input is greater than the limit, then clip it
        x = sign(x)*maxaccels[0];
    
    if (maxaccels.size() > 1 && fabs(y) > fabs(maxaccels[1]))      // if clipping is available, and the input is greater than the limit, then clip it
        y = sign(y)*maxaccels[1];
    
    if (maxaccels.size() > 2 && fabs(yaw) > fabs(maxaccels[2]))    // if clipping is available, and the input is greater than the limit, then clip it
        yaw = sign(yaw)*maxaccels[2];
     
    // set the current speeds 
    m_speed_x = m_speed_x + x*timestep;
    m_speed_y = m_speed_y + y*timestep;
    m_speed_yaw = m_speed_yaw + yaw*timestep;
}

/*! @brief Updates currentspeed with the current speed of the walk engine
 */
void NUWalk::getCurrentSpeed(vector<float>& currentspeed)
{
    static vector<float> speeds(3,0);
    speeds[0] = m_speed_x;
    speeds[1] = m_speed_y;
    speeds[2] = m_speed_yaw;
    currentspeed = speeds;
}

/*! @brief Updates maxspeeds with the current maximum speeds of the walk engine */
void NUWalk::getMaximumSpeed(vector<float>& maxspeeds)
{
    maxspeeds = m_walk_parameters.getMaxSpeeds();
}

/*! @brief Sets the walk parameters to those specified in the variable
    @param walkparameters the new walk parameters for the walk and gain engines
 */
void NUWalk::setWalkParameters(const WalkParameters& walkparameters)
{
    #if DEBUG_NUMOTION_VERBOSITY > 3
        debug << "NUWalk::setWalkParameters(" << walkparameters << ")" << endl;
    #endif
    m_walk_parameters = walkparameters;
}

/*! @brief Returns the current walk parameters
    @return the current walkparameters
 */
WalkParameters& NUWalk::getWalkParameters()
{
    return m_walk_parameters;
}

/*! @brief Sets whether each of the arms can be used by the walk engine
 */
void NUWalk::setArmEnabled(bool leftarm, bool rightarm)
{
    m_larm_enabled = leftarm;
    m_rarm_enabled = rightarm;
}

/*! @brief Returns true if the robot is in the walk engine's initial position
 */
bool NUWalk::inInitialPosition()
{
    // get the current joint positions
    vector<float> sensor_larm, sensor_rarm;
    vector<float> sensor_lleg, sensor_rleg;
    m_data->getPosition(NUSensorsData::LArm, sensor_larm);
    m_data->getPosition(NUSensorsData::RArm, sensor_rarm);
    m_data->getPosition(NUSensorsData::LLeg, sensor_lleg);
    m_data->getPosition(NUSensorsData::RLeg, sensor_rleg);
    
    // compare the sensor positions to the initial positions
    return allEqual(sensor_larm, m_initial_larm, 0.15f) and allEqual(sensor_rarm, m_initial_rarm, 0.15f) and allEqual(sensor_lleg, m_initial_lleg, 0.05f) and allEqual(sensor_rleg, m_initial_rleg, 0.05f);
}

/*! @brief Moves the robot into the initial position
 */
void NUWalk::moveToInitialPosition()
{
    static const float movespeed = 0.8;
    //static const float movespeed = 0.6;
    static double movecompletiontime = -100;
    if (movecompletiontime >= m_current_time)                // if there is already a move happening let it finish
        return; 
    else if (movecompletiontime >= m_current_time - 100)     // if a move has just finished don't start another one, just enable the walk (to avoid infinite loop)
    {
        m_walk_enabled = true;
        setArmEnabled(true, true);
        return;
    }
    else
    {
        #if DEBUG_NUMOTION_VERBOSITY > 3
            debug << "NUWalk::moveToInitialPosition()" << endl;
        #endif
        // get the current joint positions
        vector<float> sensor_larm, sensor_rarm;
        vector<float> sensor_lleg, sensor_rleg;
        m_data->getPosition(NUSensorsData::LArm, sensor_larm);
        m_data->getPosition(NUSensorsData::RArm, sensor_rarm);
        m_data->getPosition(NUSensorsData::LLeg, sensor_lleg);
        m_data->getPosition(NUSensorsData::RLeg, sensor_rleg);
        
        // compute the time required to move into the initial pose for each limb
        double time_larm = 1000*(maxDifference(sensor_larm, m_initial_larm)/movespeed);
        double time_rarm = 1000*(maxDifference(sensor_rarm, m_initial_rarm)/movespeed);
        double time_lleg = 1000*(maxDifference(sensor_lleg, m_initial_lleg)/movespeed);
        double time_rleg = 1000*(maxDifference(sensor_rleg, m_initial_rleg)/movespeed);
        
        // set the move complettion to be the maximum of each limb
        movecompletiontime = m_current_time + std::max(std::max(time_larm, time_rarm), std::max(time_lleg, time_rleg)) + 100;
        
        const float c_leg_stiffness = 75.f;
        const float c_arm_stiffness = 20.f;

        // give the command to the actionators
        m_actions->add(NUActionatorsData::LArm, m_current_time + 100, sensor_larm, c_arm_stiffness);
        m_actions->add(NUActionatorsData::RArm, m_current_time + 100, sensor_rarm, c_arm_stiffness);
        m_actions->add(NUActionatorsData::LLeg, m_current_time + 100, sensor_lleg, c_leg_stiffness);
        m_actions->add(NUActionatorsData::RLeg, m_current_time + 100, sensor_rleg, c_leg_stiffness);
        
        m_actions->add(NUActionatorsData::LArm, m_current_time + time_larm, m_initial_larm, c_arm_stiffness);
        m_actions->add(NUActionatorsData::RArm, m_current_time + time_rarm, m_initial_rarm, c_arm_stiffness);
        m_actions->add(NUActionatorsData::LLeg, m_current_time + time_lleg, m_initial_lleg, c_leg_stiffness);
        m_actions->add(NUActionatorsData::RLeg, m_current_time + time_rleg, m_initial_rleg, c_leg_stiffness);
    }
}

/*! @brief Applies m_perturbation_magnitude and m_perturbation_direction to the leftleg and rightleg vectors in place
    @param leftleg a vector of joint values for the left leg
    @param rightleg a vector of joint values for the right leg
 */
void NUWalk::applyPerturbation(vector<float>& leftleg, vector<float>& leftleggains, vector<float>& rightleg, vector<float> rightleggains)
{   // the problem is that here I can't do this in a platform independent way :(
    // so this will have to go on my TODO: implement this in a platform independent way
    if (m_current_time - m_perturbation_start_time < 200)
    {
        float roll = -(m_perturbation_magnitude/2000)*sin(m_perturbation_direction);
        float pitch = -(m_perturbation_magnitude/1000)*cos(m_perturbation_direction); 
        
        // THIS WILL ONLY WORK FOR THE NAO [Roll,Pitch,Yaw,Knee,Roll,Pitch]
        leftleg[0] += roll*(100/leftleggains[0]);
        leftleg[4] -= roll*(100/leftleggains[4]);
        rightleg[0] += roll*(100/rightleggains[0]);
        rightleg[4] -= roll*(100/rightleggains[4]);
        leftleg[5] += pitch*(100/leftleggains[5]);
        rightleg[5] += pitch*(100/rightleggains[5]);
    }
}

