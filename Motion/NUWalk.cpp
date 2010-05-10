/*! @file NUWalk.cpp
    @brief Implementation of nuwalk class

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

#include "NUWalk.h"
#include "NUPlatform/NUSensors/NUSensorsData.h"
#include "NUPlatform/NUActionators/NUActionatorsData.h"
#include "Behaviour/Jobs/MotionJobs/WalkJob.h"
#include "Behaviour/Jobs/MotionJobs/WalkToPointJob.h"
#include "Behaviour/Jobs/MotionJobs/WalkParametersJob.h"

#include "walkconfig.h"
#ifdef USE_JWALK
    #include "Walks/JWalk/JWalk.h"
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

#include "NUPlatform/NUSystem.h"

#include "debug.h"
#include "debugverbositynumotion.h"

#include <math.h>
using namespace std;

NUWalk* NUWalk::getWalkEngine()
{
#ifdef USE_JWALK
    return new JWalk();
#else
    #ifdef USE_JUPPWALK
        return new JuppWalk();
    #else
        #ifdef USE_NBWALK
            return new NBWalk();
        #else
            #ifdef USE_VSCWALK
                return new VSCWalk();
            #else
                #ifdef USE_ALWALK
                    return new ALWalk();
                #endif
            #endif
        #endif
    #endif
#endif
    return NULL;
}

NUWalk::NUWalk()
{
    m_target_speed_x = 0;                         //!< the current target x speed cm/s
    m_target_speed_y = 0;                         //!< the current target y speed cm/s
    m_target_speed_yaw = 0;                       //!< the current target yaw speed rad/s
    
    m_speed_x = 0;                                //!< the current x speed in cm/s
    m_speed_y = 0;                                //!< the current y speed in cm/s
    m_speed_yaw = 0;                              //!< the current rotation speed in rad/s
    
    m_point_time = 0;                             //!< the desired time to reach the current target point in milliseconds from now
    m_point_x = 0;                                //!< the current target point's x position in cm
    m_point_y = 0;                                //!< the current target point's y position in cm
    m_point_theta = 0;                            //!< the current target point's final orientation relative to the current in radians
    
    m_walk_enabled = false;
    m_larm_enabled = true;
    m_rarm_enabled = true;
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

/*! @brief Kills the walk engine
 */
void NUWalk::kill()
{
    m_walk_enabled = false;
}

/*! @brief Process new sensor data, and produce actionator commands
 
    @param data a pointer to the most recent sensor data storage class
    @param actions a pointer to the actionators data storage class. 
*/
void NUWalk::process(NUSensorsData* data, NUActionatorsData* actions)
{
#if DEBUG_NUMOTION_VERBOSITY > 3
    debug << "NUWalk::process(" << data << ", " << actions << ")" << endl;
#endif
    if (actions == NULL || data == NULL)
        return;
    m_data = data;
    m_actions = actions;
    if (m_walk_enabled)
    {
        calculateCurrentSpeed();
        doWalk();
    }
}

/*! @brief Process a walk speed job
    @param job the walk job to be processed
 */
void NUWalk::process(WalkJob* job)
{
    vector<float> speed;
    job->getSpeed(speed);
    m_walk_enabled = true;
    setTargetSpeed(speed);
}

/*! @brief Process a walk to point job
    @param job the walk to point job to be processed
 */
void NUWalk::process(WalkToPointJob* job)
{
    double time;
    vector<float> position;
    job->getPosition(time, position);
    m_walk_enabled = true;
    setTargetPoint(time, position);
}

/*! @brief Process a walk parameters job
    @param job the walk parameter job to be processed
 */
void NUWalk::process(WalkParametersJob* job)
{
    WalkParameters parameters;
    job->getWalkParameters(parameters);                
    setWalkParameters(parameters);
}

/*! @brief Sets m_target_speed_x, m_target_speed_y and m_target_speed_yaw. The given speeds will be clipped if they are faster than the maximum possible speeds
    @param speed the desired speeds (x,y,theta)
 */
void NUWalk::setTargetSpeed(const vector<float>& speed)
{
    float x = 0;
    float y = 0;
    float yaw = 0;
    vector<float> maxspeeds = m_walk_parameters.getMaxSpeeds();
    
    if (speed.size() > 0)
    {
        x = speed[0];
        if (maxspeeds.size() > 0 && fabs(x) > fabs(maxspeeds[0]))      // if clipping is available, and the input is greater than the limit, then clip it
            x = sign(x)*maxspeeds[0];
    }
    if (speed.size() > 1)
    {
        y = speed[1];
        if (maxspeeds.size() > 1 && fabs(y) > fabs(maxspeeds[1]))      // if clipping is available, and the input is greater than the limit, then clip it
            y = sign(y)*maxspeeds[1];
    }
    if (speed.size() > 2)
    {
        yaw = speed[2];
        if (maxspeeds.size() > 2 && fabs(yaw) > fabs(maxspeeds[2]))    // if clipping is available, and the input is greater than the limit, then clip it
            yaw = sign(yaw)*maxspeeds[2];
    }
    m_target_speed_x = x;
    m_target_speed_y = y;
    m_target_speed_yaw = yaw;
}

/*! @brief Sets m_speed_x, m_speed_y and m_speed_yaw; they are smoothed to satisify acceleration constraints
 */
void NUWalk::calculateCurrentSpeed()
{
    if (m_data == NULL)
        return;
    static double previoustime = m_data->CurrentTime;
    float timestep = (m_data->CurrentTime - previoustime)/1000.0;
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
    vector<float> maxaccels = m_walk_parameters.getMaxAccelerations();
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
    
    previoustime = m_data->CurrentTime;
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

/*! @brief Walk to the given point by the given time
 
    Use this function to make use of the internal path planning algorithms. These algorithms are purely time-based and
    consequently they will try to get to the given point as fast as possible.
    To instruct the robot to move as fast as possible, just put in a small time value, the speed will be clipped internally.
    To instruct the robot to stop specify all coordinates to be zero. Alternatively, the robot will stop if no
    walk command has been issued in the last second.
 
    @warning Don't think that you can use this function to queue up points to walk to, or
             even that you can just call this function once and expect the robot to magically
             walk to the point. You need to call this function at every behaviour iteration with new
             data so that localisation feedback is used to walk to the point.
 
    @param time the desired time to reach the given point (ms)
    @param x the desired relative target [x (cm), y (cm), theta (rad)]
 */
void NUWalk::setTargetPoint(double time, const vector<float>& position)
{
    m_point_time = time;
    if (position.size() == 3)
    {
        m_point_x = position[0];
        m_point_y = position[1];
        m_point_theta = position[2];
    }
    else if (position.size() == 2)
    {
        m_point_x = position[0];
        m_point_y = position[1];
        m_point_theta = 0;
    }
}

/*! @brief Sets the walk parameters to those specified in the variable
    @param walkparameters the new walk parameters for the walk and gain engines
 */
void NUWalk::setWalkParameters(const WalkParameters& walkparameters)
{
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

