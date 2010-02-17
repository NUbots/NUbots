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

#include "walkconfig.h"
#include "NUWalk.h"
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

#include "NUPlatform/NUSystem.h"
#include "debug.h"

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
    m_speed_timestamp = 0;                        //!< the timestamp of the last speed command
    
    m_point_time = 0;                             //!< the desired time to reach the current target point in milliseconds from now
    m_point_x = 0;                                //!< the current target point's x position in cm
    m_point_y = 0;                                //!< the current target point's y position in cm
    m_point_theta = 0;                            //!< the current target point's final orientation relative to the current in radians
    m_point_timestamp = 0;                        //!< the timestamp of the last point command
}

/*! @brief Destructor for motion module
 */
NUWalk::~NUWalk()
{
    m_gait_walk_parameters.clear();
    m_gait_max_speeds.clear();
    m_gait_max_accelerations.clear();
    m_gait_arm_gains.clear();      
    m_gait_torso_gains.clear();    
    m_gait_leg_gains.clear();      
}

/*! @brief Process new sensor data, and produce actionator commands
 
    @param data a pointer to the most recent sensor data storage class
    @param actions a pointer to the actionators data storage class. 
*/
void NUWalk::process(NUSensorsData* data, NUActionatorsData* actions)
{
    if (actions == NULL || data == NULL)
        return;
    m_data = data;
    m_actions = actions;
    doWalk();
}

/*! @brief Walk with the given speed vector
 
    Use this function to precisely control the locomotion direction of the robot. 
    To instruct the robot to move as fast as possible, just put in a very large value, it will be clipped internally.
    To instruct the robot to stop specify all speeds to be zero. Alternatively, the robot will stop if no
    walk command has been issued in the last second.
 
    @param speed the desired walk velocity [x (cm/s), y (cm/s), rotation (rad/s)]
 */
void NUWalk::walkSpeed(const vector<float>& speed)
{
    setTargetSpeeds(speed);
    setCurrentSpeeds();
    m_speed_timestamp = nusystem->getTime();
}

/*! @brief Sets the target speeds. The given speeds will be clipped if they are faster than the maximum possible speeds
 
    m_target_speed_x, m_target_speed_y and m_target_speed_yaw are set with valid speeds. This is may not be the current speed
    if the new targets require the robot to accelerate too quickly.
 
    @param speed the desired speeds (x,y,theta)
 */
void NUWalk::setTargetSpeeds(const vector<float>& speed)
{
    static float temp_x, temp_y, temp_yaw;
    temp_x = 0;
    temp_y = 0;
    temp_yaw = 0;
    if (speed.size() > 0)
    {
        temp_x = speed[0];
        if (m_gait_max_speeds.size() > 0 && fabs(temp_x) > fabs(m_gait_max_speeds[0]))      // if clipping is available, and the input is greater than the limit, then clip it
            temp_x = (fabs(temp_x)/temp_x)*m_gait_max_speeds[0];
    }
    if (speed.size() > 1)
    {
        temp_y = speed[1];
        if (m_gait_max_speeds.size() > 1 && fabs(temp_y) > fabs(m_gait_max_speeds[1]))      // if clipping is available, and the input is greater than the limit, then clip it
            temp_y = (fabs(temp_y)/temp_y)*m_gait_max_speeds[1];
    }
    if (speed.size() > 2)
    {
        temp_yaw = speed[2];
        if (m_gait_max_speeds.size() > 2 && fabs(temp_yaw) > fabs(m_gait_max_speeds[2]))      // if clipping is available, and the input is greater than the limit, then clip it
            temp_yaw = (fabs(temp_yaw)/temp_yaw)*m_gait_max_speeds[2];
    }
    m_target_speed_x = temp_x;
    m_target_speed_y = temp_y;
    m_target_speed_yaw = temp_yaw;
}

/*! @brief Sets the current walk engine speed. The current speeds are smoothed to satisify acceleration constraints
 
    m_speed_x, m_speed_y and m_speed_yaw are set with smoothed speeds
 */
void NUWalk::setCurrentSpeeds()
{
    if (m_data == NULL)
        return;
    static double previoustime = m_data->CurrentTime;
    float timestep = (m_data->CurrentTime - previoustime)/1000.0;
    float acceleration_x, acceleration_y, acceleration_yaw;
    // calculate the accelerations required to go move to the target speed in the next timestep
    if (timestep != 0)
    {
        acceleration_x = (m_target_speed_x - m_speed_x)/timestep;
        acceleration_y = (m_target_speed_y - m_speed_y)/timestep;
        acceleration_yaw = (m_target_speed_yaw - m_speed_yaw)/timestep;
    }
    else 
    {
        acceleration_x = 0;
        acceleration_y = 0;
        acceleration_yaw = 0;
    }
    debug << "After accel calc." << endl;
    // clip the accelerations to the max values (if the max values exist)
    if (m_gait_max_accelerations.size() > 0 && fabs(acceleration_x) > fabs(m_gait_max_accelerations[0]))      // if clipping is available, and the input is greater than the limit, then clip it
        acceleration_x = sign(acceleration_x)*m_gait_max_accelerations[0];
    
    if (m_gait_max_accelerations.size() > 1 && fabs(acceleration_y) > fabs(m_gait_max_accelerations[1]))      // if clipping is available, and the input is greater than the limit, then clip it
        acceleration_y = sign(acceleration_y)*m_gait_max_accelerations[1];
    
    if (m_gait_max_accelerations.size() > 2 && fabs(acceleration_yaw) > fabs(m_gait_max_accelerations[2]))      // if clipping is available, and the input is greater than the limit, then clip it
        acceleration_yaw = sign(acceleration_yaw)*m_gait_max_accelerations[2];
    debug << "After accel clip." << endl;
    // set the current speeds 
    m_speed_x = m_speed_x + acceleration_x*timestep;
    m_speed_y = m_speed_y + acceleration_y*timestep;
    m_speed_yaw = m_speed_yaw + acceleration_yaw*timestep;
    
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
void NUWalk::walkToPoint(double time, const vector<float>& position)
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
    m_point_timestamp = nusystem->getTime();
}

/*! @brief The primary walk function. This takes in the sensor data and the walk command to produce new nuactionators data
 */
void NUWalk::doWalk()
{
    // implementation is walk engine dependent
}

/*! @brief Sets the walk parameters to those specified in the variable
    @param walkparameters the new walk parameters for the walk and gain engines
 */
void NUWalk::setWalkParameters(WalkParameters& walkparameters)
{
    // walkparameters >> m_gait_*
    walkparameters.getArmGains(m_gait_arm_gains);
    walkparameters.getTorsoGains(m_gait_torso_gains);
    walkparameters.getLegGains(m_gait_leg_gains);
    
    walkparameters.getParameters(m_gait_walk_parameters);
    walkparameters.getMaxSpeeds(m_gait_max_speeds);
    walkparameters.getMaxAccelerations(m_gait_max_accelerations);
}

/*! @brief Gets the walk parameters and stores them in the passed variable
    @param walkparameters the storage variable for the current parameters
 */
void NUWalk::getWalkParameters(WalkParameters& walkparameters)
{
    // m_gait_* >> walkparameters
    walkparameters.setArmGains(m_gait_arm_gains);
    walkparameters.setTorsoGains(m_gait_torso_gains);
    walkparameters.setLegGains(m_gait_leg_gains);
    
    walkparameters.setParameters(m_gait_walk_parameters);
    walkparameters.setMaxSpeeds(m_gait_max_speeds);
    walkparameters.setMaxAccelerations(m_gait_max_accelerations);
}


