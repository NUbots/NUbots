/*! @file NUWalk.h
    @brief Declaration of nuwalk (abstract) class
 
    @class NUWalk
    @brief A module to provide locomotion
 
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

#ifndef NUWALK_H
#define NUWALK_H

#include "NUPlatform/NUSensors/NUSensorsData.h"
#include "NUPlatform/NUActionators/NUActionatorsData.h"
#include "Walks/WalkParameters.h"

class NUWalk
{
public:
    static NUWalk* getWalkEngine();
    virtual ~NUWalk();
    
    void process(NUSensorsData* data, NUActionatorsData* actions);
    void walkSpeed(const vector<float>& speed);
    void walkToPoint(double time, const vector<float>& position);
    
    virtual void setWalkParameters(WalkParameters& walkparameters);
    virtual void getWalkParameters(WalkParameters& walkparameters);
protected:
    virtual void doWalk();
private:
public:
protected:
    NUSensorsData* m_data;                          //!< local pointer to the latest sensor data
    NUActionatorsData* m_actions;                   //!< local pointer to the next actionators data
    
    float m_speed_x;                                //!< the current x speed in cm/s
    float m_speed_y;                                //!< the current y speed in cm/s
    float m_speed_yaw;                              //!< the current rotation speed in rad/s
    double m_speed_timestamp;                       //!< the timestamp of the last speed command
    
    double m_point_time;                            //!< the desired time to reach the current target point in milliseconds from now
    float m_point_x;                                //!< the current target point's x position in cm
    float m_point_y;                                //!< the current target point's y position in cm
    float m_point_theta;                            //!< the current target point's final orientation relative to the current in radians
    double m_point_timestamp;                       //!< the timestamp of the last point command
    
    // A semi-standard way of storing gait parameters for every walk engine
    vector<vector<float> > m_gait_arm_gains;        //!< the arm gains over a gait cycle
    vector<vector<float> > m_gait_torso_gains;      //!< the torso gains over a gait cycle
    vector<vector<float> > m_gait_leg_gains;        //!< the leg gains over a gait cycle
    
    vector<vector<WalkParameters::Parameter> > m_gait_walk_parameters;  //!< the walk engine parameters over a gait cycle

private:
};

#endif

