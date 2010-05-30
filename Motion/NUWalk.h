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

#include "Walks/WalkParameters.h"
class NUSensorsData;
class NUActionatorsData;

class WalkJob;
class WalkToPointJob;
class WalkParametersJob;

class NUWalk
{
public:
    static NUWalk* getWalkEngine();
    NUWalk();
    virtual ~NUWalk();
    virtual void freeze();
    virtual void kill();
    
    void process(NUSensorsData* data, NUActionatorsData* actions);
    void process(WalkJob* job);
    void process(WalkToPointJob* job);
    void process(WalkParametersJob* job);
    
    virtual void setWalkParameters(const WalkParameters& walkparameters);
    WalkParameters& getWalkParameters();
    
    virtual void setArmEnabled(bool leftarm, bool rightarm);
    
    void getCurrentSpeed(vector<float>& currentspeed);
protected:
    virtual void doWalk() = 0;
    
    virtual void enableWalk();
    bool inInitialPosition();
    void moveToInitialPosition();

    void setTargetSpeed(const vector<float>& speed);
    void setTargetPoint(double time, const vector<float>& position);
    void calculateCurrentSpeed();

private:
public:
protected:
    NUSensorsData* m_data;                          //!< local pointer to the latest sensor data
    NUActionatorsData* m_actions;                   //!< local pointer to the next actionators data
    
    double m_current_time;                          //!< the current time
    double m_previous_time;                         //!< the previous time doWalk was called
    
    bool m_walk_enabled;                            //!< true if the walk is enabled, false otherwise
    bool m_larm_enabled;                            //!< true if the walk is allowed to move the left arm
    bool m_rarm_enabled;                            //!< true if the walk is allowed to move the right arm
    
    // the target speeds before acceleration clipping
    float m_target_speed_x;                         //!< the current target x speed cm/s
    float m_target_speed_y;                         //!< the current target y speed cm/s
    float m_target_speed_yaw;                       //!< the current target yaw speed rad/s
    
    // the current speeds (use these ones in doWalk)
    float m_speed_x;                                //!< the current x speed in cm/s
    float m_speed_y;                                //!< the current y speed in cm/s
    float m_speed_yaw;                              //!< the current rotation speed in rad/s
    
    double m_point_time;                            //!< the desired time to reach the current target point in milliseconds from now
    float m_point_x;                                //!< the current target point's x position in cm
    float m_point_y;                                //!< the current target point's y position in cm
    float m_point_theta;                            //!< the current target point's final orientation relative to the current in radians
    
    WalkParameters m_walk_parameters;               //!< the current set of walk parameters
    
    vector<float> m_initial_larm;
    vector<float> m_initial_rarm;
    vector<float> m_initial_lleg;
    vector<float> m_initial_rleg;

private:
};

#endif

