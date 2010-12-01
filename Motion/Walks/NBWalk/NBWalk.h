/*! @file NBWalk.h
    @brief Declaration of Northern Bites's walk engine 
 
    @class NBWalk
    @brief A module to provide locomotion using Northern Bites 2009 walk.
 
    This module is uses Johannes Strom's omni-directional walk engine detailed in the paper:
    An Implementation of an Omnidirectional Walk Engine on a Nao Robot, RoboCup 2009.
 
    As the Northern Bites are kind enough to share their code, most of the code for the implementation
    of this walk engine was written by them. The header of each file will indicate the author of that
    file, I will mark files I needed to modify.
 
    I am integrating the engine at the level of NB's MotionSwitchboard, that is this class will share
    much of its code with the MotionSwitchboard. However, there will be no mention of head or script providers.
 
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

#ifndef NBWALK_H
#define NBWALK_H

#include "Motion/NUWalk.h"
#include "WalkProvider.h"
class NUSensorsData;
class NUActionatorsData;

#include <vector>
#include <fstream>
using namespace std;

class NBWalk : public NUWalk
{
public:
    NBWalk(NUSensorsData* data, NUActionatorsData* actions);
    ~NBWalk();
    
    void kill();

    void setWalkParameters(const WalkParameters& walkparameters);
protected:
    void doWalk();
private:
    void sendWalkCommand();
    void processJoints();
    
    void updateNBSensors();
    void setGait();
    void nuToNBJointOrder(const vector<float>& nujoints, vector<float>& nbjoints);
    void nbToNUJointOrder(const vector<float>& nbjoints, vector<float>& nujoints);
    void nbToNULeftLegJointOrder(const vector<float>& nbjoints, vector<float>& nuleftlegjoints);
    void nbToNURightLegJointOrder(const vector<float>& nbjoints, vector<float>& nurightlegjoints);
    void nbToNULeftArmJointOrder(const vector<float>& nbjoints, vector<float>& nuleftarmjoints);
    void nbToNURightArmJointOrder(const vector<float>& nbjoints, vector<float>& nurightarmjoints);
    void updateActionatorsData();
    
public:
protected:
private:
    boost::shared_ptr<Sensors> nb_sensors;
    WalkProvider walkProvider;
    
    std::vector <float> nextJoints;
    mutable pthread_mutex_t next_joints_mutex;
    
    boost::shared_ptr<Gait> m_gait;
    
};

#endif

