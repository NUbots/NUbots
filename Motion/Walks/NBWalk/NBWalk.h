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
#include "NUPlatform/NUSensors/NUSensorsData.h"
#include "NUPlatform/NUActionators/NUActionatorsData.h"

#include "MotionProvider.h"
#include "WalkProvider.h"
#include "NullBodyProvider.h"

#include <vector>
#include <fstream>
using namespace std;

class NBWalk : public NUWalk
{
public:
    NBWalk();
    ~NBWalk();
protected:
    void doWalk();
private:
    void sendWalkCommand();
    void preProcess();
    void processJoints();
    void processStiffness();
    bool postProcess();
    void swapBodyProvider();    
    
    void updateNBSensors();
    void nuToNBJointOrder(const vector<float>& nujoints, vector<float>& nbjoints);
    void nbToNUJointOrder(const vector<float>& nbjoints, vector<float>& nujoints);
    void updateActionatorsData();
    
public:
protected:
private:
    
    // Pattern generation debugging
    ofstream m_pattern_debug;
    
    // The following variables are taken straight from NB's MotionSwitchboard.
    boost::shared_ptr<Sensors> nb_sensors;
    WalkProvider walkProvider;
    NullBodyProvider nullBodyProvider;
    
	MotionProvider * curProvider;
	MotionProvider * nextProvider;
    
	MotionProvider * curHeadProvider;
	MotionProvider * nextHeadProvider;
    
    std::vector <float> nextJoints;
    std::vector <float> nextStiffnesses;
    
	mutable bool newJoints; // Way to track if we ever use the same joints twice
    
    bool readyToSend;
    
    mutable pthread_mutex_t next_provider_mutex;
    mutable pthread_mutex_t next_joints_mutex;
    mutable pthread_mutex_t stiffness_mutex;
    
    bool noWalkTransitionCommand;
};

#endif

