/*! @file ALWalk.h
    @brief Declaration of walk class to use Aldebaran's
 
    @class ALWalk
    @brief A module to provide locomotion using Aldebaran's stuff
 
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

#ifndef ALWALK_H
#define ALWALK_H

#include "Motion/NUWalk.h"
class NUSensorsData;
class NUActionatorsData;

#include <almotionproxy.h>
#include <string>
using namespace AL;

class ALWalk : public NUWalk
{
public:
    ALWalk(NUSensorsData* data, NUActionatorsData* actions);
    ~ALWalk();
    
    void setWalkParameters(const WalkParameters& walkparameters);
    
    void kill();
    void enableWalk();
    void setArmEnabled(bool leftarm, bool rightarm);
protected:
    void doWalk();
    void initALConfig();
    void setALConfig();
    
    vector<float> convertToNUArmOrder(const vector<float>& data);
    vector<float> convertToNULegOrder(const vector<float>& data);
private:
public:
protected:
private:
    ALMotionProxy* m_al_motion;
    ALValue m_al_config;            //!< Walk Parameter config for m_al_motion->setMotionConfig()
    ALValue m_al_param;             //!< A single parameter of m_al_config
    
    ALValue m_al_stiffness_protection;
    double m_last_enabled_time;
};

#endif

