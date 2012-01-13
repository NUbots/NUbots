/*! @file DarwinWalk.h
    @brief Declaration of the NUbot wrapper for the default darwin walk engine
 
    @class DarwinWalk
    @brief A module to provide locomotion using Darwin Default walk engine.
 
    The class is responsible for transforming the sensor values retrieved from the NUSensorsData
    instance into sensor values that can be used within the default darwin walk engine.
    These values are applied along with the current walk command and the walk engine generates a set
    of joint targets. These targets are retrived from the walk engine and transformed back into the
    nubots joint space. The targets are then applied to the motors via actions added to the
    NUActionatorsData instance.

    @author Aaron Wong
    @author Steven Nicklin
 
        Copyright (c) 2011 Aaron Wong, Jason Kulk, Steven Nicklin
 
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

#ifndef DARWINWALK_H
#define DARWINWALK_H
#include <vector>
#include "Motion/NUWalk.h"

class NUSensorsData;
class NUActionatorsData;
class DarwinJointMapping;

using namespace std;

class DarwinWalk : public NUWalk
{
public:
    DarwinWalk(NUSensorsData* data, NUActionatorsData* actions);
    ~DarwinWalk();
    void doWalk();
protected:
    void setDarwinSensor(int id,float joint);
    float getTarget(int id);
    void updateActionatorsData();
    void updateWalkEngineSensorData();
    DarwinJointMapping* m_joint_mapping;
    std::vector<int> m_darwin_ids;
};

#endif

