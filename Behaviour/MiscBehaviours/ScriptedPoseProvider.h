/*! @file ScriptedPoseProvider.h
    @brief Declaration of a behaviour provider for pose purposes
 
    @class ScriptedPoseProvider
    @brief A special behaviour for calibrating images.
 

    @author Aaron Wong
 
  Copyright (c) 2010 Aaron Wong
 
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

#ifndef SCRIPTEDPOSEPROVIDER_H
#define SCRIPTEDPOSEPROVIDER_H

#include "../BehaviourProvider.h"

class JobList;
class NUSensorsData;
class NUActionatorsData;
class FieldObjects;
class GameInformation;
class TeamInformation;

#include <vector>
#include <string>
#include "Motion/Tools/MotionScript.h"

class ScriptedPoseProvider : public BehaviourProvider
{
public:
    ScriptedPoseProvider(Behaviour* manager);
    ~ScriptedPoseProvider();
protected:
    void doBehaviour();
    void doSelectedMotion();


private:
    int m_pitch_index;
    int m_yaw_index;
    int m_num_pitch_motions;
    int m_num_yaw_motions;
    int isStart;
    bool m_saving_images;
    bool m_script_playing;
    MotionScript m_script;

};


#endif

