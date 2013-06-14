/*!  @file ScriptTunerStates.h
    @brief Provider of Head behaviour Testing behaviour. Darwin simply stands, observes and localises. Modified from Zombie state.
    @author Jake Fountain

    @author Jake Fountain

    Copyright (c) 2012 Jake Fountain

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

#ifndef SCRIPT_TUNER_STATES_H
#define SCRIPT_TUNER_STATES_H

#include "Behaviour/BehaviourState.h"
#include "ScriptTunerProvider.h"

#include "Infrastructure/Jobs/JobList.h"
#include "Infrastructure/NUSensorsData/NUSensorsData.h"
#include "Infrastructure/NUActionatorsData/NUActionatorsData.h"
#include "Infrastructure/FieldObjects/FieldObjects.h"
#include "Infrastructure/TeamInformation/TeamInformation.h"
#include "Infrastructure/NUBlackboard.h"
#include "Infrastructure/Jobs/MotionJobs/KickJob.h"
#include "Infrastructure/Jobs/MotionJobs/WalkJob.h"
#include "Infrastructure/Jobs/MotionJobs/HeadJob.h"
#include "Infrastructure/Jobs/MotionJobs/HeadTrackJob.h"
#include "Infrastructure/Jobs/MotionJobs/HeadPanJob.h"
#include "Infrastructure/Jobs/MotionJobs/HeadNodJob.h"
#include "Infrastructure/Jobs/MotionJobs/MotionFreezeJob.h"
#include "Infrastructure/GameInformation/GameInformation.h"
#include <thread>
#include <chrono>

#include "Behaviour/Common/HeadBehaviour.h"


#include "debug.h"

using std::vector;
using std::string;

class ScriptTunerSubState : public BehaviourState
{
public:
    ScriptTunerSubState(ScriptTunerProvider* provider){m_provider = provider;}
protected:
    ScriptTunerProvider* m_provider;
};

// ----------------------------------------------------------------------------------------------------------------------- PausedState
class ScriptTunerState : public ScriptTunerSubState
{
public:
    vector<vector<vector<float> > > frames;//(frames)_ij = [position,gain] of motor j at frame i.
    vector<float> times;

    ScriptTunerState(ScriptTunerProvider* provider) : ScriptTunerSubState(provider) {

    }

    BehaviourState* nextState() {return m_provider->m_state;}
    void doState();
    void editCurrentFrame();
    /*! @brief Loads a motion script from a file into the member variables frames and times */
    bool loadScript(string filename);
     /*! @brief Applies frame, given by frames[frame], to robot.*/
    void applyFrameToRobot();
     /*! @brief Reads motor positions from robot and saves them to the current frame, frames[frame].*/
    void saveCurrentFrame();
     /*! @brief Writes the vector frames to script file.*/
    void saveScriptToFile(string filename);
     /*! @brief Adds a new frame to the vector of frames.*/
    void addFrame(string argument);

    void interpretSeekCommand(string command);

    void moveToFrame(int frame_number);

    void exitScript();

    void playScript();

    /*! @brief Applies positional or torque settings to motors.
        @param other_parameters = "<position> <gain>". If gain is not specified a default value is used.*/
    void applyRequestToMotors(string motorID,string other_parameters);

};

#endif

