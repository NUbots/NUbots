/*! @file WalkOptimiserBehaviour.h
    @brief Declaration of WalkOptimiserBehaviour class
 
    @class WalkOptimiserBehaviour
    @brief A module to provide control the robot while walk optimisation is running
 
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

#ifndef WALKOPTIMISERBEHAVIOUR_H
#define WALKOPTIMISERBEHAVIOUR_H

#include "Behaviour/Jobs.h"
#include "Motion/NUWalk.h"
#include "NUPlatform/NUPlatform.h"
#include "NUPlatform/NUSensors/NUSensorsData.h"
#include "NUPlatform/NUActionators/NUActionatorsData.h"
#include "../WalkParameters.h"
#include "WalkOptimiser.h"

#include <string>
using namespace std;

class WalkOptimiserBehaviour
{
public:
    WalkOptimiserBehaviour(NUPlatform* p_platform, NUWalk* p_walk);
    ~WalkOptimiserBehaviour();
    
    void process(NUSensorsData* data, NUActionatorsData* actions);
    void process(JobList& joblist);
protected:
    void startTrial();
    void runTrial();
    void perturbRobot();
    void finishTrial();
    void respawn();
    
    void loadOptimiser();
    void saveOptimiser();
private:
public:
protected:
private:
    NUSensorsData* m_data;
    NUActionatorsData* m_actions;
    
    NUWalk* m_walk;
    WalkParameters m_walk_parameters;
    WalkOptimiser* m_optimiser;
    
    enum State 
    {
        Initial,
        Start,
        Trial
    };
    State m_state, m_previous_state;
    float m_respawn_x, m_respawn_y, m_respawn_bearing;
    float m_target_speed;
    float m_target_trial_duration;
    
    double m_trial_start_time;
    float m_trial_start_x, m_trial_start_y;
    float m_trial_energy_used;
    
    // Serialisation
    string m_saved_optimiser_filename;
};

#endif

