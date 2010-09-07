/*! @file WalkOptimisationProvider.cpp
    @brief Implementation of chase ball behaviour class

    @author Jason Kulk
 
 Copyright (c) 2010 Jason Kulk
 
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

#include "WalkOptimisationProvider.h"
#include "GenerateWalkParametersState.h"
#include "EvaluateWalkParametersState.h"
#include "PausedWalkOptimisationState.h"

#include "Tools/Optimisation/EHCLSOptimiser.h"
#include "Tools/Optimisation/PGRLOptimiser.h"
#include "Tools/Optimisation/PSOOptimiser.h"
#include "Motion/Tools/MotionFileTools.h"

#include "Behaviour/Jobs/JobList.h"
#include "Behaviour/Jobs/MotionJobs/WalkParametersJob.h"

#include "debug.h"
#include "debugverbositybehaviour.h"
#include "nubotdataconfig.h"
#include "targetconfig.h"

WalkOptimisationProvider::WalkOptimisationProvider(Behaviour* manager) : BehaviourFSMProvider(manager)
{
    #if DEBUG_BEHAVIOUR_VERBOSITY > 1
        debug << "WalkOptimisationProvider::WalkOptimisationProvider" << endl;
    #endif
    fstream id_file((DATA_DIR + "/Optimisation/Count.txt").c_str());
    stringstream id;
    if (id_file.is_open())
    {
        int instance_id = 0;
        id_file >> instance_id;
        instance_id++;
        id_file.seekp(0);
        id_file << instance_id;
        id << instance_id;
        id_file.close();
    } 
    
    m_parameters.load("NBWalkStart");
    m_optimiser = new EHCLSOptimiser(id.str() + "EHCLS", m_parameters.getAsParameters());
    //m_optimiser = new PGRLOptimiser("PGRL", m_parameters.getAsParameters());    
    //m_optimiser = new PSOOptimiser("PSO", m_parameters.getAsParameters());
    m_log.open((DATA_DIR + "/Optimisation/" + id.str() + "Log.log").c_str());
    
    ifstream points_file((CONFIG_DIR + string("Motion/Optimisation/WayPoints.cfg")).c_str());
    if (points_file.is_open())
    {
        m_speed_points = MotionFileTools::toFloatMatrix(points_file);
        #if DEBUG_BEHAVIOUR_VERBOSITY > 0
            debug << "WalkOptimisationProvider::m_speed_points " << MotionFileTools::fromMatrix(m_speed_points) << endl;
        #endif
        m_stability_points = MotionFileTools::toFloatMatrix(points_file);
        #if DEBUG_BEHAVIOUR_VERBOSITY > 0
            debug << "WalkOptimisationProvider::m_stability_points " << MotionFileTools::fromMatrix(m_stability_points) << endl;
        #endif
        points_file.close();
    }
    else
        errorlog << "WalkOptimisationProvider::WalkOptimsationProvider. Unable to load WalkPoints.cfg" << endl;
    
    m_generate = new GenerateWalkParametersState(this);
    m_evaluate = new EvaluateWalkParametersState(this);
    m_paused = new PausedWalkOptimisationState(this);
    
    m_state = m_paused;
    
    m_iteration_count = -1;
    m_duration = 0;
    m_energy = 0;	
    m_stability = 0;
}

WalkOptimisationProvider::~WalkOptimisationProvider()
{
    delete m_generate;
    delete m_evaluate;
    delete m_paused;
    
    delete m_optimiser;
}

BehaviourState* WalkOptimisationProvider::nextStateCommons()
{
    while (m_game_info->getCurrentState() != GameInformation::PlayingState)
        m_game_info->doManualStateChange();
    
    #ifndef TARGET_IS_NAOWEBOTS    
        if (singleChestClick() or longChestClick())
        {
            if (m_state == m_paused)
                return m_generate;
            else
                return m_paused;
        }
        else
            return m_state;
    #else
        if (m_state == m_paused)
            return m_generate;
        else
            return m_state;
    #endif
}

void WalkOptimisationProvider::tickOptimiser()
{
    m_iteration_count++;
    // save the state of the optimiser, and the walk parameters in case of hardware failure.
    m_optimiser->save();
    
    // update the optimiser and give the next set of parameters to the walk engine
    if (m_iteration_count != 0)
        m_optimiser->setParametersResult(calculateFitness());
    vector<float> nextparameters = m_optimiser->getNextParameters();
    m_parameters.set(nextparameters);
    m_jobs->addMotionJob(new WalkParametersJob(m_parameters));
    
    // reset the fitness
    m_duration = 0;
    m_energy = 0;	
    m_stability = 0;
    
    #if DEBUG_BEHAVIOUR_VERBOSITY > 1
        debug << "WalkOptimisationProvider::tickOptimiser" << endl;
        m_parameters.summaryTo(debug);
        m_optimiser->summaryTo(debug);
    #endif
}

float WalkOptimisationProvider::calculateFitness()
{
    #if DEBUG_BEHAVIOUR_VERBOSITY > 1
        debug << "WalkOptimisationProvider::calculateFitness() " << m_duration << " " << m_energy << " " << m_stability << endl;
        m_parameters.summaryTo(debug);
        m_optimiser->summaryTo(debug);
    #endif
    float fitness = 0;
    float speed = 0;
    float cost = 0;
    if (m_duration == 0 and m_stability == 0)
    {	// we have fallen over before actually reaching the start of the evaluation
        fitness = 0;
    }
    else if (m_energy == 0 and m_stability == 0)
    {	// we have fallen over during the evaluation
        if (m_duration > 120000)
            fitness = 0;
        else
        {
            vector<float>& speeds = m_parameters.getMaxSpeeds();
            float travelleddistance = 0.1*speeds[0]*m_duration/1000;
            float pathdistance = calculatePathDistance();
            float predictedfalls = pathdistance/travelleddistance;
            fitness = pathdistance/(pathdistance/(0.1*speeds[0]) + predictedfalls*15);
        }
    }
    else if (m_stability == 0)
    {	// we are not using the stability metric
        speed = 1000*calculatePathDistance()/m_duration;			// cm/s
        cost = 100*m_energy/(9.81*4.6*calculatePathDistance()); 	// J/Nm
     	fitness = speed;   
    }
    else
    {	// we are not using the stability metric
        speed = 1000*calculatePathDistance()/m_duration;			// cm/s
        cost = 100*m_energy/(9.81*4.6*calculatePathDistance()); 	// J/Nm
        fitness = 1000*calculatePathDistance()/m_duration;
    }
    
    m_log << m_iteration_count << ", " << fitness << ", " << speed << ", " << cost << ", " << m_stability << ", " << m_parameters.getAsVector() << endl << flush;
    
    return fitness;
}

/*! @brief Returns the distance in cm of the path specified by m_points 
 @return the distance of the evaluation path 
 */
float WalkOptimisationProvider::calculatePathDistance()
{
    float distance = 0;
    for (size_t i=1; i<m_speed_points.size(); i++)
        distance += sqrt(pow(m_speed_points[i][0] - m_speed_points[i-1][0],2) + pow(m_speed_points[i][1] - m_speed_points[i-1][1],2));
    return distance;
}

/*! @brief Sets the duration of the current trial */
void WalkOptimisationProvider::setDuration(float duration)
{
    m_duration = duration;
}

/*! @brief Sets the energy used over the current trial */
void WalkOptimisationProvider::setEnergy(float energy)
{
    m_energy = energy;
}

/*! @brief Sets the stability of the parameters */
void WalkOptimisationProvider::setStability(float stability)
{
    m_stability = stability;
}

