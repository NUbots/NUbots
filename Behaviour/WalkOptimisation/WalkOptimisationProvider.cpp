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

#include "Infrastructure/Jobs/JobList.h"
#include "Infrastructure/NUSensorsData/NUSensorsData.h"
#include "Infrastructure/NUActionatorsData/NUActionatorsData.h"
#include "Infrastructure/FieldObjects/FieldObjects.h"
#include "NUPlatform/NUPlatform.h"

#include "Infrastructure/Jobs/MotionJobs/HeadJob.h"
#include "Infrastructure/Jobs/MotionJobs/HeadPanJob.h"
#include "Infrastructure/Jobs/MotionJobs/HeadNodJob.h"
#include "Infrastructure/Jobs/MotionJobs/WalkJob.h"
#include "Infrastructure/Jobs/MotionJobs/WalkParametersJob.h"

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
        id << instance_id;
        id_file.seekp(0, fstream::beg);
        id_file << instance_id << endl;
        id_file.close();
    } 
    
    m_parameters.load("ALWalkAldebaran");
    vector<Parameter> parameters = m_parameters.getAsParameters();
    //parameters.resize(parameters.size() - 6);           // remove the stiffnesses from the parameter set!
    //m_optimiser = new EHCLSOptimiser(id.str() + "EHCLS", parameters);
    m_optimiser = new PGRLOptimiser(id.str() + "PGRL", parameters);    
    //m_optimiser = new PSOOptimiser(id.str() + "PSO", parameters);
    //m_optimiser = NULL;    
    m_log.open((DATA_DIR + "/Optimisation/" + id.str() + "Log.log").c_str(), fstream::out | fstream::app);
    
    if (m_optimiser)
        m_parameters.set(m_optimiser->getNextParameters());

    #if DEBUG_BEHAVIOUR_VERBOSITY > 1
        debug << "Initial Parameters: " << m_parameters << endl;
    #endif
    
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
    
    m_iteration_count = 0;
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
        if (m_state == m_paused and Platform->getTime() > 5000)
            return m_generate;
        else
            return m_state;
    #else
        if (Platform->getTime() > 20*60*60*1e3)
            exit(1);
        
        if (m_state == m_paused and Platform->getTime() > 1000)
            return m_generate;
        else
            return m_state;
    #endif
}

void WalkOptimisationProvider::doBehaviourCommons()
{;
    if (m_previous_state == m_paused and m_state == m_generate)
        m_jobs->addMotionJob(new WalkParametersJob(m_parameters)); 
    m_jobs->addMotionJob(new HeadJob(0,vector<float>(2,0)));
}

void WalkOptimisationProvider::tickOptimiser()
{    
    #if DEBUG_BEHAVIOUR_VERBOSITY > 1
        debug << "WalkOptimisationProvider::tickOptimiser" << endl;
    #endif 
    // update the optimiser and give the next set of parameters to the walk engine
    float fitness = calculateFitness();
    if (m_optimiser)
    {            
        m_optimiser->setParametersResult(fitness);
        vector<float> nextparameters = m_optimiser->getNextParameters();
        m_parameters.set(nextparameters);
    }
    m_jobs->addMotionJob(new WalkParametersJob(m_parameters));
    
    // reset the fitness
    m_duration = 0;
    m_energy = 0;	
    m_stability = 0;
    
    #if DEBUG_BEHAVIOUR_VERBOSITY > 1
        m_parameters.summaryTo(debug);
        if (m_optimiser)
            m_optimiser->summaryTo(debug);
    #endif

    // save the state of the optimiser, and the walk parameters in case of hardware failure.
    if (m_optimiser)
        m_optimiser->save();
    m_iteration_count++;
}

float WalkOptimisationProvider::calculateFitness()
{
    #if DEBUG_BEHAVIOUR_VERBOSITY > 1
        debug << "WalkOptimisationProvider::calculateFitness() " << m_duration << " " << m_energy << " " << m_stability << endl;
        m_parameters.summaryTo(debug);
        if (m_optimiser)
            m_optimiser->summaryTo(debug);
    #endif
    float fitness = 0;
    float speed = 0;
    float cost = 0;
    if (m_duration == 0 and m_stability == 0)
    {	// we have fallen over before actually reaching the start of the evaluation
        cost = 1000;        
        speed = 0;
    }
    else if (m_energy == 0 and m_stability == 0)
    {	// we have fallen over during the evaluation
        vector<float>& speeds = m_parameters.getMaxSpeeds();
        float travelleddistance = 0.1*speeds[0]*m_duration/1000;
        float pathdistance = calculatePathDistance();
        travelleddistance = min(travelleddistance, pathdistance);
        float predictedfalls = pathdistance/travelleddistance;
        
        cost = 100*(3000 + predictedfalls*(9.81*4.6*0.3)*4)/(9.81*4.6*travelleddistance);            // note: m_energy=0, so i cant use that in the metric
        speed = pathdistance/(pathdistance/1.5 + predictedfalls*15);
    }
    else if (m_stability == 0)
    {	// we are not using the stability metric
        cost = 100*m_energy/(9.81*4.6*calculatePathDistance()); 	// J/Nm
        speed = 1000*calculatePathDistance()/m_duration;			// cm/s
    }
    else
    {	// we are not using the stability metric
        cost = 100*m_energy/(9.81*4.6*calculatePathDistance()); 	// J/Nm
        speed = 1000*calculatePathDistance()/m_duration;			// cm/s
    }
    
    //fitness = speed;                      // speed--based fitness
    fitness = 180/(4+cost);                 // cost--based fitness
    //fitness = 20*pow(speed,2)/(9.81*m_parameters.getAsVector()[18]);      // froude--based fitness
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

/*! @brief Returns the distance required to stop for the current walk parameters */
float WalkOptimisationProvider::stoppingDistance()
{
    vector<float>& speeds = m_parameters.getMaxSpeeds();
    vector<float>& accels = m_parameters.getMaxAccelerations();
    float xd = pow(1.1*speeds[0],2)/(2*accels[0]);          // s = u^2/2a with a 10% margin for error
    float yd = pow(1.1*speeds[1],2)/(2*accels[1]);
    return sqrt(xd*xd + yd*yd);              
    
}

