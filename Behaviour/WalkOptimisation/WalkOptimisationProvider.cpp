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
#include "Tools/Math/StlVector.h"

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
#include "nubotconfig.h"
#include "nubotdataconfig.h"
#include "targetconfig.h"

#include <boost/random.hpp>
#include <sys/types.h>
#include <unistd.h>
#include <signal.h>

#define USE_PGRL
#define USE_MO
#define USE_COST
#define USE_STIFFNESS

WalkOptimisationProvider::WalkOptimisationProvider(Behaviour* manager) : BehaviourFSMProvider(manager)
{
    #if DEBUG_BEHAVIOUR_VERBOSITY > 1
        debug << "WalkOptimisationProvider::WalkOptimisationProvider" << endl;
    #endif
    loadId();
    loadWayPoints();
    #ifdef TARGET_IS_DARWIN
        loadParameters("DarwinWalkStart");
    #else
        loadParameters("NBWalkStart");
    #endif
    initOptimiser();

    if (not m_optimiser)
        m_log.open((DATA_DIR + "/Optimisation/" + m_parameters.getName() + ".log").c_str(), fstream::out);
    else
        m_log.open((DATA_DIR + "/Optimisation/" + m_id + "Log.log").c_str(), fstream::out | fstream::app);

    m_generate = new GenerateWalkParametersState(this);
    m_evaluate = new EvaluateWalkParametersState(this);
    m_paused = new PausedWalkOptimisationState(this);
    m_state = m_paused;
    
    m_iteration_count = 0;
    m_fall_count = 0;
}

WalkOptimisationProvider::~WalkOptimisationProvider()
{
    delete m_generate;
    delete m_evaluate;
    delete m_paused;
    
    delete m_optimiser;
}

/*! @brief Handles state transitions common to all walk optimisation states
 * 				- In webots this includes the timely termination of the simulation, and the automatix transition from paused to generate
 * 				- On a real robot this handles the automatic transition from paused to generate
 */
BehaviourState* WalkOptimisationProvider::nextStateCommons()
{
    while (m_game_info->getCurrentState() != GameInformation::PlayingState)
        m_game_info->doManualStateChange();
    
    #ifdef TARGET_IS_NAOWEBOTS
		if (not m_optimiser and Platform->getTime() > 20*60*1e3)
		{
			kill(getppid(), SIGKILL);
			kill(getpid(), SIGKILL);
		}
		else if (m_iteration_count + 9*m_fall_count > 2000)
		{
			kill(getppid(), SIGKILL);
			kill(getpid(), SIGKILL);
		}

		if (m_state == m_paused and Platform->getTime() > 5000)
			return m_generate;
		else
			return m_state;
	#else
        if (m_state == m_paused and Platform->getTime() > 10000)
            return m_generate;
        else
            return m_state;
    #endif
}

/*! @brief Does behaviour common to all walk optimisations states */
void WalkOptimisationProvider::doBehaviourCommons()
{;
    if (m_previous_state == m_paused and m_state == m_generate)
        m_jobs->addMotionJob(new WalkParametersJob(m_parameters)); 
	#ifndef USE_VISION		// if there is no vision then just fix the head in (0,0) position
		m_jobs->addMotionJob(new HeadJob(0,vector<float>(2,0)));
	#endif
}

void WalkOptimisationProvider::tickOptimiser()
{    
    #if DEBUG_BEHAVIOUR_VERBOSITY > 1
        debug << "WalkOptimisationProvider::tickOptimiser" << endl;
    #endif 
    // update the optimiser and give the next set of parameters to the walk engine
	#ifdef USE_MO
        vector<float> fitness = calculateFitnesses();
	#else
        float fitness = calculateFitness();
	#endif
    if (m_optimiser)
    {            
		m_optimiser->setParametersResult(fitness);
        vector<float> nextparameters = m_optimiser->getNextParameters();
        m_parameters.set(nextparameters);
    }
    m_jobs->addMotionJob(new WalkParametersJob(m_parameters));
    
    #if DEBUG_BEHAVIOUR_VERBOSITY > 1
			debug << "WalkOptimisationProvider::tickOptimiser() new parameters: " << m_parameters.getAsVector() << endl;
    #endif

    // save the state of the optimiser, and the walk parameters in case of hardware failure.
    if (m_optimiser)
        m_optimiser->save();
    m_iteration_count++;
}

vector<float> WalkOptimisationProvider::calculateFitnesses()
{
    vector<float> fitness(2,0);
    float speed = 0;
    float cost = 0;
    bool unstable = true;

    if (not m_generate->success())
    {
		#if DEBUG_BEHAVIOUR_VERBOSITY > 2
			debug << "WalkOptimisationProvider::calculateFitness() Generate state was not successful" << endl;
		#endif
		float distance = max(10.0f, m_generate->distance());
		float duration = max(300.0f, m_generate->duration());
		float energy = max(20.0f, m_generate->energy());
		speed = 1000*distance/duration;
		cost = 100*energy/(9.81*4.6*distance);

		// penalise for falling
		speed *= 0.5*distance/333.0;
		cost /= 0.5*distance/333.0;
    }
    else if (not m_evaluate->success())
    {
		#if DEBUG_BEHAVIOUR_VERBOSITY > 2
			debug << "WalkOptimisationProvider::calculateFitness() Evaluate state was not successful" << endl;
		#endif
		float distance = max(10.0f, m_generate->distance() + m_evaluate->distance());
		float duration = max(300.0f, m_generate->duration() + m_evaluate->duration());
		float energy = max(20.0f, m_generate->energy() + m_evaluate->energy());
		speed = 1000*distance/duration;
		cost = 100*energy/(9.81*4.6*distance);

		// penalise for falling
		speed *= 0.5*distance/333.0;
		cost /= 0.5*distance/333.0;
    }
    else
    {
		#if DEBUG_BEHAVIOUR_VERBOSITY > 2
			debug << "WalkOptimisationProvider::calculateFitness() Evaluate state was successful" << endl;
		#endif
    	speed = 1000*m_evaluate->distance()/m_evaluate->duration();
    	cost = 100*m_evaluate->energy()/(9.81*4.6*m_evaluate->distance());
    	unstable = false;
    }

	#if DEBUG_BEHAVIOUR_VERBOSITY > 2
		debug << "WalkOptimisationProvider::calculateFitness() speed: " << speed << " " << cost << endl;
	#endif

    #ifdef TARGET_IS_NAOWEBOTS
        //speed *= normalDistribution(1, 0.105);      // 0.045, 0.055, 0.065
        //cost *= normalDistribution(1, 0.105);
    #endif
    
    fitness[0] = speed;                      	// speed--based fitness
    fitness[1] = 180/(4+cost);                  // cost--based fitness
    //fitness = 20*pow(speed,2)/(9.81*m_parameters.getAsVector()[18]);      // froude--based fitness
    m_log << m_iteration_count << ", " << fitness[1] << ", " << speed << ", " << cost << ", " << unstable << ", " << m_parameters.getAsVector() << endl << flush;

    // update fall count    
    if (unstable)
        m_fall_count++;    

	#if DEBUG_BEHAVIOUR_VERBOSITY > 1
        debug << "WalkOptimisationProvider::calculateFitness() " << fitness << " for " << m_parameters.getAsVector() << endl;
    #endif

    return fitness;
}

float WalkOptimisationProvider::calculateFitness()
{
	#if defined(USE_SPEED)
		return calculateFitnesses()[0];
	#elif defined(USE_COST)
		return calculateFitnesses()[1];
	#endif
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

/*! @brief Loads the way points from WayPoints.cfg into m_way_points
 */
void WalkOptimisationProvider::loadWayPoints()
{
	ifstream points_file((CONFIG_DIR + string("Motion/Optimisation/WayPoints.cfg")).c_str());
	if (points_file.is_open())
	{
		points_file >> m_way_points;
		#if DEBUG_BEHAVIOUR_VERBOSITY > 0
			debug << "WalkOptimisationProvider::m_way_points " << m_way_points << endl;
		#endif
		points_file.close();
	}
	else
		errorlog << "WalkOptimisationProvider::WalkOptimsationProvider. Unable to load WalkPoints.cfg" << endl;
}

/* @brief Loads the optimisation episode's id from Count.txt. The id is stored in m_id
 */
void WalkOptimisationProvider::loadId()
{
	fstream id_file((DATA_DIR + "/Optimisation/Count.txt").c_str());
	if (id_file.is_open())
	{
		int int_id = 0;
		id_file >> int_id;
		int_id++;
		stringstream ss;
		ss << int_id;
		m_id = ss.str();
		id_file.seekp(0, fstream::beg);
		id_file << int_id << endl;
		id_file.close();
	}
	#if DEBUG_BEHAVIOUR_VERBOSITY > 0
		debug << "Episode Id: " << m_id << endl;
	#endif
	cout << m_id << endl;
}

/*! @brief Loads the initial set of walk parameters for the optimisation episode
 *  @param name the name of the parameters to load
 */
void WalkOptimisationProvider::loadParameters(const string& name)
{
	m_parameters.load(name);
	#if DEBUG_BEHAVIOUR_VERBOSITY > 1
		debug << "Initial Parameters: " << m_parameters << endl;
	#endif
}

/*! @brief Initialises the optimiser
 */
void WalkOptimisationProvider::initOptimiser()
{
	vector<Parameter> parameters = m_parameters.getAsParameters();
	#if not defined(USE_STIFFNESS)
		parameters.resize(parameters.size() - 6);           // remove the stiffnesses from the parameter set
	#endif

	#if defined(USE_EHCLS)
		m_optimiser = new EHCLSOptimiser(m_id + "EHCLS", parameters);
	#elif defined(USE_PGRL)
		m_optimiser = new PGRLOptimiser(m_id + "PGRL", parameters);
	#elif defined(USE_PSO)
		m_optimiser = new PSOOptimiser(m_id + "PSO", parameters);
	#else
		m_optimiser = NULL;
	#endif

	if (m_optimiser)
		m_parameters.set(m_optimiser->getNextParameters());
}

/*! @brief Returns a normal random variable from the normal distribution with mean and sigma
 */
float WalkOptimisationProvider::normalDistribution(float mean, float sigma)
{
    static unsigned int seed = 1e6*Platform->getRealTime()*Platform->getRealTime()*Platform->getRealTime();          // I am hoping that at least one of the three calls is different for each process
    static boost::mt19937 generator(seed);                       // you need to seed it here with an unsigned int!
    static boost::normal_distribution<float> distribution(0,1);
    static boost::variate_generator<boost::mt19937, boost::normal_distribution<float> > standardnorm(generator, distribution);
    
    float z = standardnorm();       // take a random variable from the standard normal distribution
    float x = mean + z*sigma;       // then scale it to belong to the specified normal distribution
    
    return x;
}

