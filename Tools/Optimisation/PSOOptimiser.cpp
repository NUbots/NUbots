/*! @file PSOOptimiser.cpp
    @brief Implemenation of PSOOptimiser class
 
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

#include "PSOOptimiser.h"
#include "Parameter.h"

#include "debug.h"
#include "nubotdataconfig.h"

/*! @brief Constructor for abstract optimiser
 	@param name the name of the optimiser. The name is used in debug logs, and is used for load/save filenames by default
 	@param parameters the initial seed for the optimisation
 */
PSOOptimiser::PSOOptimiser(std::string name, vector<Parameter> parameters) : Optimiser(name, parameters)
{
    m_c1 = 0.5;
    m_c2 = 0.5;
    m_inertia = 0.95;
    m_num_dimensions = parameters.size();
    m_num_particles = 20;
    
    srand(static_cast<unsigned int> (clock()*clock()*clock()));

    initSwarm();
}

void PSOOptimiser::initSwarm()
{
    // we want to start the swarm around initial_parameters
    debug << "Initial Swarm: " << endl;
    for (int i=0; i<m_num_particles; i++)
    {
    	m_swarm_best.push_back(m_initial_parameters);
        m_swarm_best_fitness.push_back(0);
        
        vector<float> r = getRandVector();
        vector<Parameter> particle = m_initial_parameters;
        for (int j=0; j<m_num_dimensions; j++)
        {
            float range = 0.5*(m_initial_parameters[j].max() - m_initial_parameters[j].min());
            particle[j].set(range*r[j] - 0.5*range + m_initial_parameters[j].get());
        }
        debug << i << ": " << Parameter::getAsVector(particle) << endl;
        m_swarm_position.push_back(particle);
        m_swarm_velocity.push_back(vector<float>(m_num_dimensions,0));
    }
    m_best = m_initial_parameters;
    m_best_fitness = 0;
}

/*! @brief Destructor for the abstract optimiser */
PSOOptimiser::~PSOOptimiser()
{
}

void PSOOptimiser::setParametersResult(float fitness)
{
    debug << "PSOOptimiser::setParametersResult fitness: " << fitness << endl;
	m_swarm_fitness.push_back(fitness);
    if (m_swarm_fitness.size() == (unsigned int) m_num_particles)
        updateSwarm();
}

vector<float> PSOOptimiser::getNextParameters()
{
    return Parameter::getAsVector(m_swarm_position[m_swarm_fitness.size()]);
}

void PSOOptimiser::updateSwarm()
{
    debug << "Fitnesses: " << m_swarm_fitness << endl;
    // update the personal and global bests
    for (int i=0; i<m_num_particles; i++)
    {
        if (m_swarm_fitness[i] > m_swarm_best_fitness[i])
        {
            m_swarm_best_fitness[i] = m_swarm_fitness[i];
            m_swarm_best[i] = m_swarm_position[i];
        }
        
        if (m_swarm_fitness[i] > m_best_fitness)
        {
            m_best_fitness = m_swarm_fitness[i];
            m_best = m_swarm_position[i];
        }
    }
    
    debug << "Personal bests: " << m_swarm_best_fitness << endl;
    debug << "Global best: " << m_best_fitness << " " << Parameter::getAsVector(m_best) << endl;
    
    debug << "Current Swarm:" << endl;
    // update the positions and velocities of the particles
    for (int i=0; i<m_num_particles; i++)
    {
        vector<float> r1 = getRandVector();
        vector<float> r2 = getRandVector();
        m_swarm_velocity[i] = m_inertia*m_swarm_velocity[i] + m_c1*r1*(m_swarm_best[i] - m_swarm_position[i]) + m_c2*r2*(m_best - m_swarm_position[i]);
        m_swarm_position[i] += m_swarm_velocity[i];
        
        debug << "pos " << i << ": " << Parameter::getAsVector(m_swarm_position[i]) << endl;
        debug << "vel" << i << ": " << m_swarm_velocity[i] << endl;
    }
    
    // clear the fitnesses
    m_swarm_fitness.clear();
}

vector<float> PSOOptimiser::getRandVector()
{
    vector<float> result;
    result.reserve(m_num_dimensions);
    for (int i=0; i<m_num_dimensions; i++)
        result.push_back(static_cast<float>(rand())/RAND_MAX);
    return result;
}

void PSOOptimiser::summaryTo(ostream& stream)
{
}

void PSOOptimiser::toStream(ostream& o) const
{
}

void PSOOptimiser::fromStream(istream& i)
{
}

