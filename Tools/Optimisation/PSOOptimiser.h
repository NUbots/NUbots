/*! @file PSOOptimiser.h
    @brief Declaration of the PSO optimiser
 
    @class PSOOptimiser
    @brief An implementation of the Particle Swarm Optimisation algorithm
 
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

#ifndef PSO_OPTIMISER_H
#define PSO_OPTIMISER_H

#include "Optimiser.h"

#include <string>
#include <vector>
#include <iostream>



class PSOOptimiser : public Optimiser
{
public:
    PSOOptimiser(std::string name, std::vector<Parameter> parameters);
    ~PSOOptimiser();
    
    std::vector<float> getNextParameters();
    void setParametersResult(float fitness);
    
    void summaryTo(std::ostream& stream);

    std::vector<Parameter> getBest() const { return m_best;}

private:
    void initSwarm();
    void updateSwarm();
    
    
    void toStream(std::ostream& o) const;
    void fromStream(std::istream& i);
private:
    std::vector<std::vector<Parameter> > m_swarm_position;
    std::vector<std::vector<float> > m_swarm_velocity;
    std::vector<float> m_swarm_fitness;
    
    std::vector<std::vector<Parameter> > m_swarm_best;
    std::vector<float> m_swarm_best_fitness;
    std::vector<float> m_swarm_failures;
    std::vector<Parameter> m_best;
    float m_best_fitness;

    float m_inertia;
    float m_reset_limit;
    float m_reset_fraction;
    int m_num_particles;
    int m_num_dimensions;
};

#endif

