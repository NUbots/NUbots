/*! @file WalkParameters.h
    @brief Declaration of WalkParameters class
 
    @class WalkParameters
    @brief A module to store walk parameters
 
    Each walk engine must implement a function to get and save its parameters to a WalkParameters object.
    The walk parameters are separated into:
        - arm gains (0 to 100%)
        - torso gains (0 to 100%)
        - leg gains (0 to 100%)
        - parameters (walk engine parameters)
 
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

#ifndef WALKPARAMETERS_H
#define WALKPARAMETERS_H

#include "Tools/Optimisation/Parameter.h"

#include <vector>
#include <string>
#include <iostream>
#include <sstream>

class WalkParameters
{
public:
    WalkParameters();
    WalkParameters(const std::string& name);
    WalkParameters(const std::string& name, const std::vector<float>& maxspeeds, const std::vector<float>& maxaccels, const std::vector<Parameter>& parameters, const std::vector<std::vector<float> >& armgains, const std::vector<std::vector<float> >& torsogains, const std::vector<std::vector<float> >& leggains);
    ~WalkParameters();
    
    // get methods
    std::vector<float> getAsVector();
    std::vector<Parameter> getAsParameters();
    std::string& getName();
    std::vector<float>& getMaxSpeeds();
    std::vector<float>& getMaxAccelerations();
    std::vector<Parameter>& getParameters();
    std::vector<std::vector<float> >& getArmGains();
    std::vector<std::vector<float> >& getTorsoGains();
    std::vector<std::vector<float> >& getLegGains();
    
    // set methods
    void set(const std::vector<float>& data);
    void setName(const std::string& name);
    void setMaxSpeeds(const std::vector<float>& maxspeeds);
    void setMaxAccelerations(const std::vector<float>& maxaccels);
    void setParameters(const std::vector<Parameter>& parameters);
    void setArmGains(const std::vector<std::vector<float> >& armgains);
    void setTorsoGains(const std::vector<std::vector<float> >& torsogains);
    void setLegGains(const std::vector<std::vector<float> >& leggains);
    
    // display methods
    void summaryTo(std::ostream& output);
    void csvTo(std::ostream& output);
    
    // serialisation
    friend std::ostream& operator<< (std::ostream& output, const WalkParameters& p_walkparameters);
    friend std::ostream& operator<< (std::ostream& output, const WalkParameters* p_walkparameters);
    friend std::istream& operator>> (std::istream& input, WalkParameters& p_walkparameters);
    friend std::istream& operator>> (std::istream& input, WalkParameters* p_walkparameters);
    void save();
    void saveAs(const std::string& name);
    void load(const std::string& name);
    
    size_t size() const;
private:
    void setGains(std::vector<std::vector<float> >& gains, unsigned int& numgains, const std::vector<std::vector<float> >& newgains);
public:
private:
    std::string m_name;                             //!< the name of the walk parameter set
    std::vector<float> m_max_speeds;                //!< stores the maximum speeds (x,y,theta) allowed by the walk engine
    std::vector<float> m_max_accelerations;         //!< stores the maximum accelerations (x,y,theta) allowed by the walk engine
    
    std::vector<Parameter> m_parameters;            //!< stores the parameters for the walk engine
    
    std::vector<std::vector<float> > m_arm_gains;        //!< stores the arm gains for a walk
    unsigned int m_num_arm_gains;              //!< stores the total number of arm gains
    std::vector<std::vector<float> > m_torso_gains;      //!< stores the torso gains for a walk
    unsigned int m_num_torso_gains;            //!< stores the total number of torso gains
    std::vector<std::vector<float> > m_leg_gains;        //!< stores the leg gains for a walk
    unsigned int m_num_leg_gains;              //!< stores the total number of leg gains
};

#endif

