/*! @file WalkOptimiserBehaviour.cpp
    @brief Implementation of WalkOptimiserBehaviour class

    @author Jason Kulk
 
 Copyright (c) 2009, 2010 Jason Kulk
 
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

#include "WalkParameters.h"
#include "../Tools/MotionFileTools.h"

#include "debug.h"
#include "debugverbositynumotion.h"
#include "nubotdataconfig.h"

/*! @brief Construct a walk parameter set with no parameters
 */
WalkParameters::WalkParameters()
{
    m_name = "blank";
    m_num_arm_gains = 0; 
    m_num_torso_gains = 0;
    m_num_leg_gains = 0;
}

/*! @brief Constructs a walk parameter set with only a name. Presumably the parameters will be set later using the set functions
    @param name the name of this walk parameter set
 */
WalkParameters::WalkParameters(const string& name)
{
    m_name = name;
    m_num_arm_gains = 0;
    m_num_torso_gains = 0;
    m_num_leg_gains = 0;
}

/*! @brief Construct a walk parameter set with the given parameters
    @param name the name of the walk parameter set
    @param armgains the gains: [first phase, second phase, third phase, etc] where each phase [shoulder roll, shouler pitch, shouler yaw, elbow roll, elbow pitch]
    @param torsogains the gains: [first phase, second phase, third phase, etc] where each phase [torso roll, torso pitch, torso yaw]
    @param leggains the gains: [first phase, second phase, third phase, etc] where each phase [hip roll, hip pitch, hip yaw, knee, ankle roll, ankle pitch]
    @param parameters the walk engine parameters
 */
WalkParameters::WalkParameters(const string& name, const vector<float>& maxspeeds, const vector<float>& maxaccels, const vector<Parameter>& parameters, const vector<vector<float> >& armgains, const vector<vector<float> >& torsogains, const vector<vector<float> >& leggains)
{
    m_name = name;
    setMaxSpeeds(maxspeeds);
    setMaxAccelerations(maxaccels);
    setParameters(parameters);
    setArmGains(armgains);
    setTorsoGains(torsogains);
    setLegGains(leggains);
}

/*! @brief Destroy a walk parameter set
 */
WalkParameters::~WalkParameters()
{
}

/*! @brief Gets all of the values of every interesting walk parameter.
    @return a vector of floats containing all of the values of all of the interesting walk parameters.
 */
vector<float> WalkParameters::getAsVector()
{
    vector<float> data;
    data.reserve(size());
    
    for (size_t i=0; i<m_max_speeds.size(); i++)
        data.push_back(m_max_speeds[i]);
    
    for (size_t i=0; i<m_max_accelerations.size(); i++)
        data.push_back(m_max_accelerations[i]);
    
    for (size_t i=0; i<m_parameters.size(); i++)
        data.push_back(m_parameters[i].get());
    
    for (size_t i=0; i<m_leg_gains.size(); i++)
        for (size_t j=0; j<m_leg_gains[i].size(); j++)
            data.push_back(m_leg_gains[i][j]);
    
    return data;
}

/*! @brief Gets all of the values of every interesting walk parameter.
    @return a vector of floats containing all of the values of all of the interesting walk parameters.
 */
vector<Parameter> WalkParameters::getAsParameters()
{
    vector<Parameter> data;
    data.reserve(size());
    
    // Need to hand-code the maximum speeds
    data.push_back(Parameter("Velocity", m_max_speeds[0], 0, 70));
    data.push_back(Parameter("Velocity", m_max_speeds[1], 0, 70));
    data.push_back(Parameter("Velocity", m_max_speeds[2], 0, 2));

    data.push_back(Parameter("Acceleration", m_max_accelerations[0], 0, 140));
    data.push_back(Parameter("Acceleration", m_max_accelerations[1], 0, 140));
    data.push_back(Parameter("Acceleration", m_max_accelerations[2], 0, 4));
    
    for (size_t i=0; i<m_parameters.size(); i++)
        data.push_back(m_parameters[i]);
    
    for (size_t i=0; i<m_leg_gains.size(); i++)
    {
        for (size_t j=0; j<m_leg_gains[i].size(); j++)
        {
            Parameter p("Gain", m_leg_gains[i][j], 25, 100);
            data.push_back(p);
        }
    }
    
    return data;
}

/*! @brief Returns the walk parameter set's name
    @return the walk parameter name
 */
string& WalkParameters::getName()
{
    return m_name;
}

/*! @brief Gets the walk engine max speed values stored here
 @returns the walk engine max speeds stored as [x (cm/s), y (cm/s), theta (rad/s)].
 */
vector<float>& WalkParameters::getMaxSpeeds()
{
    return m_max_speeds;
}

/*! @brief Gets the walk engine max acceleration values stored here
 @return the maximum accelerations as [x (cm/s/s), y (cm/s/s), theta (rad/s/s)].
 */
vector<float>& WalkParameters::getMaxAccelerations()
{
    return m_max_accelerations;
}

/*! @brief Gets the walk engine parameters stored here
 @return the walk engine parameters stored in this object
 */
vector<Parameter>& WalkParameters::getParameters()
{
    return m_parameters;
}

/*! @brief Gets the arm gains stored in this WalkParameter class
    @return returns the armgains as: [first phase, second phase, third phase, etc] where each phase [shoulder roll, shouler pitch, shouler yaw, elbow roll, elbow pitch]
 */
vector<vector<float> >& WalkParameters::getArmGains()
{
    return m_arm_gains;
}

/*! @brief Gets the torso gains stored in this WalkParameter class
    @return the torsogains as [first phase, second phase, third phase, etc] where each phase [torso roll, torso pitch, torso yaw]
 */
vector<vector<float> >& WalkParameters::getTorsoGains()
{
    return m_torso_gains;
}

/*! @brief Gets the leg gains stored in this WalkParameter class
    @return the leggains as [first phase, second phase, third phase, etc] where each phase [hip roll, hip pitch, hip yaw, knee, ankle roll, ankle pitch]
 */
vector<vector<float> >& WalkParameters::getLegGains()
{
    return m_leg_gains;
}

/*! @brief Sets all of the interesting walk parameters with the given values
    @param data, a vector containing all of the new and interesting walk parameters
 */
void WalkParameters::set(const vector<float>& data)
{
    if (data.size() < m_max_speeds.size() + m_max_accelerations.size() + m_parameters.size())
        return;
    else
    {
        size_t offset = 0;
        
        for (size_t i=0; i<m_max_speeds.size(); i++)
            m_max_speeds[i] = data[i+offset];
        offset += m_max_speeds.size();
        
        for (size_t i=0; i<m_max_accelerations.size(); i++)
            m_max_accelerations[i] = data[i+offset];
        offset += m_max_accelerations.size();

        for (size_t i=0; i<m_parameters.size(); i++)
            m_parameters[i].set(data[i+offset]);
        offset += m_parameters.size();
        
        if (data.size() < size())               // check if the new parameters includes stiffnesses for the legs, 
            return;
        else
        {
            for (size_t i=0; i<m_leg_gains.size(); i++)
            {
                for (size_t j=0; j<m_leg_gains[i].size(); j++)
                    m_leg_gains[i][j] = data[offset+j];
                offset += m_leg_gains[i].size();
            }
        }
    }
}

/*! @brief Sets the walk parameter set's name
    @param name the new name
 */
void WalkParameters::setName(const string& name)
{
    m_name = name;
}

/*! @brief Sets the walk engine max speeds stored here
 @param maxspeeds the walk engine max speeds stored as [x (cm/s), y (cm/s), theta (rad/s)].
 */
void WalkParameters::setMaxSpeeds(const vector<float>& maxspeeds)
{
    vector<float> tempspeeds;
    for (unsigned int i=0; i<maxspeeds.size(); i++)
    {
        if (maxspeeds[i] < 0)
            tempspeeds.push_back(0);
        else
            tempspeeds.push_back(maxspeeds[i]);
    }
    m_max_speeds = tempspeeds;
}

/*! @brief Sets the walk engine max accelerations stored here
 @param maxspeeds the walk engine max accelerations stored as [x (cm/s/s), y (cm/s/s), theta (rad/s/s)].
 */
void WalkParameters::setMaxAccelerations(const vector<float>& maxaccels)
{
    vector<float> tempaccels;
    for (unsigned int i=0; i<maxaccels.size(); i++)
    {
        if (maxaccels[i] < 0)
            tempaccels.push_back(0);
        else
            tempaccels.push_back(maxaccels[i]);
    }
    m_max_accelerations = tempaccels;
}


/*! @brief Sets the walk engine parameters stored here
    @param parameters the walk engine parameters
 */
void WalkParameters::setParameters(const vector<Parameter>& parameters)
{
    m_parameters = parameters;
}

/*! @brief Sets the arm gains
    @param armgains the gains: [first phase, second phase, third phase, etc] where each phase [shoulder roll, shouler pitch, shouler yaw, elbow roll, elbow pitch]
 */
void WalkParameters::setArmGains(const vector<vector<float> >& armgains)
{
    setGains(m_arm_gains, m_num_arm_gains, armgains);
}

/*! @brief Sets the torso gains stored in this WalkParameter class
    @param torsogains the gains: [first phase, second phase, third phase, etc] where each phase [torso roll, torso pitch, torso yaw]
 */
void WalkParameters::setTorsoGains(const vector<vector<float> >& torsogains)
{
    setGains(m_torso_gains, m_num_torso_gains, torsogains);
}

/*! @brief Sets the leg gains stored in this WalkParameter class
    @param leggains the gains: [first phase, second phase, third phase, etc] where each phase [hip roll, hip pitch, hip yaw, knee, ankle roll, ankle pitch]
 */
void WalkParameters::setLegGains(const vector<vector<float> >& leggains)
{
    setGains(m_leg_gains, m_num_torso_gains, leggains);
}

/*! @brief Sets the gains with the new gains being careful to clip to 0 and 100.
    @param gains the data member to be updated
    @param newgains the new gains to be saved to the class
 */
void WalkParameters::setGains(vector<vector<float> >& gains, unsigned int& numgains, const vector<vector<float> >& newgains)
{
    if (newgains.size() == 0)
    {
        gains = newgains;
        numgains = 0;
    }
    else
    {
        vector<vector<float> > tempgains;
        // we need to make sure that the shape of the gains is rectangular (ie each row has the same length)
        // so I pick the shortest row and use that length for all rows
        unsigned int width = newgains[0].size();
        for (unsigned int i=0; i<newgains.size(); i++)
        {
            if (newgains[i].size() < width)
                width = newgains[i].size();
        }
        
        // now copy and clip the gains to tempgains
        for (unsigned int i=0; i<newgains.size(); i++)
        {
            tempgains.push_back(vector<float>());
            for (unsigned int j=0; j<width; j++)
            {
                if (newgains[i][j] < 0)
                    tempgains.back().push_back(0);
                else if (newgains[i][j] > 100)
                    tempgains.back().push_back(100);
                else
                    tempgains.back().push_back(newgains[i][j]);
            }
        }
        gains = tempgains;
        numgains = gains.size()*width;
    }
}

/*! @brief Prints a human readable summary of the walk parameters
    Only those relevant to optimisation are shown.
 */
void WalkParameters::summaryTo(ostream& output)
{
    output << m_name << " WalkParameters: ";
    vector<float> temp = getAsVector();
    for (int i=0; i<temp.size(); i++)
        output << temp[i] << " ";
    output << endl;
}

/*! @brief Prints a comma separated string of the parameters
 */
void WalkParameters::csvTo(ostream& output)
{
    vector<float> temp = getAsVector();
    for (int i=0; i<temp.size(); i++)
        output << temp[i] << ", ";
}

/*! @brief Saves the entire contents of the WalkParameters class in the stream
 */
ostream& operator<< (ostream& output, const WalkParameters& p_walkparameters)
{
    output << p_walkparameters.m_name << endl;
    output << "Max Speeds (x cm/s, y cm/s, yaw rad/s): " << MotionFileTools::fromVector(p_walkparameters.m_max_speeds) << endl;
    output << "Max Accelerations (x cm/s/s, y cm/s/s, yaw rad/s/s): " << MotionFileTools::fromVector(p_walkparameters.m_max_accelerations) << endl;
    
    for (unsigned int i=0; i<p_walkparameters.m_parameters.size(); i++)
        output << p_walkparameters.m_parameters[i] << endl;

    output << "ArmGains (%): " << MotionFileTools::fromMatrix(p_walkparameters.m_arm_gains) << endl;
    output << "TorsoGains (%): " << MotionFileTools::fromMatrix(p_walkparameters.m_torso_gains) << endl;
    output << "LegGains (%): " << MotionFileTools::fromMatrix(p_walkparameters.m_leg_gains) << endl;

    return output;
}

/*! @brief Saves the entire contents of the WalkParameters class in the stream
 */
ostream& operator<< (ostream& output, const WalkParameters* p_walkparameters)
{
    output << (*p_walkparameters);
    return output;
}

/*! @brief Loads the entire contents of the WalkParameters class from the stream
 */
istream& operator>> (istream& input, WalkParameters& p_walkparameters)
{
    getline(input, p_walkparameters.m_name);
    p_walkparameters.m_max_speeds = MotionFileTools::toFloatVector(input);
    p_walkparameters.m_max_accelerations = MotionFileTools::toFloatVector(input);
    
    input.ignore(10,'\n');
    
    Parameter p;
    p_walkparameters.m_parameters.clear();
    
    int beforepeek = input.tellg();
    string label;
    input >> label;
    while (label.find("ArmGains") == string::npos && label.find("TorsoGains") == string::npos && label.find("LegGains") == string::npos)
    {
        input.seekg(beforepeek);
        input >> p;
        p_walkparameters.m_parameters.push_back(p);
        beforepeek = input.tellg();
        if (beforepeek < 0)
            return input;
        input >> label;
    }
    
    while (!input.eof())
    {
        if (label.find("ArmGains") != string::npos)
        {
            p_walkparameters.m_arm_gains = MotionFileTools::toFloatMatrix(input);
            p_walkparameters.m_num_arm_gains = MotionFileTools::size(p_walkparameters.m_arm_gains);
        }
        else if (label.find("TorsoGains") != string::npos)
        {
            p_walkparameters.m_torso_gains = MotionFileTools::toFloatMatrix(input);
            p_walkparameters.m_num_torso_gains = MotionFileTools::size(p_walkparameters.m_torso_gains);
        }
        else if (label.find("LegGains") != string::npos)
        {
            p_walkparameters.m_leg_gains = MotionFileTools::toFloatMatrix(input);
            p_walkparameters.m_num_leg_gains = MotionFileTools::size(p_walkparameters.m_leg_gains);
        }
        input >> label;
    }
        
    return input;
}

/*! @brief Loads the entire contents of the WalkParameters class from the stream
 */
istream& operator>> (istream& input, WalkParameters* p_walkparameters)
{
    input >> (*p_walkparameters);
    return input;
}

/*! @brief Saves the walk parameters to a file
 */
void WalkParameters::save()
{
    saveAs(m_name);
}

/*! @brief Saves the walk parameters to a new file given by the name
    @param name the new file will be name.cfg
 */
void WalkParameters::saveAs(const string& name)
{
    ofstream file((CONFIG_DIR + string("Motion/Walks/") + name + ".cfg").c_str());
    if (file.is_open())
    {
        file << this;
        file.close();
    }
    else
        debug << "WalkParameters::save(): Failed to open file " << name + ".cfg" << endl;
    file.close();
}

/*! @brief Loads the walk parameters from a file
    @param name the name of the file to load from
 */
void WalkParameters::load(const string& name)
{
    string filepath = CONFIG_DIR + string("Motion/Walks/") + name + ".cfg";
    ifstream file(filepath.c_str());
    if (file.is_open())
    {
        file >> this;
        file.close();
    }
    else
        debug << "WalkParameters::load(): Failed to load file " << filepath << endl;
    file.close();
}

/*! @brief Returns the size of the WalkParameters, this is the number of interesting walk parameters
 */
size_t WalkParameters::size() const
{
    return m_max_speeds.size() + m_max_accelerations.size() + m_parameters.size() + m_num_leg_gains;
}
