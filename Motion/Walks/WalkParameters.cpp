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
    m_max_speeds.clear();
    m_max_accelerations.clear();
    m_parameters.clear();
    m_arm_gains.clear();
    m_torso_gains.clear();
    m_leg_gains.clear();
}

/*! @brief Returns the walk parameter set's name
    @return the walk parameter name
 */
string& WalkParameters::getName()
{
    return m_name;
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

/*! @brief Gets the walk engine parameters stored here
    @return the walk engine parameters stored in this object
 */
vector<WalkParameters::Parameter>& WalkParameters::getParameters()
{
    return m_parameters;
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
    cout << "setGains" << endl;
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
        
        cout << gains[0][0] << endl;
    }
}

/*! @brief Prints a human readable summary of the walk parameters
    Only those relevant to optimisation are shown.
 */
void WalkParameters::summaryTo(ostream& output)
{
    output << m_name << " WalkParameters: ";
    for (int i=0; i<size(); i++)
        output << (*this)[i] << " ";
    output << endl;
}

/*! @brief Prints a comma separated string of the parameters
 */
void WalkParameters::csvTo(ostream& output)
{
    for (int i=0; i<size(); i++)
        output << (*this)[i] << ", ";
}

/*! @brief Attempts to load a set of parameters from a file created using csvTo
 */
void WalkParameters::csvFrom(istream& input)
{
    // this isn't the most robust get from csv in the world but it will do for tonight.
    // (it assumes that the walk_parameters has already been initialised, and will
    // only overwrite the usual parameters, the rest are left alone.
    for (int i=0; i<size(); i++)
    {
        input >> (*this)[i];
        input.ignore(2, ',');
    }
    
}

/*! @brief Saves the entire contents of the WalkParameters class in the stream
 */
ostream& operator<< (ostream& output, const WalkParameters& p_walkparameters)
{
    output << p_walkparameters.m_name << endl;
    output << "Max Speeds (x cm/s, y cm/s, yaw rad/s): [";
    for (unsigned int i=0; i<p_walkparameters.m_max_speeds.size()-1; i++)
        output << p_walkparameters.m_max_speeds[i] << ", ";
    output << p_walkparameters.m_max_speeds[p_walkparameters.m_max_speeds.size()-1] << "]" << endl;
    
    output << "Max Accelerations (x cm/s/s, y cm/s/s, yaw rad/s/s): [";
    for (unsigned int i=0; i<p_walkparameters.m_max_accelerations.size()-1; i++)
        output << p_walkparameters.m_max_accelerations[i] << ", ";
    output << p_walkparameters.m_max_accelerations[p_walkparameters.m_max_accelerations.size()-1] << "]" << endl;
    
    for (unsigned int i=0; i<p_walkparameters.m_parameters.size(); i++)
        output << p_walkparameters.m_parameters[i] << endl;
    
    output << "ArmGains (%): [";
    for (unsigned int i=0; i<p_walkparameters.m_arm_gains.size(); i++)
    {
        output << "[";
        for (unsigned int j=0; j<p_walkparameters.m_arm_gains[i].size()-1; j++)
            output << p_walkparameters.m_arm_gains[i][j] << ", ";
        output << p_walkparameters.m_arm_gains[i][p_walkparameters.m_arm_gains[i].size()-1] << "]";
    }
    output << "]" << endl;

    output << "TorsoGains (%): [";
    for (unsigned int i=0; i<p_walkparameters.m_torso_gains.size(); i++)
    {
        output << "[";
        for (unsigned int j=0; j<p_walkparameters.m_torso_gains[i].size()-1; j++)
            output << p_walkparameters.m_torso_gains[i][j] << ", ";
        output << p_walkparameters.m_torso_gains[i][p_walkparameters.m_torso_gains[i].size()-1] << "]";
    }
    output << "]" << endl;
    
    output << "LegGains (%): [";
    for (unsigned int i=0; i<p_walkparameters.m_leg_gains.size(); i++)
    {
        output << "[";
        for (unsigned int j=0; j<p_walkparameters.m_leg_gains[i].size()-1; j++)
            output << p_walkparameters.m_leg_gains[i][j] << ", ";
        output << p_walkparameters.m_leg_gains[i][p_walkparameters.m_leg_gains[i].size()-1] << "]";
    }
    output << "]" << endl;

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
    /*
    int intBuffer;
    float floatBuffer;
    // m_num_max_speeds
    input.read(reinterpret_cast<char*>(&intBuffer), sizeof(int));
    p_walkparameters.m_num_max_speeds = intBuffer;
    // m_max_speeds
    p_walkparameters.m_max_speeds.resize(p_walkparameters.m_num_max_speeds, 0);
    for (int i=0; i<p_walkparameters.m_num_max_speeds; i++)
    {
        input.read(reinterpret_cast<char*>(&floatBuffer), sizeof(float));
        p_walkparameters.m_max_speeds[i] = floatBuffer;
    }
    // m_num_max_accelerations
    input.read(reinterpret_cast<char*>(&intBuffer), sizeof(int));
    p_walkparameters.m_num_max_accelerations = intBuffer;
    // m_max_accelerations
    p_walkparameters.m_max_accelerations.resize(p_walkparameters.m_num_max_accelerations, 0);
    for (int i=0; i<p_walkparameters.m_num_max_accelerations; i++)
    {
        input.read(reinterpret_cast<char*>(&floatBuffer), sizeof(float));
        p_walkparameters.m_max_accelerations[i] = floatBuffer;
    }
    
    // m_num_parameters, numperphase
    input.read(reinterpret_cast<char*>(&intBuffer), sizeof(int));
    p_walkparameters.m_num_parameters = intBuffer;
    if (p_walkparameters.m_num_parameters > 0)
    {
        input.read(reinterpret_cast<char*>(&intBuffer), sizeof(int));
        int numperphase = intBuffer;
        p_walkparameters.m_parameters.resize(p_walkparameters.m_num_parameters/numperphase);
        // m_parameters
        for (int i=0; i<p_walkparameters.m_num_parameters/numperphase; i++)
        {
            p_walkparameters.m_parameters[i].resize(numperphase);
            for (int j=0; j<numperphase; j++)
                input >> p_walkparameters.m_parameters[i][j];
        }
    }
    else
        p_walkparameters.m_parameters.clear();
    
    // m_num_arm_gains, numperphase
    input.read(reinterpret_cast<char*>(&intBuffer), sizeof(int));
    p_walkparameters.m_num_arm_gains = intBuffer;
    if (p_walkparameters.m_num_arm_gains > 0)
    {
        input.read(reinterpret_cast<char*>(&intBuffer), sizeof(int));
        int numperphase = intBuffer;
        p_walkparameters.m_arm_gains.resize(p_walkparameters.m_num_arm_gains/numperphase);
        // m_arm_gains
        for (int i=0; i<p_walkparameters.m_num_arm_gains/numperphase; i++)
        {
            p_walkparameters.m_arm_gains[i].resize(numperphase);
            for (int j=0; j<numperphase; j++)
            {
                input.read(reinterpret_cast<char*>(&floatBuffer), sizeof(float));
                p_walkparameters.m_arm_gains[i][j] = floatBuffer;
            }
        }
    }
    else
        p_walkparameters.m_arm_gains.clear();
    
    // m_num_torso_gains, numperphase
    input.read(reinterpret_cast<char*>(&intBuffer), sizeof(int));
    p_walkparameters.m_num_torso_gains = intBuffer;
    if (p_walkparameters.m_num_torso_gains > 0)
    {
        input.read(reinterpret_cast<char*>(&intBuffer), sizeof(int));
        int numperphase = intBuffer;
        p_walkparameters.m_torso_gains.resize(p_walkparameters.m_num_torso_gains/numperphase);
        // m_torso_gains
        for (int i=0; i<p_walkparameters.m_num_torso_gains/numperphase; i++)
        {
            p_walkparameters.m_torso_gains[i].resize(numperphase);
            for (int j=0; j<numperphase; j++)
            {
                input.read(reinterpret_cast<char*>(&floatBuffer), sizeof(float));
                p_walkparameters.m_torso_gains[i][j] = floatBuffer;
            }
        }
    }
    else
        p_walkparameters.m_torso_gains.clear();
    
    // m_num_leg_gains, numperphase
    input.read(reinterpret_cast<char*>(&intBuffer), sizeof(int));
    p_walkparameters.m_num_leg_gains = intBuffer;
    if (p_walkparameters.m_num_leg_gains > 0)
    {
        input.read(reinterpret_cast<char*>(&intBuffer), sizeof(int));
        int numperphase = intBuffer;
        p_walkparameters.m_leg_gains.resize(p_walkparameters.m_num_leg_gains/numperphase);
        // m_leg_gains
        for (int i=0; i<p_walkparameters.m_num_leg_gains/numperphase; i++)
        {
            p_walkparameters.m_leg_gains[i].resize(numperphase);
            for (int j=0; j<numperphase; j++)
            {
                input.read(reinterpret_cast<char*>(&floatBuffer), sizeof(float));
                p_walkparameters.m_leg_gains[i][j] = floatBuffer;
            }
        }
    }
    else
        p_walkparameters.m_leg_gains.clear();
    */
    return input;
}

/*! @brief Loads the entire contents of the WalkParameters class from the stream
 */
istream& operator>> (istream& input, WalkParameters* p_walkparameters)
{
    input >> (*p_walkparameters);
    return input;
}

void WalkParameters::save()
{
    ofstream file((CONFIG_DIR + string("Motion/Walks/") + m_name + ".cfg").c_str());
    file << this;
    file.close();
}

/*! @brief Overloading subscript operator has been designed to be used by a walk optimiser.
           In that only parameters relevant to an optimiser are returned.
 */
float& WalkParameters::operator[] (const int index)
{
    // I have written this function with the assumption that I don't want to optimise the arm or torso gains
    // Furthermore, that I only want to optimise the speed in the forward direction (for now)
    static int nummaxspeedsused = 1;
    static int nummaxaccelsused = 1;
    if (m_max_speeds.size() == 0)
        nummaxspeedsused = 0;
    else
        nummaxspeedsused = 1;
    if (m_max_accelerations.size() == 0)
        nummaxaccelsused = 0;
    else
        nummaxaccelsused = 1;
    
    int numwalkparameters = m_parameters.size();
    if (index < nummaxspeedsused)
        return m_max_speeds[index];
    else if (index < nummaxspeedsused + nummaxaccelsused)
        return m_max_accelerations[index - nummaxspeedsused];
    else if (index < numwalkparameters + nummaxspeedsused + nummaxaccelsused)
        return m_parameters[index - nummaxspeedsused - nummaxaccelsused].Value;
    else if (index < numwalkparameters + nummaxspeedsused + nummaxaccelsused + m_num_leg_gains)
        return m_leg_gains[(index - numwalkparameters - nummaxspeedsused - nummaxaccelsused)/m_leg_gains[0].size()][(index - numwalkparameters - nummaxspeedsused - nummaxaccelsused)%m_leg_gains[0].size()];
    else
        return m_max_speeds[0];
}

/*! @brief Returns the size of the WalkParameters, that is the number of elements stored here that are relevant to an optimiser
 */
int WalkParameters::size() const
{
    static int nummaxspeedsused = 1;
    static int nummaxaccelsused = 1;
    if (m_max_speeds.size() == 0)
        nummaxspeedsused = 0;
    else
        nummaxspeedsused = 1;
    if (m_max_accelerations.size() == 0)
        nummaxaccelsused = 0;
    else
        nummaxaccelsused = 1;

    return m_parameters.size() + m_num_leg_gains + nummaxspeedsused + nummaxaccelsused;
}
