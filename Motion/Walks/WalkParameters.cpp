/*! @file WalkOptimiserBehaviour.cpp
    @brief Implementation of WalkOptimiserBehaviour class

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

#include "WalkParameters.h"

#include "debug.h"

/*! @brief Construct a walk parameter set with no parameters
 */
WalkParameters::WalkParameters()
{
    m_num_max_speeds = 0;                      
    m_num_parameters = 0;                      
    m_num_arm_gains = 0;                       
    m_num_torso_gains = 0;                     
    m_num_leg_gains = 0;
}

/*! @brief Construct a walk parameter set with the given parameters
    @param armgains the gains: [first phase, second phase, third phase, etc] where each phase [shoulder roll, shouler pitch, shouler yaw, elbow roll, elbow pitch]
    @param torsogains the gains: [first phase, second phase, third phase, etc] where each phase [torso roll, torso pitch, torso yaw]
    @param leggains the gains: [first phase, second phase, third phase, etc] where each phase [hip roll, hip pitch, hip yaw, knee, ankle roll, ankle pitch]
    @param parameters the walk engine parameters are stored as [first phase, second phase, etc] where each phase is a list of WalkParameter structs. However, only the walk engine knows the order they are saved in.
 */
WalkParameters::WalkParameters(const vector<vector<float> >& armgains, const vector<vector<float> >& torsogains, const vector<vector<float> >& leggains, const vector<vector<Parameter> >& parameters, const vector<float>& maxspeeds, const vector<float>& maxaccels)
{
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

/*! @brief Gets the arm gains stored in this WalkParameter class
    @param armgains the gains: [first phase, second phase, third phase, etc] where each phase [shoulder roll, shouler pitch, shouler yaw, elbow roll, elbow pitch]
 */
void WalkParameters::getArmGains(vector<vector<float> >& armgains)
{
    armgains = m_arm_gains;
}

/*! @brief Gets the torso gains stored in this WalkParameter class
    @param torsogains the gains: [first phase, second phase, third phase, etc] where each phase [torso roll, torso pitch, torso yaw]
 */
void WalkParameters::getTorsoGains(vector<vector<float> >& torsogains)
{
    torsogains = m_torso_gains;
}

/*! @brief Gets the leg gains stored in this WalkParameter class
    @param leggains the gains: [first phase, second phase, third phase, etc] where each phase [hip roll, hip pitch, hip yaw, knee, ankle roll, ankle pitch]
 */
void WalkParameters::getLegGains(vector<vector<float> >& leggains)
{
    leggains = m_leg_gains;
}

/*! @brief Gets the walk engine parameters stored here
    @param parameters the walk engine parameters are stored as [first phase, second phase, etc] where each phase is a list of WalkParameter structs. However, only the walk engine knows the order they are saved in.
 */
void WalkParameters::getParameters(vector<vector<Parameter> >& parameters)
{
    parameters = m_parameters;
}

/*! @brief Gets the walk engine max speed values stored here
    @param maxspeeds the walk engine max speeds stored as [x (cm/s), y (cm/s), theta (rad/s)].
 */
void WalkParameters::getMaxSpeeds(vector<float>& maxspeeds)
{
    maxspeeds = m_max_speeds;
}

/*! @brief Gets the walk engine max acceleration values stored here
    @param maxaccels the walk engine max accelerations stored as [x (cm/s/s), y (cm/s/s), theta (rad/s/s)].
 */
void WalkParameters::getMaxAccelerations(vector<float>& maxaccels)
{
    maxaccels = m_max_accelerations;
}

/*! @brief Sets the arm gains
    @param armgains the gains: [first phase, second phase, third phase, etc] where each phase [shoulder roll, shouler pitch, shouler yaw, elbow roll, elbow pitch]
 */
void WalkParameters::setArmGains(const vector<vector<float> >& armgains)
{
    m_arm_gains = armgains;
    if (armgains.size() == 0)
        m_num_arm_gains = 0;
    else
        m_num_arm_gains = armgains.size()*armgains[0].size();
}

/*! @brief Sets the torso gains stored in this WalkParameter class
    @param torsogains the gains: [first phase, second phase, third phase, etc] where each phase [torso roll, torso pitch, torso yaw]
 */
void WalkParameters::setTorsoGains(const vector<vector<float> >& torsogains)
{
    m_torso_gains = torsogains;
    if (m_torso_gains.size() == 0)
        m_num_torso_gains = 0;
    else
        m_num_torso_gains = m_torso_gains.size()*m_torso_gains[0].size();
}

/*! @brief Sets the leg gains stored in this WalkParameter class
    @param leggains the gains: [first phase, second phase, third phase, etc] where each phase [hip roll, hip pitch, hip yaw, knee, ankle roll, ankle pitch]
 */
void WalkParameters::setLegGains(const vector<vector<float> >& leggains)
{
    m_leg_gains = leggains;
    if (m_leg_gains.size() == 0)
        m_num_leg_gains = 0;
    else
        m_num_leg_gains = m_leg_gains.size()*m_leg_gains[0].size();
}

/*! @brief Sets the walk engine parameters stored here
    @param parameters the walk engine parameters are stored as [first phase, second phase, etc] where each phase is a list of WalkParameter structs. However, only the walk engine knows the order they are saved in.
 */
void WalkParameters::setParameters(const vector<vector<Parameter> >& parameters)
{
    m_parameters = parameters;
    if (m_parameters.size() == 0)
        m_num_parameters = 0;
    else
        m_num_parameters = m_parameters.size()*m_parameters[0].size();
}

/*! @brief Sets the walk engine max speeds stored here
    @param maxspeeds the walk engine max speeds stored as [x (cm/s), y (cm/s), theta (rad/s)].
 */
void WalkParameters::setMaxSpeeds(const vector<float>& maxspeeds)
{
    m_max_speeds = maxspeeds;
    m_num_max_speeds = m_max_speeds.size();
}

/*! @brief Sets the walk engine max accelerations stored here
    @param maxspeeds the walk engine max accelerations stored as [x (cm/s/s), y (cm/s/s), theta (rad/s/s)].
 */
void WalkParameters::setMaxAccelerations(const vector<float>& maxaccels)
{
    m_max_accelerations = maxaccels;
    m_num_max_accelerations = m_max_accelerations.size();
}

/*! @brief Prints a human readable summary of the walk parameters
    Only those relevant to optimisation are shown.
 */
void WalkParameters::summaryTo(ostream& output)
{
    output << "WalkParameters: ";
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
    // m_num_max_speeds
    output.write((char*) &p_walkparameters.m_num_max_speeds, sizeof(int));
    // m_max_speeds
    for (int i=0; i<p_walkparameters.m_num_max_speeds; i++)
        output.write((char*) &p_walkparameters.m_max_speeds[i], sizeof(float));
    // m_num_max_accelerations
    output.write((char*) &p_walkparameters.m_num_max_accelerations, sizeof(int));
    // m_max_accelerations
    for (int i=0; i<p_walkparameters.m_num_max_accelerations; i++)
        output.write((char*) &p_walkparameters.m_max_accelerations[i], sizeof(float));
    // m_num_parameters, numperphase
    output.write((char*) &p_walkparameters.m_num_parameters, sizeof(int));
    if (p_walkparameters.m_num_parameters > 0)
    {
        int numperphase = p_walkparameters.m_parameters[0].size();
        output.write((char*) &numperphase, sizeof(int));
        // m_parameters
        for (int i=0; i<p_walkparameters.m_num_parameters/numperphase; i++)
            for (int j=0; j<numperphase; j++)
                output << p_walkparameters.m_parameters[i][j];
    }
    // m_num_arm_gains, numperphase
    output.write((char*) &p_walkparameters.m_num_arm_gains, sizeof(int));
    if (p_walkparameters.m_num_arm_gains > 0)
    {
        int numperphase = p_walkparameters.m_arm_gains[0].size();
        output.write((char*) &numperphase, sizeof(int));
        // m_arm_gains
        for (int i=0; i<p_walkparameters.m_num_arm_gains/numperphase; i++)
            for (int j=0; j<numperphase; j++)
                output.write((char*) &p_walkparameters.m_arm_gains[i][j], sizeof(float));
    }
    // m_num_torso_gains, numperphase
    output.write((char*) &p_walkparameters.m_num_torso_gains, sizeof(int));
    if (p_walkparameters.m_num_torso_gains > 0)
    {
        int numperphase = p_walkparameters.m_torso_gains[0].size();
        output.write((char*) &numperphase, sizeof(int));
        // m_torso_gains
        for (int i=0; i<p_walkparameters.m_num_torso_gains/numperphase; i++)
            for (int j=0; j<numperphase; j++)
                output.write((char*) &p_walkparameters.m_torso_gains[i][j], sizeof(float));
    }
    // m_num_leg_gains, numperphase
    output.write((char*) &p_walkparameters.m_num_leg_gains, sizeof(int));
    if (p_walkparameters.m_num_leg_gains > 0)
    {
        int numperphase = p_walkparameters.m_leg_gains[0].size();
        output.write((char*) &numperphase, sizeof(int));
        // m_leg_gains
        for (int i=0; i<p_walkparameters.m_num_leg_gains/numperphase; i++)
            for (int j=0; j<numperphase; j++)
                output.write((char*) &p_walkparameters.m_leg_gains[i][j], sizeof(float));
    }
    return output;
}

/*! @brief Loads the entire contents of the WalkParameters class from the stream
 */
istream& operator>> (istream& input, WalkParameters& p_walkparameters)
{
    char inbuffer[100];
    // m_num_max_speeds
    input.read(inbuffer, sizeof(int));
    p_walkparameters.m_num_max_speeds = *((int*) inbuffer);
    // m_max_speeds
    p_walkparameters.m_max_speeds.resize(p_walkparameters.m_num_max_speeds, 0);
    for (int i=0; i<p_walkparameters.m_num_max_speeds; i++)
    {
        input.read(inbuffer, sizeof(float));
        p_walkparameters.m_max_speeds[i] = *((float*) inbuffer);
    }
    // m_num_max_accelerations
    input.read(inbuffer, sizeof(int));
    p_walkparameters.m_num_max_accelerations = *((int*) inbuffer);
    // m_max_accelerations
    p_walkparameters.m_max_accelerations.resize(p_walkparameters.m_num_max_accelerations, 0);
    for (int i=0; i<p_walkparameters.m_num_max_accelerations; i++)
    {
        input.read(inbuffer, sizeof(float));
        p_walkparameters.m_max_accelerations[i] = *((float*) inbuffer);
    }
    
    // m_num_parameters, numperphase
    input.read(inbuffer, sizeof(int));
    p_walkparameters.m_num_parameters = *((int*) inbuffer);
    if (p_walkparameters.m_num_parameters > 0)
    {
        input.read(inbuffer, sizeof(int));
        int numperphase = *((int*) inbuffer);
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
    input.read(inbuffer, sizeof(int));
    p_walkparameters.m_num_arm_gains = *((int*) inbuffer);
    if (p_walkparameters.m_num_arm_gains > 0)
    {
        input.read(inbuffer, sizeof(int));
        int numperphase = *((int*) inbuffer);
        p_walkparameters.m_arm_gains.resize(p_walkparameters.m_num_arm_gains/numperphase);
        // m_arm_gains
        for (int i=0; i<p_walkparameters.m_num_arm_gains/numperphase; i++)
        {
            p_walkparameters.m_arm_gains[i].resize(numperphase);
            for (int j=0; j<numperphase; j++)
            {
                input.read(inbuffer, sizeof(float));
                p_walkparameters.m_arm_gains[i][j] = *((float*) inbuffer);
            }
        }
    }
    else
        p_walkparameters.m_arm_gains.clear();
    
    // m_num_torso_gains, numperphase
    input.read(inbuffer, sizeof(int));
    p_walkparameters.m_num_torso_gains = *((int*) inbuffer);
    if (p_walkparameters.m_num_torso_gains > 0)
    {
        input.read(inbuffer, sizeof(int));
        int numperphase = *((int*) inbuffer);
        p_walkparameters.m_torso_gains.resize(p_walkparameters.m_num_torso_gains/numperphase);
        // m_torso_gains
        for (int i=0; i<p_walkparameters.m_num_torso_gains/numperphase; i++)
        {
            p_walkparameters.m_torso_gains[i].resize(numperphase);
            for (int j=0; j<numperphase; j++)
            {
                input.read(inbuffer, sizeof(float));
                p_walkparameters.m_torso_gains[i][j] = *((float*) inbuffer);
            }
        }
    }
    else
        p_walkparameters.m_torso_gains.clear();
    
    // m_num_leg_gains, numperphase
    input.read(inbuffer, sizeof(int));
    p_walkparameters.m_num_leg_gains = *((int*) inbuffer);
    if (p_walkparameters.m_num_leg_gains > 0)
    {
        input.read(inbuffer, sizeof(int));
        int numperphase = *((int*) inbuffer);
        p_walkparameters.m_leg_gains.resize(p_walkparameters.m_num_leg_gains/numperphase);
        // m_leg_gains
        for (int i=0; i<p_walkparameters.m_num_leg_gains/numperphase; i++)
        {
            p_walkparameters.m_leg_gains[i].resize(numperphase);
            for (int j=0; j<numperphase; j++)
            {
                input.read(inbuffer, sizeof(float));
                p_walkparameters.m_leg_gains[i][j] = *((float*) inbuffer);
            }
        }
    }
    else
        p_walkparameters.m_leg_gains.clear();
    
    return input;
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
    if (m_num_max_speeds == 0)
        nummaxspeedsused = 0;
    else
        nummaxspeedsused = 1;
    if (m_num_max_accelerations == 0)
        nummaxaccelsused = 0;
    else
        nummaxaccelsused = 1;
    
    if (index < nummaxspeedsused)
        return m_max_speeds[index];
    else if (index < nummaxspeedsused + nummaxaccelsused)
        return m_max_accelerations[index - nummaxspeedsused];
    else if (index < m_num_parameters + nummaxspeedsused + nummaxaccelsused)
        return m_parameters[(index - nummaxspeedsused - nummaxaccelsused)/m_parameters[0].size()][(index - nummaxspeedsused - nummaxaccelsused)%m_parameters[0].size()].Value;
    else if (index < m_num_parameters + nummaxspeedsused + nummaxaccelsused + m_num_leg_gains)
        return m_leg_gains[(index - m_num_parameters - nummaxspeedsused - nummaxaccelsused)/m_leg_gains[0].size()][(index - m_num_parameters - nummaxspeedsused - nummaxaccelsused)%m_leg_gains[0].size()];
    else
        return m_max_speeds[0];
}

/*! @brief Returns the size of the WalkParameters, that is the number of elements stored here that are relevant to an optimiser
 */
int WalkParameters::size() const
{
    static int nummaxspeedsused = 1;
    static int nummaxaccelsused = 1;
    if (m_num_max_speeds == 0)
        nummaxspeedsused = 0;
    else
        nummaxspeedsused = 1;
    if (m_num_max_accelerations == 0)
        nummaxaccelsused = 0;
    else
        nummaxaccelsused = 1;

    return m_num_parameters + m_num_leg_gains + nummaxspeedsused + nummaxaccelsused;
}
