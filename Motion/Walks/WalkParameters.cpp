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

#include "Tools/debug.h"

/*! @brief Construct a walk parameter set with no parameters
 */
WalkParameters::WalkParameters()
{
}

/*! @brief Construct a walk parameter set with the given parameters
    @param armgains the gains: [first phase, second phase, third phase, etc] where each phase [shoulder roll, shouler pitch, shouler yaw, elbow roll, elbow pitch]
    @param torsogains the gains: [first phase, second phase, third phase, etc] where each phase [torso roll, torso pitch, torso yaw]
    @param leggains the gains: [first phase, second phase, third phase, etc] where each phase [hip roll, hip pitch, hip yaw, knee, ankle roll, ankle pitch]
    @param parameters the walk engine parameters are stored as [first phase, second phase, etc] where each phase is a list of WalkParameter structs. However, only the walk engine knows the order they are saved in.
 */
WalkParameters::WalkParameters(const vector<vector<float> >& armgains, const vector<vector<float> >& torsogains, const vector<vector<float> >& leggains, const vector<vector<Parameter> >& parameters)
{
    setParameters(parameters);
    setArmGains(armgains);
    setTorsoGains(torsogains);
    setLegGains(leggains);
}

/*! @brief Destroy a walk parameter set
 */
WalkParameters::~WalkParameters()
{
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

void WalkParameters::summaryTo(ostream& output)
{
    output << "WalkParameters: ";
    for (int i=0; i<size(); i++)
        output << (*this)[i] << " ";
    output << endl;
}

void WalkParameters::csvTo(ostream& output)
{
}

ostream& operator<< (ostream& output, const WalkParameters& p_walkparameters)
{
}

istream& operator>> (istream& input, WalkParameters& p_walkparameters)
{
}

/*! @brief Overloading subscript operator has been designed to be used by a walk optimiser.
           In that only parameters relevant to an optimiser are returned.
 */
float& WalkParameters::operator[] (const int index)
{
    // I have written this function with the assumption that I don't want to optimise the arm or torso gains
    if (index < m_num_parameters)
        return m_parameters[index/m_parameters[0].size()][index%m_parameters[0].size()].Value;
    else if (index < m_num_parameters + m_num_leg_gains)
        return m_leg_gains[(index - m_num_parameters)/m_leg_gains[0].size()][(index - m_num_parameters)%m_leg_gains[0].size()];
}

/*! @brief Returns the size of the WalkParameters, that is the number of elements stored here that are relevant to an optimiser
 */
int WalkParameters::size() const
{
    return m_num_parameters + m_num_leg_gains;
}
