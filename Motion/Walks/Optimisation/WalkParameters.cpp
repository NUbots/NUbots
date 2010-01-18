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
    @param armstiffnesses the stiffnesses: [first phase, second phase, third phase, etc] where each phase [shoulder roll, shouler pitch, shouler yaw, elbow roll, elbow pitch]
    @param torsostiffnesses the stiffnesses: [first phase, second phase, third phase, etc] where each phase [torso roll, torso pitch, torso yaw]
    @param legstiffnesses the stiffnesses: [first phase, second phase, third phase, etc] where each phase [hip roll, hip pitch, hip yaw, knee, ankle roll, ankle pitch]
    @param parameters the walk engine parameters are stored as [first phase, second phase, etc] where each phase is a list of WalkParameter structs. However, only the walk engine knows the order they are saved in.
 */
WalkParameters::WalkParameters(const vector<vector<float> >& armstiffnesses, const vector<vector<float> >& torsostiffnesses, const vector<vector<float> >& legstiffnesses, const vector<vector<WalkParameter> >& parameters)
{
    setArmStiffnesses(armstiffnesses);
    setTorsoStiffnesses(torsostiffnesses);
    setLegStiffnesses(legstiffnesses);
    setParameters(parameters);
}

/*! @brief Destroy a walk parameter set
 */
WalkParameters::~WalkParameters()
{
}

/*! @brief Gets the arm stiffnesses stored in this WalkParameter class
    @param armstiffnesses the stiffnesses: [first phase, second phase, third phase, etc] where each phase [shoulder roll, shouler pitch, shouler yaw, elbow roll, elbow pitch]
 */
void getArmStiffnesses(vector<vector<float> >& armstiffnesses)
{
    armstiffnesses = m_arm_stiffnesses;
}

/*! @brief Gets the torso stiffnesses stored in this WalkParameter class
    @param torsostiffnesses the stiffnesses: [first phase, second phase, third phase, etc] where each phase [torso roll, torso pitch, torso yaw]
 */
void getTorsoStiffnesses(vector<vector<float> >& torsostiffnesses)
{
    torsostiffnesses = m_torso_stiffnesses;
}

/*! @brief Gets the leg stiffnesses stored in this WalkParameter class
    @param legstiffnesses the stiffnesses: [first phase, second phase, third phase, etc] where each phase [hip roll, hip pitch, hip yaw, knee, ankle roll, ankle pitch]
 */
void getLegStiffnesses(vector<vector<float> >& legstiffnesses)
{
    legstiffnesses = m_leg_stiffnesses;
}

/*! @brief Gets the walk engine parameters stored here
    @param parameters the walk engine parameters are stored as [first phase, second phase, etc] where each phase is a list of WalkParameter structs. However, only the walk engine knows the order they are saved in.
 */
void getParameters(vector<vector<WalkParameter> >& parameters)
{
    parameters = m_parameters;
}

/*! @brief Sets the arm stiffnesses
    @param armstiffnesses the stiffnesses: [first phase, second phase, third phase, etc] where each phase [shoulder roll, shouler pitch, shouler yaw, elbow roll, elbow pitch]
 */
void setArmStiffnesses(const vector<vector<float> >& armstiffnesses)
{
    m_arm_stiffnesses = armstiffnesses;
}

/*! @brief Sets the torso stiffnesses stored in this WalkParameter class
    @param torsostiffnesses the stiffnesses: [first phase, second phase, third phase, etc] where each phase [torso roll, torso pitch, torso yaw]
 */
void setTorsoStiffnesses(const vector<vector<float> >& torsostiffnesses)
{
    m_torso_stiffnesses = torsostiffnesses;
}

/*! @brief Sets the leg stiffnesses stored in this WalkParameter class
    @param legstiffnesses the stiffnesses: [first phase, second phase, third phase, etc] where each phase [hip roll, hip pitch, hip yaw, knee, ankle roll, ankle pitch]
 */
void setLegStiffnesses(const vector<vector<float> >& legstiffnesses)
{
    m_leg_stiffnesses = legstiffnesses;
}

/*! @brief Sets the walk engine parameters stored here
    @param parameters the walk engine parameters are stored as [first phase, second phase, etc] where each phase is a list of WalkParameter structs. However, only the walk engine knows the order they are saved in.
 */
void setParameters(const vector<vector<WalkParameter> >& parameters)
{
    m_parameters = parameters;
}
