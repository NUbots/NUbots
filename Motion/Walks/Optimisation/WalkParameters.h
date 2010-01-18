/*! @file WalkParameters.h
    @brief Declaration of WalkParameters class
 
    @class WalkParameters
    @brief A module to store walk parameters
 
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

#include <vector>
using namespace std;

struct WalkParameter 
{
    float Value;
    float Min;
    float Max;
}

class WalkParameters
{
public:
    WalkParameters();
    WalkParameters(const vector<vector<float> >& armstiffnesses, const vector<vector<float> >& torsostiffnesses, const vector<vector<float> >& legstiffnesses, const vector<vector<WalkParameter> >& parameters);
    ~WalkParameters();
    
    // get methods
    void getArmStiffnesses(vector<vector<float> >& armstiffnesses);
    void getTorsoStiffnesses(vector<vector<float> >& torsostiffnesses);
    void getLegStiffnesses(vector<vector<float> >& legstiffnesses);
    void getParameters(vector<vector<WalkParameter> >& parameters);
    
    // set methods
    void setArmStiffnesses(const vector<vector<float> >& armstiffnesses);
    void setTorsoStiffnesses(const vector<vector<float> >& torsostiffnesses);
    void setLegStiffnesses(const vector<vector<float> >& legstiffnesses);
    void setParameters(const vector<vector<WalkParameter> >& parameters);
protected:
private:
public:
private:
    vector<vector<float> > m_arm_stiffnesses;        //!< stores the arm stiffnesses for a walk
    vector<vector<float> > m_torso_stiffnesses;      //!< stores the torso stiffnesses for a walk
    vector<vector<float> > m_leg_stiffnesses;        //!< stores the leg stiffnesses for a walk
    vector<vector<WalkParameter> > m_parameters;     //!< stores the parameters for the walk engine
};

#endif

