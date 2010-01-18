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

#include <vector>
using namespace std;

class WalkParameters
{
public:
    class Parameter 
    {
    public:
        Parameter(float value, float min, float max) {Value = value; Min = min; Max = max;};
        ~Parameter() {};
        float Value;
        float Min;
        float Max;
    };
public:
    WalkParameters();
    WalkParameters(const vector<vector<float> >& armgains, const vector<vector<float> >& torsogains, const vector<vector<float> >& leggains, const vector<vector<Parameter> >& parameters);
    ~WalkParameters();
    
    // get methods
    void getArmGains(vector<vector<float> >& armgains);
    void getTorsoGains(vector<vector<float> >& torsogains);
    void getLegGains(vector<vector<float> >& leggains);
    void getParameters(vector<vector<Parameter> >& parameters);
    
    // set methods
    void setArmGains(const vector<vector<float> >& armgains);
    void setTorsoGains(const vector<vector<float> >& torsogains);
    void setLegGains(const vector<vector<float> >& leggains);
    void setParameters(const vector<vector<Parameter> >& parameters);
    
    void summaryTo(ostream& output);
    void csvTo(ostream& output);
    
    friend ostream& operator<< (ostream& output, const WalkParameters& p_walkparameters);
    friend istream& operator>> (istream& input, WalkParameters& p_walkparameters);
    
    float& operator[] (const int index);
    int size() const;
protected:
private:
public:
private:
    vector<vector<Parameter> > m_parameters;   //!< stores the parameters for the walk engine
    int m_num_parameters;                      //!< stores the total number of parameters for the walk engine
    vector<vector<float> > m_arm_gains;        //!< stores the arm gains for a walk
    int m_num_arm_gains;                       //!< stores the total number of arm gains for the walk
    vector<vector<float> > m_torso_gains;      //!< stores the torso gains for a walk
    int m_num_torso_gains;                     //!< stores the total number of torso gains for the walk
    vector<vector<float> > m_leg_gains;        //!< stores the leg gains for a walk
    int m_num_leg_gains;
};

#endif

