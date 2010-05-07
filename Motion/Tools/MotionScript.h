/*! @file MotionScript.h
    @brief Declaration of class to hold a motion script
 
    @class MotionScript
    @brief A class to hold a motion script
 
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

#ifndef MOTIONSCRIPT_H
#define MOTIONSCRIPT_H

class NUActionatorsData;
class NUSensorsData;

#include <string>
#include <vector>
using namespace std;

class MotionScript
{
public:
    MotionScript();
    MotionScript(std::string filename);
    ~MotionScript();
    
    void play(NUSensorsData* data, NUActionatorsData* actions);
protected:
    bool load();
private:
    void calculateCurve();
protected:
    std::string m_name;                 //!< the name of the script
    bool m_is_valid;                    //!< true if the motion script file was loaded without error

    float m_playspeed;
    
    // original script data
    vector<string> m_labels;             //!< the labels for each row
    float m_smoothness;                  //!< the smoothness loaded from the script file
    bool m_return_to_start;              //!< a flag to specify whether the script should return to the position when the script started playing
    vector<vector<double> > m_times;     //!< the times read in from the script file
    vector<vector<float> > m_positions;  //!< the positions read in from the script file
    vector<vector<float> > m_gains;      //!< the gains read in from the script file
    
    // smoothed script data (these curves are only valid for a playspeed of 1)
    vector<vector<double> > m_curvetimes;
    vector<vector<float> > m_curvepositions;
    vector<vector<float> > m_curvevelocities;
    vector<vector<float> > m_curvegains;
};

#endif

