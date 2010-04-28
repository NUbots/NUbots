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

#include <string>
#include <vector>
using namespace std;

class MotionScript
{
public:
    MotionScript(std::string filename);
    ~MotionScript();
protected:
    bool load();
private:
protected:
    std::string m_name;                 //!< the name of the script
    bool m_is_valid;                    //!< true if the motion script file was loaded without error
    
    // original script data
    vector<vector<double> > times;      //!< the times read in from the script file
    vector<vector<float> > positions;   //!< the positions read in from the script file
    vector<vector<float> > gains;       //!< the gains read in from the script file
    
    // smoothed script data
    float m_motion_smoothness;
    vector<vector<double> > curvetimes;
    vector<vector<float> > curvepositions;
    vector<vector<float> > curvevelocities;
};

#endif

