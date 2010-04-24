/*! @file MotionFileTools.h
    @brief Declaration of functions to deal with motion files
 
    @class MotionFileTools
    @brief A module to load and save information to from files for the motion module
 
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

#ifndef FILETOOLS_H
#define FILETOOLS_H

#include <vector>
#include <string>
using namespace std;

class MotionFileTools
{
public:
    static vector<float> toFloatVector(char* data);
    static vector<float> toFloatVector(string& data);
    static vector<float> toFloatVector(istream& input);
    
private:
    MotionFileTools() {};
    ~MotionFileTools() {};
protected:
public:
};

#endif

