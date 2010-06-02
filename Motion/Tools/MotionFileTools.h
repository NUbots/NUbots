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

#ifndef MOTIONFILETOOLS_H
#define MOTIONFILETOOLS_H

#include <vector>
#include <string>
using namespace std;

class MotionFileTools
{
public:
    static bool toBool(istream& input);
    
    static float toFloat(const char* data);
    static float toFloat(const string& data);
    static float toFloat(istream& input);
    
    static string fromVector(vector<float> data);
    static string fromVector(vector<double> data);
    static vector<float> toFloatVector(const string& data);
    static vector<float> toFloatVector(istream& input);
    
    static string fromVector(vector<string> data);
    static vector<string> toStringVector(istream& input);
    
    static void toFloatWithRange(const string& data, float& value, vector<float>& range);
    static void toFloatWithRange(istream& input, float& value, vector<float>& range);
    
    static string fromMatrix(const vector<vector<float> >& data);
    static string fromMatrix(const vector<vector<double> >& data);
    static vector<vector<float> > toFloatMatrix(const string& data);
    static vector<vector<float> > toFloatMatrix(istream& input);
    static size_t size(vector<vector<float> > data);
    static size_t size(vector<vector<double> > data);
    
    static void toFloatWithMatrix(istream& input, float& value, vector<vector<float> >& matrix);
    
private:
    MotionFileTools() {};
    ~MotionFileTools() {};
protected:
public:
};

#endif

