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

class MotionFileTools
{
public:
    static bool toBool(std::istream& input);
    
    static float toFloat(const char* data);
    static float toFloat(const std::string& data);
    static float toFloat(std::istream& input);
    
    static std::string fromVector(std::vector<float> data);
    static std::string fromVector(std::vector<double> data);
    static std::vector<float> toFloatVector(const std::string& data);
    static std::vector<float> toFloatVector(std::istream& input);
    
    static std::string fromVector(std::vector<std::string> data);
    static std::vector<std::string> toStringVector(std::istream& input);
    
    static void toFloatWithRange(const std::string& data, float& value, std::vector<float>& range);
    static void toFloatWithRange(std::istream& input, float& value, std::vector<float>& range);
    
    static std::string fromMatrix(const std::vector<std::vector<float> >& data);
    static std::string fromMatrix(const std::vector<std::vector<double> >& data);
    static std::vector<std::vector<float> > toFloatMatrix(const std::string& data);
    static std::vector<std::vector<float> > toFloatMatrix(std::istream& input);
    static size_t size(std::vector<std::vector<float> > data);
    static size_t size(std::vector<std::vector<double> > data);
    
    static void toFloatWithMatrix(std::istream& input, float& value, std::vector<std::vector<float> >& matrix);
    
private:
    MotionFileTools() {};
    ~MotionFileTools() {};
protected:
public:
};

#endif

