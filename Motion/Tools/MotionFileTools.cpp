/*! @file MotionFileTools.cpp
    @brief Implementation of file tools for motion

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

#include "MotionFileTools.h"

#include <cstring>
#include <sstream>
#include <iostream>
using namespace std;

float MotionFileTools::toFloat(const char* data)
{
    return atof(data);
}

float MotionFileTools::toFloat(const string& data)
{
    return toFloat(data.c_str());
}

float MotionFileTools::toFloat(istream& input)
{
    char buffer[128];
    input.ignore(128, ':');
    input.getline(buffer, 128);
    return toFloat(buffer);
}

string MotionFileTools::fromVector(vector<float> data)
{
    stringstream ss;
    ss << "[";
    for (unsigned int i=0; i<data.size()-1; i++)
        ss << data[i] << ", ";
    ss << data[data.size()-1] << "]";
    return ss.str();
}

vector<float> MotionFileTools::toFloatVector(const string& data)
{
    stringstream ss(data);
    string buffer;
    vector<float> values;
    while (getline(ss, buffer, ','))
        values.push_back(toFloat(buffer));
    
    return values;
}

vector<float> MotionFileTools::toFloatVector(istream& input)
{
    string buffer;
    input.ignore(128, '[');
    getline(input, buffer, ']');
    return toFloatVector(buffer);
}

string MotionFileTools::fromMatrix(const vector<vector<float> >& data)
{
    stringstream ss;
    ss << "[";
    for (unsigned int i=0; i<data.size(); i++)
        ss << fromVector(data[i]);
    ss << "]";
    return ss.str();
}

vector<vector<float> > MotionFileTools::toFloatMatrix(const string& data)
{
    stringstream ss(data);
    string buffer;
    vector<vector<float> > values;
    ss.ignore(10, '[');
    while(getline(ss, buffer, ']'))
    {
        values.push_back(toFloatVector(buffer));
        ss.ignore(10, '[');
    }
    
    return values;
}

vector<vector<float> > MotionFileTools::toFloatMatrix(istream& input)
{
    char buffer[256];
    input.ignore(256, '[');
    input.getline(buffer, 256);
    return toFloatMatrix(buffer);
}

unsigned int MotionFileTools::size(vector<vector<float> > data)
{
    unsigned int size = 0;
    for (unsigned int i=0; i<data.size(); i++)
        size += data[i].size();
    return size;
}

