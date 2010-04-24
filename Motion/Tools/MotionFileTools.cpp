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
#include <iostream>
using namespace std;

vector<float> MotionFileTools::toFloatVector(char* data)
{
    char *pch = strtok (data, " ,/t");
    vector<float> values;
    while (pch != NULL)
    {
        values.push_back(atof(pch));
        pch = strtok (NULL, " ,/t");
    }
    return values;
}

vector<float> MotionFileTools::toFloatVector(string& data)
{
    char* buffer = (char*) data.c_str();
    return toFloatVector(buffer);
}

vector<float> MotionFileTools::toFloatVector(istream& input)
{
    char buffer[128];
    input.ignore(128, '[');
    input.getline(buffer, 128, ']');
    return toFloatVector(buffer);
}