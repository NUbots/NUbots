/*! @file NUbot.h
    @brief Serialisation and math for the standard template library vector.
 
    @author Jason Kulk
 
  Copyright (c) 2010 Jason Kulk
 
     This program is free software: you can redistribute it and/or modify
     it under the terms of the GNU General Public License as published by
     the Free Software Foundation, either version 3 of the License, or
     (at your option) any later version.
     
     This program is distributed in the hope that it will be useful,
     but WITHOUT ANY WARRANTY; without even the implied warranty of
     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
     GNU General Public License for more details.
     
     You should have received a copy of the GNU General Public License
     along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef STL_VECTOR_H
#define STL_VECTOR_H

#include <iostream>
#include <sstream>
using namespace std;

// ----------------------------------------------------------------------------------------------------------------------------- Serialisation
template<typename T> ostream& operator<<(ostream& output, const vector<T>& v)
{
    output << "[";
    if (not v.empty())
    {
    	for (size_t i=0; i<v.size()-1; i++)
            output << v[i] << ", ";
        output << v.back();
    }
    output << "]";
	return output;
}

template<typename T> istream& operator>>(istream& input, vector<T>& v)
{
    string wholevector;
    // get all of the data between [ ... ]
    input.ignore(128, '[');
    getline(input, wholevector, ']');
    
    v.clear();
    stringstream ss(wholevector);
    T buffer;
    
    // now split the data based on the commas
    while (ss.good())
    {
        ss >> buffer;
        ss.ignore(128, ',');
        v.push_back(buffer);
    }
    
    return input;
}

template<typename T> istream& operator>>(istream& input, vector<vector<T> >& v)
{
    v.clear();
    
    // read everything between '[' and the matching ']'
    input.ignore(128, '[');
    stringstream wholematrix;
    char c;
    int brackets = 1;
    while (brackets != 0)
    {
        input.get(c);
        wholematrix << c;
        if (c == '[')
            brackets++;
        else if (c == ']')
            brackets--;
    }
    
    vector<T> buffer;
    while (wholematrix.good())
    {
        wholematrix >> buffer;
        v.push_back(buffer);
    }
}

// ----------------------------------------------------------------------------------------------------------------------------- Math

#endif

