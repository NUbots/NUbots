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
#include <cmath>
#include <vector>
using namespace std;

// ----------------------------------------------------------------------------------------------------------------------------- Serialisation
/*! @brief Overloaded stream insertion operator for an stl vector. 
 
     Data will be written as ASCII in the following format [1.222, 2.333, -3.4444].
     Matricies are also supported with this templated function, they will be written as [[1.222, 2.333, -3.4444], [1.222, 2.333, -3.4444], [1.222, 2.333, -3.4444]]
 
    @param output the stream the stl will be written to
    @param v the vector to write
 */
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

/*! @brief Overloaded stream extraction operator for an stl vector. The vector in the stream needs to be formated as [1.222, 2.333, -3.4444].
    @param input the stream the vector will be read from
    @param v the vector from the stream will be placed here
 */
template<typename T> istream& operator>>(istream& input, vector<T>& v)
{
    stringstream wholevector;
    v.clear();
    // get all of the data between [ ... ]
    input.ignore(128, '[');
	char c;
	int brackets = 1;
	while (brackets != 0 and input.good())
	{
		input.get(c);
		wholevector << c;
		if (c == '[')
			brackets++;
		else if (c == ']')
			brackets--;
	}

	T buffer;
	// now split the data based on the commas
	while (wholevector.peek() != ']' and wholevector.good())
	{
		wholevector >> buffer;
		wholevector.ignore(128, ',');
		v.push_back(buffer);
	}
	return input;
}

/*! @brief Overloaded stream extraction operator for an stl vector. The vector in the stream needs to be formated as [[1.222, 2.333, -3.4444], [1.222, 2.333, -3.4444], [1.222, 2.333, -3.4444]].
    @param input the stream the vector will be read from
    @param v the vector from the stream will be placed here
 */
template<typename T> istream& operator>>(istream& input, vector<vector<T> >& v)
{
    stringstream wholematrix;
    v.clear();
    
    // read everything between '[' and the matching ']'
    input.ignore(128, '[');
    char c;
    int brackets = 1;
    while (brackets != 0 and input.good())
    {
        input.get(c);
        wholematrix << c;
        if (c == '[')
            brackets++;
        else if (c == ']')
            brackets--;
    }

    vector<T> buffer;
    while (wholematrix.peek() != ']' and wholematrix.good())
    {
        wholematrix >> buffer;
        v.push_back(buffer);
    }
    return input;
}

inline vector<float> operator+(const float& f, const vector<float>& v)
{
    vector<float> result;
    result.reserve(v.size());
    for (size_t i=0; i<v.size(); i++)
        result.push_back(f + v[i]);
    return result;
}

inline vector<float> operator+(const vector<float>& v, const float& f)
{
    return f + v;
}

inline vector<float> operator+(const vector<float>& v1, const vector<float>& v2)
{
    vector<float> result;
    if (v1.size() != v2.size())
        return result;
    else
    {
        result.reserve(v1.size());
        for (size_t i=0; i<v1.size(); i++)
            result.push_back(v1[i] + v2[i]);
        return result;
    }
}

inline vector<float> operator-(const float& f, const vector<float>& v)
{
    vector<float> result;
    result.reserve(v.size());
    for (size_t i=0; i<v.size(); i++)
        result.push_back(f - v[i]);
    return result;
}

inline vector<float> operator-(const vector<float>& v, const float& f)
{
    vector<float> result;
    result.reserve(v.size());
    for (size_t i=0; i<v.size(); i++)
        result.push_back(v[i] - f);
    return result;
}

inline vector<float> operator-(const vector<float>& v1, const vector<float>& v2)
{
    vector<float> result;
    if (v1.size() != v2.size())
        return result;
    else
    {
        result.reserve(v1.size());
        for (size_t i=0; i<v1.size(); i++)
            result.push_back(v1[i] - v2[i]);
        return result;
    }
}

inline vector<float> operator*(const float& f, const vector<float>& v)
{
    vector<float> result;
    result.reserve(v.size());
    for (size_t i=0; i<v.size(); i++)
        result.push_back(f*v[i]);
    return result;
}

inline vector<float> operator*(const vector<float>& v, const float& f)
{
    return f*v;
}

inline vector<float> operator*(const vector<float>& v1, const vector<float>& v2)
{
    vector<float> result;
    if (v1.size() != v2.size())
        return result;
    else
    {
        result.reserve(v1.size());
        for (size_t i=0; i<v1.size(); i++)
            result.push_back(v1[i]*v2[i]);
        return result;
    }
}

inline float norm(const vector<float>& v)
{
    float sum = 0;
    for (size_t i=0; i<v.size(); i++)
        sum += pow(v[i], 2);
    return sqrt(sum);
}

#endif

