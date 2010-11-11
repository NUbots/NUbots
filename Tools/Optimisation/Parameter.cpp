/*! @file Parameter.cpp
    @brief Implementation of Parameter class

    @author Jason Kulk
 
 Copyright (c) 2009, 2010 Jason Kulk
 
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

#include "Parameter.h"

#include <cmath>

#include <sstream>
#include "debug.h"

/*! @brief Default constructor for a parameter. Everything is initialised to 0/blank */
Parameter::Parameter() 
{
    Value = 0; 
    Min = 0; 
    Max = 0;
}

/*! @brief Constructor for an unnamed and indescribable parameter
    @param value the current value of the parameter
    @param min the minimum possible value of the parameter
    @param max the maximum possible value of the parameter
 */
Parameter::Parameter(float value, float min, float max)
{
    Name = "Noname"; 
    Description = "None"; 
    Value = value; 
    Min = min; 
    Max = max;
}

/*! @brief Constructor for a named but indescribable parameter
    @param name the name of the parameter
    @param value the current value of the parameter
    @param min the minimum possible value of the parameter
    @param max the maximum possible value of the parameter
 */
Parameter::Parameter(string name, float value, float min, float max) 
{
    Name = name; 
    Description = "None"; 
    Value = value; 
    Min = min; 
    Max = max;
}

/*! @brief Constructor for parameter
    @param name the name of the parameter
    @param value the current value of the parameter
    @param min the minimum possible value of the parameter
    @param max the maximum possible value of the parameter
    @param desc a short description of the parameters purpose and/or effect
 */
Parameter::Parameter(string name, float value, float min, float max, string desc) 
{
    Name = name; 
    Description = desc; 
    Value = value; 
    Min = min; 
    Max = max;
}

Parameter::~Parameter()
{
}

/*! @brief Gets the parameter's currrent value 
    @return the current value
*/
float Parameter::get() const
{
    return Value;
}

/*! @brief Gets the parameter's minimum value 
 	@return the min
 */
float Parameter::min() const
{
    return Min;
}

/*! @brief Gets the parameter's minimum value 
 	@return the min
 */
float Parameter::max() const
{
    return Max;
}

/*! @brief Gets the parameter's name 
 	@return the name
 */
string& Parameter::name()
{
    return Name;
}

/*! @brief Gets the description of the parameter
 	@return the description
 */
string& Parameter::desc()
{
    return Description;
}

/*! @brief Sets the value of the parameter, observing the parameters min and max values
    @param value the new parameter value
*/
void Parameter::set(float value)
{
    if (value < Min) 
        Value = Min;
    else if (value > Max) 
        Value = Max;
    else 
        Value = value;
}

/*! @brief Prints a human-readble version of the walk parameter */
void Parameter::summaryTo(ostream& output) 
{
    output << Value;
}

/*! @brief Prints comma separated parameter */
void Parameter::csvTo(ostream& output)
{
    output << Value << ", ";
}

/*! @brief Subtraction operator for float and parameter. Returns the difference of their two values */
float operator-(const Parameter& p, const float& f)
{
    return p.Value - f;
}

/*! @brief Subtraction operator for float and parameter. Returns the difference of their two values */
float operator-(const float& f, const Parameter& p)
{
    return f - p.Value;
}

/*! @brief Subtraction operator for two parameters. Returns the difference of their two values */
float operator-(const Parameter& p1, const Parameter& p2)
{
    return p1.Value - p2.Value;
}

/*! @brief Addition operator for float and parameter. Returns the the sum of their values */
float operator+(const Parameter& p, const float& f)
{
    return p.Value + f;
}

/*! @brief Addition operator for float and parameter. Returns the the sum of their values */
float operator+(const float& f, const Parameter& p)
{
    return f + p.Value;
}

/*! @brief Addition operator for two parameters. Returns the sum of their two values */
float operator+(const Parameter& p1, const Parameter& p2)
{
    return p1.Value + p1.Value;
}

/*! @brief Sum assignement operator for a parameter and a float. Returns a new parameter whose value is p.Value + f */
void operator+=(Parameter& p, const float& f)
{
	p.set(p.Value + f);
}

/*! @brief Mulitplication operator for float and parameter. Returns the product of float and p.Value */
float operator*(const float& f, const Parameter& p)
{
    return f*p.Value;
}

/*! @brief Mulitplication operator for float and parameter. Returns the product of float and p.Value */
float operator*(const Parameter& p, const float& f)
{
    return f*p.Value;
}

/*! @brief Mulitplication operator two parameters. Returns the product of their values */
float operator*(const Parameter& p1, const Parameter& p2)
{
    return p1.Value*p2.Value;
}

vector<float> operator-(const vector<float>& f, const vector<Parameter>& p)
{
    vector<float> result;
    if (f.size() != p.size())
        return result;
    else
    {
        result.reserve(p.size());
        for (size_t i=0; i<p.size(); i++)
            result.push_back(f[i] - p[i]);
            return result;
    }
}

vector<float> operator-(const vector<Parameter>& p, const vector<float>& f)
{
    vector<float> result;
    if (f.size() != p.size())
        return result;
    else
    {
        result.reserve(p.size());
        for (size_t i=0; i<p.size(); i++)
            result.push_back(p[i] - f[i]);
        return result;
    } 
}

vector<float> operator-(const vector<Parameter>& p1, const vector<Parameter>& p2)
{
    vector<float> result;
    if (p1.size() != p2.size())
        return result;
    else
    {
        result.reserve(p1.size());
        for (size_t i=0; i<p1.size(); i++)
            result.push_back(p1[i] - p2[i]);
        return result;
    }
}

vector<float> operator+(const vector<float>& f, const vector<Parameter>& p)
{
    vector<float> result;
    if (f.size() != p.size())
        return result;
    else
    {
        result.reserve(f.size());
        for (size_t i=0; i<f.size(); i++)
            result.push_back(f[i] + p[i]);
        return result;
    }
}

vector<float> operator+(const vector<Parameter>& p, const vector<float>& f)
{
	return f + p;    
}

vector<float> operator+(const vector<Parameter>& p1, const vector<Parameter>& p2)
{
    vector<float> result;
    if (p1.size() != p2.size())
        return result;
    else
    {
        result.reserve(p1.size());
        for (size_t i=0; i<p1.size(); i++)
            result.push_back(p1[i] + p2[i]);
        return result;
    }
}

void operator+=(vector<Parameter>& p, const vector<float>& f)
{
    if (p.size() == f.size())
    {
        for (size_t i=0; i<p.size(); i++)
            p[i] += f[i];
    }
}

vector<float> operator*(const vector<float>& f, const vector<Parameter>& p)
{
    vector<float> result;
    if (f.size() != p.size())
        return result;
    else
    {
        result.reserve(p.size());
        for (size_t i=0; i<p.size(); i++)
            result.push_back(f[i]*p[i]);
        return result;
    }
}

vector<float> operator*(const vector<Parameter>& p, const vector<float>& f)
{
    return f*p;
}

vector<float> operator*(const float& f, const vector<Parameter>& p)
{
    vector<float> result;
    result.reserve(p.size());
    for (size_t i=0; i<p.size(); i++)
        result.push_back(f*p[i]);
    return result;
}

vector<float> operator*(const vector<Parameter>& p, const float& f)
{
    return f*p;
}

vector<float> Parameter::getAsVector(const vector<Parameter>& p)
{
    vector<float> result;
    result.reserve(p.size());
    for (size_t i=0; i<p.size(); i++)
        result.push_back(p[i].get());
    return result;
}

ostream& operator<< (ostream& output, const Parameter& p) 
{   
    output << p.Name << ": " << p.Value << " [" << p.Min << ", " << p.Max << "] " << p.Description;
    return output;
}

ostream& operator<< (ostream& output, const vector<Parameter>& p)
{
    output << "[";
    if (not p.empty())
    {
    	for (size_t i=0; i<p.size()-1; i++)
            output << p[i] << endl;
        output << p.back();
    }
    output << "]";
	return output;
}

istream& operator>> (istream& input, Parameter& p)
{
    // read in the parameter name
    getline(input, p.Name, ':');
    if (p.Name[p.Name.size() - 1] == ':')
        p.Name.resize(p.Name.size() - 1);
    
    // read in the value [min, max]
    input >> p.Value;
    input.ignore(10, '[');
    input >> p.Min;
    input.ignore(10, ',');
    input >> p.Max;
    input.ignore(10, ']');
    
    // read in the rest of the line and call it the description
    input.ignore(128, ' ');
    char charbuffer[500];
    input.getline(charbuffer, 500);
    p.Description = string(charbuffer);
    
    return input;
}

istream& operator>> (istream& input, vector<Parameter>& p)
{
    p.clear();
    input.ignore(128, '[');
    
    Parameter t;
    while (not input.eof())
    {
        input >> t;
		size_t last = t.Description.size()-1;
        
        if (last >=0 and t.Description[last] == ']')
        {
            t.Description.erase(last);
            p.push_back(t);
            break;
        }
        p.push_back(t);
    }
    return input;
}

// -----------------------------------------------------------------------------------------------------------------------------------
vector<float> operator+(const float& f, const vector<float>& v)
{
    vector<float> result;
    result.reserve(v.size());
    for (size_t i=0; i<v.size(); i++)
        result.push_back(f + v[i]);
    return result;
}

vector<float> operator+(const vector<float>& v, const float& f)
{
    return f + v;
}

vector<float> operator+(const vector<float>& v1, const vector<float>& v2)
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

vector<float> operator-(const float& f, const vector<float>& v)
{
    vector<float> result;
    result.reserve(v.size());
    for (size_t i=0; i<v.size(); i++)
        result.push_back(f - v[i]);
    return result;
}

vector<float> operator-(const vector<float>& v, const float& f)
{
    vector<float> result;
    result.reserve(v.size());
    for (size_t i=0; i<v.size(); i++)
        result.push_back(v[i] - f);
    return result;
}

vector<float> operator-(const vector<float>& v1, const vector<float>& v2)
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

vector<float> operator*(const float& f, const vector<float>& v)
{
    vector<float> result;
    result.reserve(v.size());
    for (size_t i=0; i<v.size(); i++)
        result.push_back(f*v[i]);
    return result;
}

vector<float> operator*(const vector<float>& v, const float& f)
{
    return f*v;
}

vector<float> operator*(const vector<float>& v1, const vector<float>& v2)
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

float norm(const vector<float>& v)
{
    float sum = 0;
    for (size_t i=0; i<v.size(); i++)
        sum += pow(v[i], 2);
    return sqrt(sum);
}

ostream& operator<<(ostream& output, const vector<float>& v)
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

ostream& operator<<(ostream& output, const vector<vector<float> >& v)
{
    output << "[";
    if (not v.empty())
    {
        for (size_t i=0; i<v.size()-1; i++)
            output << v[i] << ",";
        output << v.back();
    }
    output << "]";
    return output;
}

istream& operator>>(istream& input, vector<float>& v)
{
    string wholevector;
    // get all of the data between [ ... ]
    input.ignore(128, '[');
    getline(input, wholevector, ']');
    
    v.clear();
    stringstream ss(wholevector);
    float floatBuffer;
    
    // now split the data based on the commas
    while (not ss.eof())
    {
        ss >> floatBuffer;
        ss.ignore(128, ',');
        v.push_back(floatBuffer);
    }
    
    return input;
}

istream& operator>>(istream& input, vector<vector<float> >& v)
{
    string buffer;
    getline(input, buffer);
    stringstream ss(buffer);
    ss.ignore(128, '[');
    while(getline(ss, buffer, ']'))
    {
        vector<float> e;
        ss >> e;
        v.push_back(e);
        ss.ignore(128, '[');
    }
    return input;
}

