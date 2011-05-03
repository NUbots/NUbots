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

#include "Tools/Math/StlVector.h"

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

void Parameter::set(float value, float min, float max, string desc)
{
    if (value < min) 
        Value = min;
    else if (value > max) 
        Value = max;
    else 
        Value = value;
        
    Min = min;
    Max = max;
    
    Description = desc;      
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
    return p1.Value + p2.Value;
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

/*! @brief Returns the vector of Parameters back as a vector<float> containing only the values
 * 	@return a vector<float> containing the current value of each parameter
 */
vector<float> Parameter::getAsVector(const vector<Parameter>& p)
{
    vector<float> result;
    result.reserve(p.size());
    for (size_t i=0; i<p.size(); i++)
        result.push_back(p[i].get());
    return result;
}

/*! @brief Stream insertion operator for a single Parameters
		   The description and Parameter itself are terminated by a newline character.
 */
ostream& operator<< (ostream& output, const Parameter& p) 
{   
    output << p.Name << ": " << p.Value << " [" << p.Min << ", " << p.Max << "] " << p.Description << endl;
    return output;
}

/*! @brief Stream insertion operator for a vector of Parameters.
 *         Unlike other vectors, each entry is separated by a newline character, which also doubles
 *         as the terminating character for the Parameter's description.
 *  @relates Parameter
 */
ostream& operator<< (ostream& output, const vector<Parameter>& p)
{
    output << "[";
	for (size_t i=0; i<p.size(); i++)
		output << p[i];
    output << "]";
	return output;
}

/*! @brief Stream extraction operator for a parameter.
 * 	       Importantly, a single parameter takes an entire line, ie. The description must be terminated with a newline character.
 *		   This has implications when puting parameters in vectors.
 *  @relates Parameter
 */
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

/*! @brief Stream extraction operator for a vector of parameters.
 * 		   We need a specialised version for a vector of parameters because the entries are not separated by commas
 * 		   as is the case with other vectors.
 *  @relates Parameter
 */
istream& operator>> (istream& input, vector<Parameter>& p)
{
    stringstream wholevector;
    p.clear();
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

	Parameter buffer;
	// now split the data based on the commas
	while (wholevector.peek() != ']' and wholevector.good())
	{
		wholevector >> buffer;
		p.push_back(buffer);
	}
	return input;
}

