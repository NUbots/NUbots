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
            output << p[i].get() << ", ";
        output << p.back().get();
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
    char charbuffer[500];
    input.getline(charbuffer, 500);
    p.Description = string(charbuffer);
    
    return input;
}
