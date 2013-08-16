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

namespace messages
{

	/*! @brief Default constructor for a parameter. Everything is initialised to 0/blank */
	Parameter::Parameter():
		Value(0),
		Min(0),
		Max(0)
	{
	}

	/*! @brief Constructor for an unnamed and indescribable parameter
		@param value the current value of the parameter
		@param min the minimum possible value of the parameter
		@param max the maximum possible value of the parameter
	 */
	Parameter::Parameter(float value, float min, float max):
		Name("Noname"),
		Value(value),
		Min(min),
		Max(max),
		Description("None")
	{
	}

	/*! @brief Constructor for a named but indescribable parameter
		@param name the name of the parameter
		@param value the current value of the parameter
		@param min the minimum possible value of the parameter
		@param max the maximum possible value of the parameter
	 */
	Parameter::Parameter(const std::string& name, float value, float min, float max):
		Name(name),
		Value(value),
		Min(min),
		Max(max),
		Description("None")
	{
	}

	/*! @brief Constructor for parameter
		@param name the name of the parameter
		@param value the current value of the parameter
		@param min the minimum possible value of the parameter
		@param max the maximum possible value of the parameter
		@param desc a short description of the parameters purpose and/or effect
	 */
	Parameter::Parameter(const std::string& name, float value, float min, float max, const std::string& desc):
		Name(name),
		Value(value),
		Min(min),
		Max(max),
		Description(desc)
	{
	}

	Parameter::~Parameter()
	{
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

	void Parameter::set(float value, float min, float max, const std::string& desc)
	{
		Min = min;
		Max = max;
		set(value);
		Description = desc;      
	}

	/*! @brief Prints a human-readble version of the walk parameter */
	void Parameter::summaryTo(std::ostream& output) 
	{
		output << Value;
	}

	/*! @brief Prints comma separated parameter */
	void Parameter::csvTo(std::ostream& output)
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

	std::vector<float> operator-(const std::vector<float>& f, const std::vector<Parameter>& p)
	{
		std::vector<float> result;
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

	std::vector<float> operator-(const std::vector<Parameter>& p, const std::vector<float>& f)
	{
		std::vector<float> result;
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

	std::vector<float> operator-(const std::vector<Parameter>& p1, const std::vector<Parameter>& p2)
	{
		std::vector<float> result;
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

	std::vector<float> operator+(const std::vector<float>& f, const std::vector<Parameter>& p)
	{
		std::vector<float> result;
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

	std::vector<float> operator+(const std::vector<Parameter>& p, const std::vector<float>& f)
	{
		return f + p;    
	}

	std::vector<float> operator+(const std::vector<Parameter>& p1, const std::vector<Parameter>& p2)
	{
		std::vector<float> result;
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

	void operator+=(std::vector<Parameter>& p, const std::vector<float>& f)
	{
		if (p.size() == f.size())
		{
			for (size_t i=0; i<p.size(); i++)
				p[i] += f[i];
		}
	}

	std::vector<float> operator*(const std::vector<float>& f, const std::vector<Parameter>& p)
	{
		std::vector<float> result;
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

	std::vector<float> operator*(const std::vector<Parameter>& p, const std::vector<float>& f)
	{
		return f*p;
	}

	std::vector<float> operator*(const float& f, const std::vector<Parameter>& p)
	{
		std::vector<float> result;
		result.reserve(p.size());
		for (size_t i=0; i<p.size(); i++)
			result.push_back(f*p[i]);
		return result;
	}

	std::vector<float> operator*(const std::vector<Parameter>& p, const float& f)
	{
		return f*p;
	}

	/*! @brief Returns the std::vector of Parameters back as a std::vector<float> containing only the values
	 * 	@return a std::vector<float> containing the current value of each parameter
	 */
	std::vector<float> Parameter::getAsVector(const std::vector<Parameter>& p)
	{
		std::vector<float> result;
		result.reserve(p.size());
		for (size_t i=0; i<p.size(); i++)
			result.push_back(p[i].get());
		return result;
	}

	/*! @brief Stream insertion operator for a single Parameters
			   The description and Parameter itself are terminated by a newline character.
	 */
	std::ostream& operator<< (std::ostream& output, const Parameter& p) 
	{   
		output << p.Name << ": " << p.Value << " [" << p.Min << ", " << p.Max << "] " << p.Description << std::endl;
		return output;
	}

	/*! @brief Stream insertion operator for a std::vector of Parameters.
	 *         Unlike other std::vectors, each entry is separated by a newline character, which also doubles
	 *         as the terminating character for the Parameter's description.
	 *  @relates Parameter
	 */
	std::ostream& operator<< (std::ostream& output, const std::vector<Parameter>& p)
	{
		output << "[";
		for (size_t i=0; i<p.size(); i++)
			output << p[i];
		output << "]";
		return output;
	}

	/*! @brief Stream extraction operator for a parameter.
	 * 	       Importantly, a single parameter takes an entire line, ie. The description must be terminated with a newline character.
	 *		   This has implications when puting parameters in std::vectors.
	 *  @relates Parameter
	 */
	std::istream& operator>> (std::istream& input, Parameter& p)
	{
		// read in the parameter name
		std::getline(input, p.Name, ':');
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
		p.Description = std::string(charbuffer);
    
		return input;
	}

	/*! @brief Stream extraction operator for a std::vector of parameters.
	 * 		   We need a specialised version for a std::vector of parameters because the entries are not separated by commas
	 * 		   as is the case with other std::vectors.
	 *  @relates Parameter
	 */
	std::istream& operator>> (std::istream& input, std::vector<Parameter>& p)
	{
		std::stringstream wholevector;
		p.clear();
		// get all of the data between [ ... ]
		input.ignore(128, '[');
		char c;
		int brackets = 1;
		while (brackets != 0 && input.good())
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
		while (wholevector.peek() != ']' && wholevector.good())
		{
			wholevector >> buffer;
			p.push_back(buffer);
		}
		return input;
	}

}
