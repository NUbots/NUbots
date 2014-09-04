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

#include <sstream>
#include <iostream>
#include <stdlib.h>
#include <algorithm>
#include <cctype>

/*! @brief Reads a boolean from a stream
    @param input the stream to read from
 */
bool MotionFileTools::toBool(std::istream& input)
{
    input.ignore(128, ':');
    std::string buffer;
    getline(input, buffer);
    buffer.erase(remove(buffer.begin(), buffer.end(), ' '), buffer.end());          // remove leading and trailing whitespace
    if (buffer.empty())
        return false;
    
    // Now there are two ways to specify a bool using 0 and non-zero or false and true
    if (isalpha(buffer[0]))
    {
        if (buffer[0] == 't' or buffer[0] == 'T')
            return true;
        else
            return false;
    }
    else
    {
        if (atof(buffer.c_str()) > 0)
            return true;
        else
            return false;
    }
    
}

/*! @brief Converts a cstring to a float
    @param data the cstring with the float in it
    @return the float
 */
float MotionFileTools::toFloat(const char* data)
{
    return atof(data);
}

/*! @brief Converts a string to a float
    @param data the string to convert
    @return the float
 */
float MotionFileTools::toFloat(const std::string& data)
{
    return toFloat(data.c_str());
}

/*! @brief Reads in a float value from a stream. All content before ':' is considered a comment, the float must immediately follow.
    @param input the stream to read from
    @param the float
 */
float MotionFileTools::toFloat(std::istream& input)
{
    char buffer[128];
    input.ignore(128, ':');
    input.getline(buffer, 128);
    return toFloat(buffer);
}

/*! @brief Converts a std::vector<float> to formatted string
 
    The data is written as [a,b,c, ...., n]
    @param data the data to convert
    @return the string containing the serialised vector
 */
std::string MotionFileTools::fromVector(std::vector<float> data)
{
    std::stringstream ss;
    ss << "[";
    if (not data.empty())
    {
        for (size_t i=0; i<data.size()-1; i++)
            ss << data[i] << ", ";
        ss << data[data.size()-1];
    }
    ss << "]";
    return ss.str();
}

/*! @brief Converts a std::vector<float> to formatted string
 
 The data is written as [a,b,c, ...., n]
 @param data the data to convert
 @return the string containing the serialised vector
 */
std::string MotionFileTools::fromVector(std::vector<double> data)
{
    std::stringstream ss;
    ss << "[";
    if (not data.empty())
    {
        for (size_t i=0; i<data.size()-1; i++)
            ss << data[i] << ", ";
        ss << data[data.size()-1];
    }
    ss << "]";
    return ss.str();
}

/*! @brief Reads a comma separated vector from a string
    @param data the string to read
    @return the std::vector<float>
 */
std::vector<float> MotionFileTools::toFloatVector(const std::string& data)
{
    std::stringstream ss(data);
    std::string buffer;
    std::vector<float> values;
    while (getline(ss, buffer, ','))
        values.push_back(toFloat(buffer));
    
    return values;
}

/*! @brief Reads a comma separated vector from a stream. All content before the '[' is considerd a comment, the stream is read up to the next ']'.
           For example, "Gains: [65, 100, 25, 30]".
    @param input the stream to read from
    @return the std::vector<float> from the stream
 */
std::vector<float> MotionFileTools::toFloatVector(std::istream& input)
{
    std::string buffer;
    input.ignore(128, '[');
    getline(input, buffer, ']');
    return toFloatVector(buffer);
}

/*! @brief Converts a std::vector<string> to a formated string
    @param data the data to be put in the string
    @return thre formatted string
 */
std::string MotionFileTools::fromVector(std::vector<std::string> data)
{
    std::stringstream ss;
    ss << "[";
    if (not data.empty())
    {
        for (size_t i=0; i<data.size()-1; i++)
            ss << data[i] << ", ";
        ss << data[data.size()-1];
    }
    ss << "]";
    return ss.str();
}

/*! @brief Reads the stream looking for comma separated list of strings. 
    @param input the stream to read from
    @return the list of strings
 */
std::vector<std::string> MotionFileTools::toStringVector(std::istream& input)
{
    std::vector<std::string> s;
    
    std::string line, token;
    getline(input, line);
    
    std::stringstream ss(line);
    while (getline(ss, token, ','))
    {
        token.replace(token.find('"'), 1, "", 0, 1);        // I don't like quotes around my joint labels
        token.replace(token.find('"'), 1, "", 0, 1);
        s.push_back(token);
    }
    
    return s;
}

/*! @brief Reads a value and min-max pair from a string.
    @param data the string to get the value [min,max] tuple from
    @param value the value from the string
    @param range the [min,max] tuple
 */
void MotionFileTools::toFloatWithRange(const std::string& data, float& value, std::vector<float>& range)
{
    std::stringstream ss(data);
    toFloatWithRange(data, value, range);
}

/*! @brief Reads a value and min-max pair from a stream. All content before the ':' is considered a comment. 
           For example, "Yaw limit: -0.27 [-0.95, 0.95]"
 
    @param data the string to get the value [min,max] tuple from
    @param value the value from the string
    @param range the [min,max] tuple
 */
void MotionFileTools::toFloatWithRange(std::istream& input, float& value, std::vector<float>& range)
{
    char buffer[128];
    input.ignore(128, ':');
    input.getline(buffer, 128, '[');
    value = toFloat(buffer);
    input.getline(buffer, 128, ']');
    range = toFloatVector(buffer);
}

/*! @brief Converts a matrix into a nicely formatted string. The matrix will look like [[a,b,...][m,n,...]...[x,y,...]]
    @param data the matrix to convert
    @return the serialised matrix
 */
std::string MotionFileTools::fromMatrix(const std::vector<std::vector<float> >& data)
{
    std::stringstream ss;
    ss << "[";
    for (size_t i=0; i<data.size(); i++)
        ss << fromVector(data[i]);
    ss << "]";
    return ss.str();
}

/*! @brief Converts a matrix into a nicely formatted string. The matrix will look like [[a,b,...][m,n,...]...[x,y,...]]
 @param data the matrix to convert
 @return the serialised matrix
 */
std::string MotionFileTools::fromMatrix(const std::vector<std::vector<double> >& data)
{
    std::stringstream ss;
    ss << "[";
    for (size_t i=0; i<data.size(); i++)
        ss << fromVector(data[i]);
    ss << "]";
    return ss.str();
}

/*! @brief Reads a matrix from a string.
    @param data the string containing the matrix
    @return the matrix
 */
std::vector<std::vector<float> > MotionFileTools::toFloatMatrix(const std::string& data)
{
    std::stringstream ss(data);
    std::string buffer;
    std::vector<std::vector<float> > values;
    ss.ignore(10, '[');
    while(getline(ss, buffer, ']'))
    {
        values.push_back(toFloatVector(buffer));
        ss.ignore(10, '[');
    }
    return values;
}

/*! @brief Reads a matrix from a string. All content before the '[' is considered a comment.
           For example: "Leg Gains: [[25,30,50,35][55,60,70]]"
    @param data the string containing the matrix
    @return the matrix
 */
std::vector<std::vector<float> > MotionFileTools::toFloatMatrix(std::istream& input)
{
    char buffer[512];
    input.ignore(512, '[');
    input.getline(buffer, 512);
    return toFloatMatrix(buffer);
}

/*! @brief Reads a value followed by a matrix. All content is converted.
           For example, "0.55: [], [], [0.05], [1.57,100]"
    @param input the stream to read the value-matrix pair
    @param value the value will be stored here
    @param matrix the matrix will be stored here
 */
void MotionFileTools::toFloatWithMatrix(std::istream& input, float& value, std::vector<std::vector<float> >& matrix)
{
    char buffer[128];
    input.getline(buffer, 128, ':');
    value = toFloat(buffer);
    matrix = toFloatMatrix(input);
}

/*! @brief Returns the size of a matrix
 */
size_t MotionFileTools::size(std::vector<std::vector<float> > data)
{
    size_t size = 0;
    for (size_t i=0; i<data.size(); i++)
        size += data[i].size();
    return size;
}

/*! @brief Returns the size of a matrix
 */
size_t MotionFileTools::size(std::vector<std::vector<double> > data)
{
    size_t size = 0;
    for (size_t i=0; i<data.size(); i++)
        size += data[i].size();
    return size;
}

