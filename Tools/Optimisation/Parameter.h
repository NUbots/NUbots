/*! @file Parameter.h
    @brief Declaration of Parameter class
 
    @class Parameter
    @brief A simple class to store information about a parameter, including the value, min, max, name and a short description
 
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

#ifndef PARAMETER_H
#define PARAMETER_H

#include <vector>
#include <string>
#include <iostream>
#include "Tools/Math/StlVector.h"

class Parameter 
{
public:
    Parameter();
    Parameter(float value, float min, float max);
    Parameter(std::string name, float value, float min, float max);
    Parameter(std::string name, float value, float min, float max, std::string desc);
    ~Parameter();
    
    float get() const;
    float min() const;
    float max() const;
    std::string& name();
    bool compareName(const std::string n) const;
    std::string& desc();
    void set(float value);
    void set(float value, float min, float max, std::string desc);
    
    void summaryTo(std::ostream& output);
    void csvTo(std::ostream& output);
    
    friend float operator-(const Parameter& p, const float& f);
    friend float operator-(const float& f, const Parameter& p);
    friend float operator-(const Parameter& p1, const Parameter& p2);
    friend float operator+(const Parameter& p, const float& f);
    friend float operator+(const float& f, const Parameter& p);
    friend float operator+(const Parameter& p1, const Parameter& p2);
    friend void operator+=(Parameter& p, const float& f);
    friend float operator*(const float& f, const Parameter& p);
    friend float operator*(const Parameter& p, const float& f);
    friend float operator*(const Parameter& p1, const Parameter& p2);
    
    friend std::vector<float> operator-(const std::vector<float>& f, const std::vector<Parameter>& p);
    friend std::vector<float> operator-(const std::vector<Parameter>& p, const std::vector<float>& f);
    friend std::vector<float> operator-(const std::vector<Parameter>& p1, const std::vector<Parameter>& p2);
    friend std::vector<float> operator+(const std::vector<float>& f, const std::vector<Parameter>& p);
    friend std::vector<float> operator+(const std::vector<Parameter>& p, const std::vector<float>& f);
    friend std::vector<float> operator+(const std::vector<Parameter>& p1, const std::vector<Parameter>& p2);
    friend void operator+=(std::vector<Parameter>& p, const std::vector<float>& f);
    friend std::vector<float> operator*(const std::vector<float>& f, const std::vector<Parameter>& p);
    friend std::vector<float> operator*(const std::vector<Parameter>& p, const std::vector<float>& f);
    friend std::vector<float> operator*(const float& f, const std::vector<Parameter>& p);
    friend std::vector<float> operator*(const std::vector<Parameter>& p, const float& f);

    static std::vector<float> getAsVector(const std::vector<Parameter>& p);
    
    friend std::ostream& operator<< (std::ostream& output, const Parameter& p);
    friend std::ostream& operator<< (std::ostream& output, const std::vector<Parameter>& p);
    
    friend std::istream& operator>> (std::istream& input, Parameter& p);
    friend std::istream& operator>> (std::istream& input, std::vector<Parameter>& p);

private:
    std::string Name;
    float Value;
    float Min;
    float Max;
    std::string Description;
};

#endif
