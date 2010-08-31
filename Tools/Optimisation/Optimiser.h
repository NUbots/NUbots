/*! @file Optimiser.h
    @brief Declaration of an abstract optimiser class
 
    @class Optimiser
    @brief An abstract optimiser class
 
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

#ifndef OPTIMISER_H
#define OPTIMISER_H

class Parameter;

#include <string>
#include <vector>
#include <iostream>
using namespace std;

class Optimiser
{
public:
    Optimiser(string name, vector<Parameter> parameters);
    ~Optimiser();
    
    virtual vector<float> getNextParameters() = 0;
    virtual void setParametersResult(float fitness) = 0;
    
    virtual void summaryTo(ostream& stream) = 0;
    friend ostream& operator<<(ostream& o, const Optimiser& optimser);
    friend ostream& operator<<(ostream& o, const Optimiser* optimser);
    friend istream& operator>>(istream& i, Optimiser& optimser);
    friend istream& operator>>(istream& i, Optimiser* optimser);
    void save();
    void saveAs(string name);
    void load();
protected:
    virtual void toStream(ostream& o) const = 0;
    virtual void fromStream(istream& i) = 0;
protected:
    string m_name;
    vector<Parameter> m_initial_parameters;
};

#endif

