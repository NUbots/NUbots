/*! @file actionator_t.h
    @brief Declaration of a single set of actionators class
    @author Jason Kulk
 
    @class actionator_t
    @brief A single set of actionators class to store actuator data in a platform independent way
 
    A single actionator for all joint positions
 
 First decision: should an each joint be a separate actionator? Well, honestly, I think it should.
 However, that is not how sensors are organised and I would, more than anything, rather keep it
 consistent. So that will pose a few limitations:
 
 vector<actionator_point_t> points; 
 
 vector<float> time, vector<float> value, vector<float> gain.
 
    @author Jason Kulk
 
  Copyright (c) 2009 Jason Kulk
 
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

#ifndef ACTIONATOR_T_H
#define ACTIONATOR_T_H

#include <vector>
#include <string>
using namespace std;

struct actionator_point_t 
{
    double Time;            //!< a single time for each actuator point
    vector<bool> IsValid;   //!< use this flag to effect only selected subactionators in this group    
    vector<float> Values;   //!< the actual values to be given to the group of actionators
    vector<float> Gains;    //!< the gains/strength/stiffness/brightness etc for the actionator if applicable
}

/*! @brief A enum type to perform run time actionator type checking without using
           string compares. Unfortunately, to add a new actionator you need to add an
           id to this list.
 */
enum actionator_id_t 
{
    JOINT_POSITIONS,
    UNDEFINED
};


class actionator_t 
{
public:
    actionator_t();
    
    /*void summaryTo(ostream& output);
    void csvTo(ostream& output);
    
    friend ostream& operator<< (ostream& output, const actionator_t& p_actionator);
    friend istream& operator>> (istream& input, actionator_t& p_actionator);*/
};

#endif

