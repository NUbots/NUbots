/*! @file actionator_t.h
    @brief Declaration of a single actionator class
    @author Jason Kulk
 
    @class actionator_t
    @brief A single actionator class
 
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
#include <deque>
#include <string>
using namespace std;

class actionator_t 
{
public:
    /*! @brief A simple struct for storing an actionator control point
     
     At this level you need to specify the entire control data, for example, if this is a joint position actionator then you need to
     specify (pos, vel, gain), or if it is an LED you need to specify (R, G, B). This behaviour is NOT negotiable. 
     */
    struct actionator_point_t 
    {
        double Time;                //!< the time the actionator point will be completed in milliseconds since epoch or program start
        vector<float> Data;         //!< the actual data to be given to the actionator, the contents depend on the actionator's type
    };
    
    /*! @brief A enum type to perform run time actionator type checking without using
     string compares.
     */
    enum actionator_type_t 
    {
        JOINT_POSITION,
        JOINT_TORQUE,
        CAMERA_SETTING,
        LEDS,
        SOUND,
        TELEPORTER,
        UNDEFINED
    };
public:
    actionator_t();
    actionator_t(string actionatorname, actionator_type_t actionatortype);
    
    void addPoint(double time, const vector<float>& data);
    void removeCompletedPoints(double currenttime);
    bool isEmpty();
    
    void summaryTo(ostream& output);
    void csvTo(ostream& output);
    
    friend ostream& operator<< (ostream& output, const actionator_t& p_actionator);
    friend istream& operator>> (istream& input, actionator_t& p_actionator);
public:
    string Name;                                //!< the name of the actionator
    actionator_type_t ActionatorType;           //!< the actionator type
    deque<actionator_point_t*> m_points;        //!< the double-ended queue of actionator points
    actionator_point_t* m_previous_point;       //!< the last actionator point that was applied
    bool IsAvailable;                           //!< true if the actionator is avaliable, false if it is absent
};

#endif

