/*! @file actionator_t.h
    @brief Declaration of a single actionator class
    @author Jason Kulk
 
    @class actionator_t
    @brief A container for a single actionator, for example a single LED, or a single Joint.

    You can make groups of related actionators by using vectors, lists etc. In fact, the JointActionators
    are merely a vector of actionator_t's.

    actionator_t is a template class. It is a template so that we can have an actionator_t for several types;
    a joint requires the data to be a float, while the sound requires the data to be a string.
 
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

#ifndef ACTIONATOR_T_H
#define ACTIONATOR_T_H

#include <vector>
#include <deque>
#include <string>
using namespace std;

template <typename T = float>
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
        vector<T> Data;             //!< the actual data to be given to the actionator, the contents depend on the actionator's type
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
    
    void addPoint(double time, const vector<T>& data);
    void preProcess();
    void removeCompletedPoints(double currenttime);
    bool isEmpty();
    
    void summaryTo(ostream& output);
    void csvTo(ostream& output);
    
    template <typename TT> friend ostream& operator<< (ostream& output, const actionator_t<TT>& p_actionator);
    template <typename TT> friend istream& operator>> (istream& input, actionator_t<TT>& p_actionator);
public:
    string Name;                                        //!< the name of the actionator
    actionator_type_t ActionatorType;                   //!< the actionator type
    deque<actionator_point_t> m_points;                 //!< the double-ended queue of actionator points (it needs to be a deque because we remove from the front, and add to the back)
    bool IsAvailable;                                   //!< true if the actionator is avaliable, false if it is absent
private:
    vector<actionator_point_t> m_add_points_buffer;     //!< a buffer of unordered points added since the last call to preProcess()
    vector<actionator_point_t> m_preprocess_buffer;     //!< a local buffer for preProcess() to provide thread safety
    static bool comparePoints(const actionator_point_t& a, const actionator_point_t& b);
};

#include "actionator_t.cpp"                     // this is the standard way to do template classes when you separate declaration and implementation.
                                                // just make sure that you don't compile actionator_t.cpp separately

#endif

