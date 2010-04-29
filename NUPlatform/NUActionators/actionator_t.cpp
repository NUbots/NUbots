/*! @file actionator_t.cpp
    @brief Implementation of a single actionator class
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

#include "actionator_t.h"
#include "NUPlatform/NUSystem.h"
#include "debug.h"
#include "debugverbositynuactionators.h"

#include <algorithm>

/*! @brief Default constructor for a actionator_t. Initialises the actionator to be undefined and unavailable
 */
template <typename T> 
actionator_t<T>::actionator_t()
{
    Name = string("Undefined");
    ActionatorType = UNDEFINED;
    IsAvailable = false;
    m_previous_point = NULL;
}

/*! @brief Constructor for an actionator_t with known name and type
    @param actionatorname the name of the actionator
    @param actionatortype the type of the actionator to be used for RTTI
 */
template <typename T> 
actionator_t<T>::actionator_t(string actionatorname, actionator_type_t actionatortype)
{
    Name = actionatorname;
    ActionatorType = actionatortype;
    IsAvailable = true;
    m_previous_point = NULL;
}

/*! @brief Adds a point to the actionator
    @param time the time the point will be completed
    @param data the data for the point (careful the data size is hardcoded in many places, if you get it wrong your data will be ignored!)
 */
template <typename T>
void actionator_t<T>::addPoint(double time, const vector<T>& data)
{
    if (time < 0 || data.size() == 0)
    {
        debug << "actionator_t<T>::addPoint. " << Name << " Your data is invalid. It will be ignored!." << endl;
        return;
    }
    actionator_point_t* point = new actionator_point_t();
    point->Time = time;
    point->Data = data;

    // I need to keep the actionator points sorted based on their time
    if (m_points.size() == 0)           // the common (walk engine) case will be fast
        m_points.push_back(point);
    else
    {   // so instead of just pushing it to the back, I need to put it in the right place :D
        typename deque<actionator_point_t*>::iterator insertposition;
        insertposition = lower_bound(m_points.begin(), m_points.end(), point, comparePointTimes);
        m_points.resize((int) (insertposition - m_points.begin()));     // Clear all points after the new one 
        m_points.push_back(point);
    }
}

/*! @brief Remove all of the completed points
 @param currenttime the current time in milliseconds since epoch or program start (whichever you used to add the actionator point!)
 */
template <typename T>
void actionator_t<T>::removeCompletedPoints(double currenttime)
{
    if (m_points.size() > 0 && m_points[0]->Time <= currenttime)
        m_previous_point = m_points.front();            // I make the first point that is removed to previous point
    
    while (m_points.size() > 0 && m_points[0]->Time <= currenttime)
    {
        m_points.pop_front();
    }
}

/*! @brief Returns true if there are no points in the queue, false if there are point to be applied
 */
template <typename T>
bool actionator_t<T>::isEmpty()
{
    if (m_points.size() == 0)
        return true;
    else
        return false;
}

/*! Returns true if a should go before b, false otherwise.
 */
template <typename T>
bool actionator_t<T>::comparePointTimes(const void* a, const void* b)
{
    double timea = 0;
    double timeb = 0;
    
    const typename actionator_t<T>::actionator_point_t* a_a = reinterpret_cast<const typename actionator_t<T>::actionator_point_t*> (a);
    const typename actionator_t<T>::actionator_point_t* a_b = reinterpret_cast<const typename actionator_t<T>::actionator_point_t*> (b);
    timea = a_a->Time;
    timeb = a_b->Time;
    
    if (timea < timeb)
        return true;
    else
        return false;
}

/*! @brief Provides a text summary of the contents of the actionator_t
 
 The idea is to use this function when writing to a debug log. I guarentee that the 
 output will be human readable.
 
 @param output the ostream in which to put the string
 */
template <typename T>
void actionator_t<T>::summaryTo(ostream& output)
{
    output << Name << " ";
    if (isEmpty())
        output << "EMPTY" << endl;
    else {
        output << endl;
        for (unsigned int i=0; i<m_points.size(); i++)
        {
            output << m_points[i]->Time << ": ";
            for (unsigned int j=0; j<m_points[i]->Data.size(); j++)
                output << m_points[i]->Data[j] << " ";
            output << endl;
        }
    }

}

template <typename T>
void actionator_t<T>::csvTo(ostream& output)
{
}

template <typename T>
ostream& operator<< (ostream& output, const actionator_t<T>& p_actionator)
{
    //! @todo TODO: implement this function
    return output;
}

template <typename T>
istream& operator>> (istream& input, actionator_t<T>& p_actionator)
{
    //! @todo TODO: implement this function
    return input;
}

