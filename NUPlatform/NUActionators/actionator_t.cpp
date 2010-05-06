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
    m_add_points_buffer.reserve(1024);
    m_preprocess_buffer.reserve(1024);
    IsAvailable = false;
    int err;
    err = pthread_mutex_init(&m_lock, NULL);
    if (err != 0)
        errorlog << "actionator_t<T>::actionator_t(" << Name << ") Failed to create m_lock." << endl;
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
    m_add_points_buffer.reserve(1024);
    m_preprocess_buffer.reserve(1024);
    IsAvailable = true;
    int err;
    err = pthread_mutex_init(&m_lock, NULL);
    if (err != 0)
    {
        errorlog << "actionator_t<T>::actionator_t(" << Name << ") Failed to create m_lock." << endl;
        IsAvailable = false;
    }
}

/*! @brief Adds a point to the actionator
    @param time the time the point will be completed
    @param data the data for the point 
 */
template <typename T>
void actionator_t<T>::addPoint(double time, const vector<T>& data)
{
    if (data.size() == 0)
    {
        debug << "actionator_t<T>::addPoint. " << Name << " Your data is invalid. It will be ignored!." << endl;
        return;
    }
    actionator_point_t point;
    point.Time = time;
    point.Data = data;
    
    pthread_mutex_lock(&m_lock);
    m_add_points_buffer.push_back(point);
    pthread_mutex_unlock(&m_lock);
}

/*! @brief
 */
template <typename T>
void actionator_t<T>::preProcess()
{
    if (m_add_points_buffer.empty())
        return;
    else if (pthread_mutex_trylock(&m_lock))
        return;
    else
    {
        m_preprocess_buffer.swap(m_add_points_buffer);
        pthread_mutex_unlock(&m_lock);
        // I need to keep the actionator points sorted based on their time.
        //      (a) I need to sort the buffer before adding the points
        //      (b) I need to search m_points for the correct place to add new point(s)
        sort(m_preprocess_buffer.begin(), m_preprocess_buffer.end(), comparePoints);
        
        // because I did (a) and I choose to clear all existing points later in time
        // I can simply find the location where the first point should be inserted, and then insert ALL new points after that
        if (m_points.empty())
        {
            typename deque<actionator_point_t>::iterator insertposition;
            insertposition = lower_bound(m_points.begin(), m_points.end(), m_preprocess_buffer.front(), comparePoints);
            m_points.erase(insertposition, m_points.end());     // Clear all points after the new one 
        }
        
        m_points.insert(m_points.end(), m_preprocess_buffer.begin(), m_preprocess_buffer.end());

        // clear the preprocess buffer after I have added all of the points
        m_preprocess_buffer.clear();
    }
}

/*! @brief Remove all of the completed points
     @param currenttime the current time in milliseconds since epoch or program start (whichever you used to add the actionator point!)
 */
template <typename T>
void actionator_t<T>::removeCompletedPoints(double currenttime)
{
    while (not m_points.empty() and m_points[0].Time <= currenttime)
        m_points.pop_front();
}

/*! @brief Returns true if there are no points in the queue, false if there are point to be applied
 */
template <typename T>
bool actionator_t<T>::isEmpty()
{
    return m_points.empty();
}

/*! Returns true if a should go before b, false otherwise.
 */
template <typename T>
bool actionator_t<T>::comparePoints(const actionator_point_t& a, const actionator_point_t& b)
{
    return a.Time < b.Time;
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
            output << m_points[i].Time << ": ";
            for (unsigned int j=0; j<m_points[i].Data.size(); j++)
                output << m_points[i].Data[j] << " ";
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

