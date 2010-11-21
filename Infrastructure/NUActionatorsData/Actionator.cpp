/*! @file Actionator.cpp
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

#include "Actionator.h"

#include "debug.h"
#include "debugverbositynuactionators.h"

#include <algorithm>

/*! @brief Constructor for an Actionator with known name and type
    @param actionatorname the name of the actionator
 */
Actionator::Actionator(string actionatorname)
{
    Name = actionatorname;

    m_add_points_buffer.reserve(1024);
    m_preprocess_buffer.reserve(1024);
    int err;
    err = pthread_mutex_init(&m_lock, NULL);
    if (err != 0)
        errorlog << "Actionator::Actionator(" << Name << ") Failed to create m_lock." << endl;
}

/*! @brief Destroys the Actionator */
Actionator::~Actionator()
{
    pthread_mutex_destroy(&m_lock);
}

/*! @brief Attempts to get the next float data for this actionator. If there is none, return false.
 	@param time will be updated with the time associated with the data
 	@param data will be updated 
 	@return true if time,data were successfully updated, false otherwise
 */
bool Actionator::get(double& time, float& data)
{
    if (not empty())
    {
        ActionatorPoint& p = m_points[0];
        if (p.FloatData)
        {
        	time = p.Time;
    		data = *(p.FloatData);
            return true;
        }
    }
    return false;
}

/*! @brief Attempts to get the next float data for this actionator. If there is none, return false.
    @param time will be updated with the time associated with the data
    @param data will be updated 
    @return true if time,data were successfully updated, false otherwise
 */
bool Actionator::get(double& time, vector<float>& data)
{
    if (not empty())
    {
        ActionatorPoint& p = m_points[0];
        if (p.VectorData)
        {
        	time = p.Time;
    		data = *(p.VectorData);
            return true;
        }
    }
    return false;
}

/*! @brief Attempts to get the next float data for this actionator. If there is none, return false.
    @param time will be updated with the time associated with the data
    @param data will be updated 
    @return true if time,data were successfully updated, false otherwise
 */
bool Actionator::get(double& time, vector<vector<float> >& data)
{
    if (not empty())
    {
        ActionatorPoint& p = m_points[0];
        if (p.MatrixData)
        {
        	time = p.Time;
    		data = *(p.MatrixData);
            return true;
        }
    }
    return false;
}

/*! @brief Attempts to get the next float data for this actionator. If there is none, return false.
    @param time will be updated with the time associated with the data
    @param data will be updated 
    @return true if time,data were successfully updated, false otherwise
 */
bool Actionator::get(double& time, vector<vector<vector<float> > >& data)
{
    if (not empty())
    {
        ActionatorPoint& p = m_points[0];
        if (p.ThreeDimData)
        {
        	time = p.Time;
    		data = *(p.ThreeDimData);
            return true;
        }
    }
    return false;
}

/*! @brief Attempts to get the next float data for this actionator. If there is none, return false.
    @param time will be updated with the time associated with the data
    @param data will be updated 
    @return true if time,data were successfully updated, false otherwise
 */
bool Actionator::get(double& time, string& data)
{
    if (not empty())
    {
        ActionatorPoint& p = m_points[0];
        if (p.StringData)
        {
        	time = p.Time;
    		data = *(p.StringData);
            return true;
        }
    }
    return false;
}


/*! @brief Add an actionator point to the actionator
    @param time the time the data will be applied
    @param data the data associated with the point (single float)
 */
void Actionator::add(const double& time, const float& data)
{
    ActionatorPoint p(time, data);
    addToBuffer(p);
}

/*! @brief Add an actionator point to the actionator
    @param time the time the data will be applied
    @param data the data associated with the point (vector of float)
 */
void Actionator::add(const double& time, const vector<float>& data)
{
    ActionatorPoint p(time, data);
    addToBuffer(p);
}

/*! @brief Add an actionator point to the actionator
    @param time the time the data will be applied
    @param data the data associated with the point (matrix of float)
 */
void Actionator::add(const double& time, const vector<vector<float> >& data)
{
    ActionatorPoint p(time, data);
    addToBuffer(p);
}

/*! @brief Add an actionator point to the actionator
    @param time the time the data will be applied
    @param data the data associated with the point (3d matrix of float)
 */
void Actionator::add(const double& time, const vector<vector<vector<float> > >& data)
{
    ActionatorPoint p(time, data);
    addToBuffer(p);
}

/*! @brief Add an actionator point to the actionator
    @param time the time the data will be applied
    @param data the data associated with the point
 */
void Actionator::add(const double& time, const string& data)
{
    ActionatorPoint p(time, data);
    addToBuffer(p);
}

/*! @brief Pushes the point to the back of the m_add_points_buffer */
void Actionator::addToBuffer(const ActionatorPoint& p)
{
    pthread_mutex_lock(&m_lock);
    m_add_points_buffer.push_back(p);
    pthread_mutex_unlock(&m_lock);
}

/*! @brief Preprocesses the data for the actionator
 */
void Actionator::preProcess()
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
        sort(m_preprocess_buffer.begin(), m_preprocess_buffer.end());
        
        // because I did (a) and I choose to clear all existing points later in time
        // I can simply find the location where the first point should be inserted, and then insert ALL new points after that
        if (not m_points.empty())
        {
            deque<ActionatorPoint>::iterator insertposition;
            insertposition = lower_bound(m_points.begin(), m_points.end(), m_preprocess_buffer.front());
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
void Actionator::postProcess(double currenttime)
{
    while (not m_points.empty() and m_points[0].Time <= currenttime)
        m_points.pop_front();
}

/*! @brief Provides a text summary of the contents of the Actionator
 
 The idea is to use this function when writing to a debug log. I guarentee that the 
 output will be human readable.
 
 @param output the ostream in which to put the string
 */
void Actionator::summaryTo(ostream& output)
{
    if (not empty())
    {
        output << Name << " ";
        for (unsigned int i=0; i<m_points.size(); i++)
            output << m_points[i] << " ";
        output << endl;
    }
}

void Actionator::csvTo(ostream& output)
{
}


ostream& operator<< (ostream& output, const Actionator& p_actionator)
{
    //! @todo TODO: implement this function
    return output;
}


istream& operator>> (istream& input, Actionator& p_actionator)
{
    //! @todo TODO: implement this function
    return input;
}

