/*! @file Actionator.h
    @brief Declaration of a single actionator class
    @author Jason Kulk
 
    @class Actionator
    @brief A container for a single actionator, for example a single LED, a single Joint, an LCD display or a speaker.

    Actionator can handle several different types of data; floats, vectors, vector<vector>s and strings.
 
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

#ifndef ACTIONATOR_H
#define ACTIONATOR_H

#include "ActionatorPoint.h"

#include <vector>
#include <deque>
#include <string>
#include <pthread.h>


class Actionator 
{
public:
    Actionator(std::string actionatorname);
    ~Actionator();
    
    void preProcess();
    void postProcess(double currenttime);
    
    bool get(double& time, float& data);
    bool get(double& time, std::vector<float>& data);
    bool get(double& time, std::vector<std::vector<float> >& data);
    bool get(double& time, std::vector<std::vector<std::vector<float> > >& data);
    bool get(double& time, std::string& data);
    bool get(double& time, std::vector<std::string>& data);
    
    void add(const double& time, const float& data);
    void add(const double& time, const std::vector<float>& data);
    void add(const double& time, const std::vector<std::vector<float> >& data);
    void add(const double& time, const std::vector<std::vector<std::vector<float> > >& data);
    void add(const double& time, const std::string& data);
    void add(const double& time, const std::vector<std::string>& data);
    
    bool empty();
    
    void summaryTo(std::ostream& output);
    void csvTo(std::ostream& output);
    
    friend std::ostream& operator<< (std::ostream& output, const Actionator& p_actionator);
    friend std::istream& operator>> (std::istream& input, Actionator& p_actionator);
private:
    void addToBuffer(const ActionatorPoint& p);
public:
    std::string Name;                                     //!< the name of the actionator
private:
    std::deque<ActionatorPoint> m_points;                 //!< the double-ended queue of actionator points (it needs to be a deque because we remove from the front, and add to the back)
    std::vector<ActionatorPoint> m_add_points_buffer;     //!< a buffer of unordered points added since the last call to preProcess()
    std::vector<ActionatorPoint> m_preprocess_buffer;     //!< a local buffer for preProcess() to provide thread safety
    
    pthread_mutex_t m_lock;                          //!< lock for m_add_points_buffer
};

/*! @brief Returns true if there are no points in the queue, false if there are point to be applied
 */
inline bool Actionator::empty()
{
    return m_points.empty();
}

#endif

