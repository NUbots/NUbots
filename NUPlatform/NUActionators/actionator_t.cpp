/*! @file actionator_t.cpp
    @brief Implementation of a single set of actionator class
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

#include "actionator_t.h"
#include "NUPlatform/NUSystem.h"
#include "Tools/debug.h"

#include <algorithm>

static bool comparePointTimes(const void* a, const void* b);     //! @todo TODO: make this a member of the class!


/*! @brief Default constructor for a actionator_t. Initialises the actionator to be undefined and invalid
 */
actionator_t::actionator_t()
{
    Name = string("Undefined");
    ActionatorID = UNDEFINED;
    IsAvailable = false;
}

/*! @brief Constructor for an actionator_t with known name and id
    @param actionatorname the name of the actionator
    @param actionatorid the id of the actionator to be used for RTTI
 */
actionator_t::actionator_t(string actionatorname, actionator_id_t actionatorid)
{
    Name = actionatorname;
    ActionatorID = actionatorid;
    IsAvailable = false;
}

/*! @brief Adds an action to this actionator with the specified times and values
 
    This function will keep m_points sorted based on the time they need to be executed,
    and it will clear all points after the new one.
 
    @param time the time the action will be applied
    @param isvalid a vector of bools, if an entry is false the corresponding element
                   in values will be ignored, and the current value will be reused
    @param values the actionator values to be applied 
    @param gains the strength/stiffness/brightness etc of the values
 */
void actionator_t::addAction(double time, vector<bool>& isvalid, vector<float>& values, vector<float>& gains)
{
    actionator_point_t* point = new actionator_point_t();
    point->Time = time;
    point->IsValid = isvalid;
    point->Values = values;
    point->Gains = gains;
    // I need to keep the actionator points sorted based on their time
    if (m_points.size() == 0)           // the common (walk engine) case will be fast
        m_points.push_back(point);
    else
    {   // so instead of just pushing it to the back, I need to put it in the right place :(
        static vector<actionator_point_t*>::iterator insertposition;
        insertposition = lower_bound(m_points.begin(), m_points.end(), point, comparePointTimes);
        m_points.resize((int) (insertposition - m_points.begin()));     // Clear all points after the new one 
        m_points.push_back(point);  
        
        // m_points.insert(insertposition, point); // Merge-type actionator points
    }
    
}

/*! Returns true if a should go before b, false otherwise.
 */
bool comparePointTimes(const void* a, const void* b)
{
    static double timea = 0;
    static double timeb = 0;
    
    timea = ((actionator_t::actionator_point_t*)a)->Time;
    timeb = ((actionator_t::actionator_point_t*)b)->Time;
    
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
void actionator_t::summaryTo(ostream& output)
{
    output << Name << " ";
    if (!IsAvailable)
    {
        output << "Unavailable" << endl;
        return;
    }
    else
    {
        output << "Available: ";
        for (int i=0; i<m_points.size(); i++)
            output << m_points[i]->Time << " ";
        output << endl;
    }

}

void actionator_t::csvTo(ostream& output)
{
}

ostream& operator<< (ostream& output, const actionator_t& p_actionator)
{
    //! @todo TODO: implement this function
}

istream& operator>> (istream& input, actionator_t& p_actionator)
{
    //! @todo TODO: implement this function
}

