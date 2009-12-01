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
    m_points.push_back(point);
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

