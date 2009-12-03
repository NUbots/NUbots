/*! @file actionator_t.cpp
    @brief Implementation of a single actionator class
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


/*! @brief Default constructor for a actionator_t. Initialises the actionator to be undefined and unavailable
 */
actionator_t::actionator_t()
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
actionator_t::actionator_t(string actionatorname, actionator_type_t actionatortype)
{
    Name = actionatorname;
    ActionatorType = actionatortype;
    IsAvailable = true;
    m_previous_point = NULL;
}

/*! @brief Provides a text summary of the contents of the actionator_t
 
 The idea is to use this function when writing to a debug log. I guarentee that the 
 output will be human readable.
 
 @param output the ostream in which to put the string
 */
void actionator_t::summaryTo(ostream& output)
{
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

