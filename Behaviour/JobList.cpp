/*! @file JobList.cpp
    @brief Implementation of JobList class

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

#include "JobList.h"

/*! @brief JobList constructor
 */
JobList::JobList()
{
    jobs.reserve(100);           // reserve space for 100 jobs
}

/*! @brief Job destructor
 */
JobList::~JobList()
{
}

/*! @brief Append a job to the job list
    
    @param newjob the new job to be added to the list
 */
void JobList::append(Job* newjob)
{
    jobs.push_back(newjob);
}

/*! @brief Clear the job list
 */
void JobList::clear()
{
    jobs.clear();
}


