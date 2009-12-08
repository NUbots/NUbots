/*! @file JobList.h
    @brief Declaration of JobList class.
 
    @class JobList
    @brief A class containing the list of jobs to be done by modules
 
    At the moment this is a VERY thin wrapper for a vector of jobs. But this WILL need to
    change in the future because I think we will want to insert/delete to/from any position
    and we might might to implement some form of priorities.

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

#ifndef JOBLIST_H
#define JOBLIST_H

#include "Job.h"

#include <vector>
using namespace std;

class JobList
{
public:
    JobList();
    ~JobList();
    
    void append(Job* newjob);
    void clear();
    int size();

private:
    vector<Job*> jobs;                          //!< the list of jobs for subsequent modules
};

#endif

