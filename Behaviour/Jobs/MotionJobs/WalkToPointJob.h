/*! @file WalkToPointJob.h
    @brief Declaration of WalkToPointJob class.
 
    @class WalkToPointJob
    @brief A class to encapsulate jobs issued for the walk module.
 
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

#ifndef WALKTOPOINTJOB_H
#define WALKTOPOINTJOB_H

#include "../MotionJob.h"
#include <vector>
using namespace std;

class WalkToPointJob : public MotionJob
{
public:
    WalkToPointJob(double time, const vector<float>& position);
    ~WalkToPointJob();
    
    void setPosition(double time, const vector<float>& newposition);
    void getPosition(double& time, vector<float>& position);
    
    virtual void summaryTo(ostream& output);
    virtual void csvTo(ostream& output);
    
    virtual ostream& operator<< (ostream& output);
    virtual istream& operator>> (istream& input);
private:
    vector<float> m_walk_position;                 //!< the walk position x (cm), y (cm) and theta (rad)
};

#endif

