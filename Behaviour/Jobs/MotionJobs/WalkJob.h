/*! @file WalkJob.h
    @brief Declaration of WalkJob class.
 
    @class WalkJob
    @brief A base class to encapsulate jobs issued for the walk module.
 
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

#ifndef WALKJOB_H
#define WALKJOB_H

#include "../MotionJob.h"
#include <vector>
using namespace std;

class WalkJob : public MotionJob
{
public:
    WalkJob(const vector<float>& speed);
    ~WalkJob();
    
    void setSpeed(const vector<float>& newspeed);
    void getSpeed(vector<float>& speed);
    
    virtual void summaryTo(ostream& output);
    virtual void csvTo(ostream& output);
    
    virtual ostream& operator<< (ostream& output);
    virtual istream& operator>> (istream& input);
private:
    vector<float> m_walk_speed;                 //!< the walk speed x (cm/s), y (cm/s) and theta (rad/s)
};

#endif

