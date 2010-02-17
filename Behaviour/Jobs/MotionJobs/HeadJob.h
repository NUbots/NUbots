/*! @file HeadJob.h
    @brief Declaration of HeadJob class.
 
    @class HeadJob
    @brief A class to encapsulate jobs issued for the head module. This particular job just moves the 
           head to the given angles.
 
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

#ifndef HEADJOB_H
#define HEADJOB_H

#include "../MotionJob.h"
#include <vector>
using namespace std;

class HeadJob : public MotionJob
{
public:
    HeadJob(double time, const vector<float>& position);
    ~HeadJob();
    
    void setPosition(double time, const vector<float>& newposition);
    void getPosition(double& time, vector<float>& position);
    
    virtual void summaryTo(ostream& output);
    virtual void csvTo(ostream& output);
    
    virtual ostream& operator<< (ostream& output);
    virtual istream& operator>> (istream& input);
private:
    vector<float> m_head_position;                 //!< the head position [yaw (rad), pitch (rad), roll (rad)]
};

#endif

