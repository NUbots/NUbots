/*! @file HeadPanJob.h
    @brief Declaration of HeadPanJob class.
 
    @class HeadPanJob
    @brief A class to encapsulate pan jobs issued for the head module. 
 
    I can have NUHead decide how to do the pan, or I can have behaviour decide how to do the pan.
    I think I should provide both sorts:
        So we can simply have a pan type, or we can have a pan type with several parameters.
 
 
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

#ifndef PANHEADJOB_H
#define PANHEADJOB_H

#include "../MotionJob.h"
#include <vector>
using namespace std;

class HeadPanJob : public MotionJob
{
public:
    HeadPanJob(double period, const vector<float>& centre, const vector<float>& limits);
    HeadPanJob(double time, istream& input);
    ~HeadPanJob();
    
    void setPan(double period, const vector<float>& centre, const vector<float>& limits);
    void getPan(double& period, vector<float>& centre, vector<float>& limits);
    
    virtual void summaryTo(ostream& output);
    virtual void csvTo(ostream& output);
    
    friend ostream& operator<<(ostream& output, const HeadPanJob& job);
    friend ostream& operator<<(ostream& output, const HeadPanJob* job);
protected:
    virtual void toStream(ostream& output) const;
private:
    vector<float> m_centre_position;                 //!< the centre position [yaw (rad), pitch (rad), roll (rad)]
    vector<float> m_limit_positions;                 //!< the limit positions for the pan [lower (rad), upper(rad)]
};

#endif

