/*! @file NodHeadJob.h
    @brief Declaration of NodHeadJob class.
 
    @class NodHeadJob
    @brief A class to encapsulate jobs issued for the head module. This particular job nods the head 
           up and down at the given centre to the given limits
 
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

#ifndef NODHEADJOB_H
#define NODHEADJOB_H

#include "../MotionJob.h"
#include <vector>
using namespace std;

class NodHeadJob : public MotionJob
{
public:
    NodHeadJob(double period, const vector<float>& centre, const vector<float>& limits);
    NodHeadJob(double time, istream& input);
    ~NodHeadJob();
    
    void setNod(double period, const vector<float>& centre, const vector<float>& limits);
    void getNod(double& period, vector<float>& centre, vector<float>& limits);
    
    virtual void summaryTo(ostream& output);
    virtual void csvTo(ostream& output);
    
    friend ostream& operator<<(ostream& output, const NodHeadJob& job);
    friend ostream& operator<<(ostream& output, const NodHeadJob* job);
protected:
    virtual void toStream(ostream& output) const;
private:
    vector<float> m_centre_position;                 //!< the centre position [yaw (rad), pitch (rad), roll (rad)]
    vector<float> m_limit_positions;                 //!< the limit positions for the nod [lower (rad), upper(rad)]
};

#endif

