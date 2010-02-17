/*! @file SaveJob.h
    @brief Declaration of SaveJob class.
 
    @class SaveJob
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

#ifndef SAVEJOB_H
#define SAVEJOB_H

#include "../MotionJob.h"
#include <vector>
using namespace std;

class SaveJob : public MotionJob
{
public:
    SaveJob(double time, const vector<float>& position);
    ~SaveJob();
    
    void setPosition(double time, const vector<float>& newposition);
    void getPosition(double& time, vector<float>& position);
    
    virtual void summaryTo(ostream& output);
    virtual void csvTo(ostream& output);
    
    friend ostream& operator<<(ostream& output, const SaveJob& job);
    friend istream& operator>>(istream& input, SaveJob& job);
private:
    vector<float> m_save_position;                 //!< the save position [x (cm), y (cm), theta (rad)]
};

#endif

