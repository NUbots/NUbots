/*! @file WalkJob.h
    @brief Declaration of WalkJob class.
 
    @class WalkJob
    @brief A base class to encapsulate jobs issued for the walk module.
 
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

#ifndef WALKJOB_H
#define WALKJOB_H

#include "../MotionJob.h"
#include <vector>
using namespace std;

class WalkJob : public MotionJob
{
public:
    WalkJob(float x, float y, float yaw);
    WalkJob(const vector<float>& speed);
    WalkJob(istream& input);
    ~WalkJob();
    
    float getTranslationSpeed();
    float getDirection();
    float getRotationSpeed();
    
    virtual void summaryTo(ostream& output);
    virtual void csvTo(ostream& output);
    
    friend ostream& operator<<(ostream& output, const WalkJob& job);
    friend ostream& operator<<(ostream& output, const WalkJob* job);
protected:
    virtual void toStream(ostream& output) const;
private:
    float m_translation_speed;          //!< the translational speed between 0 and 1
    float m_direction;                  //!< the translational direction of the walk    
    float m_rotation_speed;             //!< the rotational speed in rad/s
};

#endif

