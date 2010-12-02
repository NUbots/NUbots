/*! @file WalkPerturbationJob.h
    @brief Declaration of WalkPerturbationJob class.
 
    @class WalkPerturbationJob
    @brief A job to to remotely perturb/push the robot while it is walking
 
    @author Jason Kulk
 
  Copyright (c) 2010 Jason Kulk
 
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

#ifndef WALK_PERTURBATION_JOB_H
#define WALK_PERTURBATION_JOB_H

#include "../MotionJob.h"

#include <vector>
using namespace std;

class WalkPerturbationJob : public MotionJob
{
public:
    WalkPerturbationJob(float magnitude, float direction);
    WalkPerturbationJob(istream& input);
    ~WalkPerturbationJob();
    
    float getMagnitude();
    float getDirection();
    
    virtual void summaryTo(ostream& output);
    virtual void csvTo(ostream& output);
    
    friend ostream& operator<<(ostream& output, const WalkPerturbationJob& job);
    friend ostream& operator<<(ostream& output, const WalkPerturbationJob* job);
protected:
    virtual void toStream(ostream& output) const;
private:
    float m_magnitude;                  //!< the magnitude of the perturbation (0 to 100)
    float m_direction;                  //!< the direction to perturbed the robot in radians
};

#endif

