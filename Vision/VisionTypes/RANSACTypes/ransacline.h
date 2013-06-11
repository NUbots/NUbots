/*! @file HeadTrackJob.h
    @brief Declaration of HeadTrackJob class.
 
    @class HeadTrackJob
    @brief A class to encapsulate jobs issued for the head module. This particular job is designed
           to track a visual object
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

#ifndef HEADTRACKJOB_H
#define HEADTRACKJOB_H

#include "../MotionJob.h"

class Object;

#include <vector>


class HeadTrackJob : public MotionJob
{
public:
    HeadTrackJob(const Object& object, float centreelevation = 0, float centrebearing = 0);
    HeadTrackJob(float elevation, float bearing, float centreelevation = 0, float centrebearing = 0);
    HeadTrackJob(std::istream& input);
    ~HeadTrackJob();
    
    void getData(float& elevation, float& bearing, float& centreelevation, float& centrebearing);
    
    virtual void summaryTo(std::ostream& output);
    virtual void csvTo(std::ostream& output);
    
    friend std::ostream& operator<<(std::ostream& output, const HeadTrackJob& job);
    friend std::ostream& operator<<(std::ostream& output, const HeadTrackJob* job);
protected:
    virtual void toStream(std::ostream& output) const;
private:
    float m_elevation;
    float m_bearing;
    float m_centre_elevation;
    float m_centre_bearing;
};

#endif

