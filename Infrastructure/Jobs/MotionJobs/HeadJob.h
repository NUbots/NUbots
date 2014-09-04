/*! @file HeadJob.h
    @brief Declaration of HeadJob class.
 
    @class HeadJob
    @brief A class to encapsulate jobs issued for the head module. This particular job just moves the 
           head to the given angles.
 
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

#ifndef HEADJOB_H
#define HEADJOB_H

#include "../MotionJob.h"
#include <vector>


class HeadJob : public MotionJob
{
public:
    HeadJob(double time, const std::vector<float>& position);
    HeadJob(const std::vector<double>& times, const std::vector<std::vector<float> >& positions); 
    HeadJob(double time, std::istream& input);
    ~HeadJob();
    
    void setPosition(double time, const std::vector<float>& newposition);
    void setPositions(const std::vector<double>& times, const std::vector<std::vector<float> >& positions);
    void getPositions(std::vector<double>& times, std::vector<std::vector<float> >& positions);
    
    virtual void summaryTo(std::ostream& output);
    virtual void csvTo(std::ostream& output);
    
    friend std::ostream& operator<<(std::ostream& output, const HeadJob& job);
    friend std::ostream& operator<<(std::ostream& output, const HeadJob* job);
protected:
    virtual void toStream(std::ostream& output) const;
private:
    std::vector<double> m_times;                                 //!< the times for each head position in the sequence
    std::vector<std::vector<float> > m_head_positions;                //!< the head position [[roll0, pitch0, yaw0], [roll1, pitch1, yaw1], ... ,[rollN, pitchN, yawN]]
    unsigned int m_size;
    unsigned int m_width;
};

#endif

