/*! @file HeadNodJob.h
    @brief Declaration of HeadNodJob class.
 
    @class HeadNodJob
    @brief A class to encapsulate jobs head nods.
 
    There are three types of nods
        - Ball
        - BallAndLocalisation
        - Localisation
 
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


class HeadNodJob : public MotionJob
{
public:
    enum head_nod_t
    {
        Ball,
        BallAndLocalisation,
        Localisation
    };
public:
    HeadNodJob(head_nod_t nodtype, float centreangle = 0);
    HeadNodJob(std::istream& input);
    ~HeadNodJob();
    
    head_nod_t getNodType();
    float getCentreAngle();
    
    virtual void summaryTo(std::ostream& output);
    virtual void csvTo(std::ostream& output);
    
    friend std::ostream& operator<<(std::ostream& output, const HeadNodJob& job);
    friend std::ostream& operator<<(std::ostream& output, const HeadNodJob* job);
protected:
    virtual void toStream(std::ostream& output) const;
private:
    head_nod_t m_nod_type;
    float m_centre_angle;
};

#endif

