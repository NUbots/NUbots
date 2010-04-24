/*! @file HeadPanJob.h
    @brief Declaration of HeadPanJob class.
 
    @class HeadPanJob
    @brief A class to encapsulate pan jobs issued for the head module. 
 
    There are three types of pans:
        - Ball --- a pan to just look for the ball
        - BallAndLocalisation --- a pan to both look for the ball and to passively localise
        - Localisation -- a pan just for localisation
 
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
    enum head_pan_t
    {
        Ball,
        BallAndLocalisation,
        Localisation
    };
public:
    HeadPanJob(head_pan_t pantype);
    HeadPanJob(istream& input);
    ~HeadPanJob();
    
    head_pan_t getPanType();
    
    virtual void summaryTo(ostream& output);
    virtual void csvTo(ostream& output);
    
    friend ostream& operator<<(ostream& output, const HeadPanJob& job);
    friend ostream& operator<<(ostream& output, const HeadPanJob* job);
protected:
    virtual void toStream(ostream& output) const;
private:
    head_pan_t m_pan_type;
};

#endif

