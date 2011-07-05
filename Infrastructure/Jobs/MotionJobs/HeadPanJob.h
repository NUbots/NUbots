/*! @file HeadPanJob.h
    @brief Declaration of HeadPanJob class.
 
    @class HeadPanJob
    @brief A class to encapsulate pan jobs issued for the head module. 
 
    There are three types of pans:
        - Ball --- a pan to just look for the ball
        - BallAndLocalisation --- a pan to both look for the ball and to passively localise
        - Localisation -- a pan just for localisation
    The type specifies (a) default pitch and yaw limits (b) the speed.
 
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
class MobileObject;
class StationaryObject;

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
    HeadPanJob(head_pan_t pantype, float xmin, float xmax, float yawmin, float yawmax);
    HeadPanJob(const MobileObject& object);
    HeadPanJob(const StationaryObject& object);
    HeadPanJob(const vector<StationaryObject>& objects);
    HeadPanJob(istream& input);
    ~HeadPanJob();
    
    head_pan_t getPanType();
    bool useDefaultValues();
    void getX(float& xmin, float& xmax);
    void getYaw(float& yawmin, float& yawmax);
    
    void summaryTo(ostream& output);
    void csvTo(ostream& output);
    
    friend ostream& operator<<(ostream& output, const HeadPanJob& job);
    friend ostream& operator<<(ostream& output, const HeadPanJob* job);
protected:
    virtual void toStream(ostream& output) const;
private:
    head_pan_t m_pan_type;              //!< the type of pan
    bool m_use_default;                 //!< true if the head should use the default values
    float m_x_min;                      //!< the minimum x distance to include in the pan (cm)
    float m_x_max;                      //!< the maximum x distance to include in the pan (cm)
    float m_yaw_min;                    //!< the minimum (right) yaw angle to include in the pan (rad)
    float m_yaw_max;                    //!< the maximum (left) yaw angle to include in the pan (rad)
};

#endif

