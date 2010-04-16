/*! @file NUHead.h
    @brief Declaration of nuhead class
 
    @class NUHead
    @brief A module to provide head
 
    @author Jason Kulk, Jed Rietveld
 
  Copyright (c) 2010 Jason Kulk, Jed Rietveld
 
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

#ifndef NUHEAD_H
#define NUHEAD_H

class NUSensorsData;
class NUActionatorsData;
class PIDController;

#include <vector>

class NUHead
{
public:
    NUHead();
    ~NUHead();
    
    void process(NUSensorsData* data, NUActionatorsData* actions);
    void moveTo(const std::vector<double>& times, const std::vector<std::vector<float> >& positions);
private:
    void doHead();

private:
    NUSensorsData* m_data;              //!< local pointer to the latest sensor data
    NUActionatorsData* m_actions;       //!< local pointer to the next actionators data
    
	double m_pitch;                     //!< current pitch target
    double m_yaw;                       //!< current yaw target

	PIDController* m_pitch_pid;         //!< low level PID controller for the pitch
    PIDController* m_yaw_pid;           //!< low level PID controller for the yaw
    
    double m_head_timestamp;

};

#endif

