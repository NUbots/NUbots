/*! @file NUKick.h
    @brief Declaration of nuhead class
 
    @class NUKick
    @brief A module to provide head
 
    @author Jed Rietveld
 
  Copyright (c) 2010 Jed Rietveld
 
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

#include "NUPlatform/NUSensors/NUSensorsData.h"
#include "NUPlatform/NUActionators/NUActionatorsData.h"

class PIDController;

class NUHead
{
public:
    NUHead();
    ~NUHead();
    
    void process(NUSensorsData* data, NUActionatorsData* actions);
    void process(const vector<float>& position);
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

