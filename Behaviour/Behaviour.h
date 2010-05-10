/*! @file Behaviour.h
    @brief Declaration of behaviour class

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

#ifndef BEHAVIOUR_H
#define BEHAVIOUR_H

#include "Behaviour/Jobs.h"
#include "Vision/FieldObjects/FieldObjects.h"
#include "NUPlatform/NUSensors/NUSensorsData.h"

class Behaviour
{
public:
    Behaviour();
    ~Behaviour();
    
    void processFieldObjects(JobList* jobs, FieldObjects* AllObjects, NUSensorsData* data);
    void TrackPoint(float sensoryaw, float sensorpitch, float elevation, float bearing, float centreelevation = 0, float centrebearing = 0);
    void Pan();

private:
    JobList* m_jobs;
};


#endif

