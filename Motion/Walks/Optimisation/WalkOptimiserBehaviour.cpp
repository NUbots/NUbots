/*! @file WalkOptimiserBehaviour.cpp
    @brief Implementation of WalkOptimiserBehaviour class

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

#include "WalkOptimiserBehaviour.h"

#include "NUPlatform/NUSystem.h"
#include "Tools/debug.h"

#include <math.h>

//! @todo TODO: put M_PI and NORMALISE somewhere else
#ifndef M_PI
    #define M_PI 3.1415926535
#endif

template <class T>
inline T NORMALISE(T theta){
    return atan2(sin(theta), cos(theta));
}

WalkOptimiserBehaviour::WalkOptimiserBehaviour()
{
    
}

WalkOptimiserBehaviour::~WalkOptimiserBehaviour()
{
    
}


void WalkOptimiserBehaviour::process(NUSensorsData* data, NUActionatorsData* actions, JobList& joblist)
{   
    if (data == NULL || actions == NULL)
        return;
    else
    {
        m_data = data;
        m_actions = actions;
    }
    
    static vector<float> fallen;
    m_data->
    
    if isFallen()
        optimiser->getNewParameters(m_walk_parameters);
    
        tick optimiser?. Not really because there is no new metric.
        it should just be a getNewParameters
    // so i need to add jobs to the job list to get what I want done.
    // I need to add a 'respawn job', I am not going to bother with a nice interface with this one because I need to have it written today!
    // if we have been using this parameter set for 10 seconds
    // I also need to make sure I get data from the same point in each gait
    //      then tick the optimiser
    //      start from the respawn location.
    // else add no jobs
}

bool WalkOptimiserBehaviour::isFallen()
{
    static vector<float> fallen;
    if (m_data->getFallen(fallen) && fallen[0] > 0)
        return true;
    else
        return false;
}

