/*! @file NUMotion.cpp
    @brief Implementation of motion class

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

#include <iostream>
using namespace std;

#include "NUPlatform/NUPlatform.h"
#include "NUMotion.h"

/*! @brief Constructor for motion module
 */
NUMotion::NUMotion()
{
    cout << "NUMotion::NUMotion" << endl;
}

/*! @brief Destructor for motion module
 */
NUMotion::~NUMotion()
{
}

/*! @brief Process new body sensor data, and action commands
 */
ActuatorCommands* NUMotion::process(BodyData* data)
{
    ActuatorCommands* temp = new ActuatorCommands();
    return temp;
}

/*! @brief Process new body sensor data, and action commands
 */
void NUMotion::process(vector<Job*> jobs)
{
    cout << "NUMotion::process():" << endl;
    job_type_t jobtype;
    for (int i=0; i<jobs.size(); i++)
    {
        jobtype = jobs[i]->getJobType();
        if (jobtype == BODY)
            cout << "This job is body" << endl;
        else if (jobtype == HEAD)
            cout << "This job is head" << endl;
    }
}

