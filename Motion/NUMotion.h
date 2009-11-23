/*! @file NUMotion.h
    @brief Declaration of motion class
 
    @class NUMotion
    @brief A module to provide motion
 
    If I rename actions to jobs. Each job can have several tasks.
 
    I will have a JobType, and a JobID

    BODYJOB:
        Job: Stand, Kick, Save
        time and position
            KICK: distance and bearing to kick the object
            SAVE: nothing extra, a dive should emerge if a distance needs to be covered in a small amount of time.
 
    HEADJOB:
        Job: Track, Nod, Pan
        time and position
 
    LIGHTJOB:
        Job: Chest, LFoot, RFoot, LEar etc
        time and value
 
    CAMERAJOB:
        Job: Exposure, Gain etc
        time and value
 
    SOUNDJOB:
        Job: Playing, Set, Initial etc
        time
 
    Action
        -- BodyAction
        -- HeadAction
        -- LightAction
        -- CameraAction
        -- SoundAction
        Every action has a
            - Type
            - Job
            - Time
        Some actions have more data
            - a Value or position
            - kick distance and kick bearing
 
    if (action.Type == Action::Body)
        position = action.getPosition()         // get the position where the action is to be performed
        time = action.getTime()               // get the time at which to perform the action
        job = action.getJob()
        
 
TODO: I need to rethink this a little. Each class can not have a different enum to represent JobType!.
 Option 1. Put all job types in Action.
 Option 2. Avoid needing enum types by providing alot of construtors/static functions.
        Action* action = BodyAction::newKickJob(time, position, kicktarget);        // I don't have a problem with this
        action->jobType;    // how do I know if it is a stand, kick or save ??? I need a get method anyway.
 Option 3. I could have a crap load of inheritance.
        Action* action = new KickJob(position, kicktarget);
        still how do I know what type it is!!!!!!!!!
 Option 4. I could use dynamic_cast or typeid. However, this seem to be a bad idea
 Option 5. Just directly access the JobType variable (This doesn't work because I can only compare enums of the same type.
 
 
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

#ifndef NUMOTION_H
#define NUMOTION_H

#include "Behaviour/JobList.h"

class ActuatorCommands
{
public:
    ActuatorCommands() {};
    ~ActuatorCommands() {};
};

class BodyData
{
public:
    BodyData() {};
    ~BodyData() {};
};

class NUMotion
{
public:
    NUMotion();
    ~NUMotion();
    
    ActuatorCommands* process(BodyData* data);
    void process(JobList jobs);
protected:
private:
public:
protected:
private:
};

#endif

