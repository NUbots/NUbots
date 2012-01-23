/*! @file NUKick.cpp
    @brief Implementation of nukick class

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

#include "NUKick.h"
#include "NUPlatform/NUPlatform.h"
#include "Infrastructure/NUSensorsData/NUSensorsData.h"
#include "Infrastructure/FieldObjects/FieldObjects.h"
#include "Infrastructure/Jobs/MotionJobs/KickJob.h"
#include "Infrastructure/Jobs/MotionJobs/WalkJob.h"
#include "Motion/Tools/MotionCurves.h"

#include "motionconfig.h"
#include "debugverbositynumotion.h"
#include "Autoconfig/targetconfig.h"

#include "Tools/Math/General.h"
#include "Tools/Math/StlVector.h"
using namespace mathGeneral;

//#if DEBUG_LOCALISATION_VERBOSITY > 0

#include "Kicks/NAOKick.h"
NUKick* NUKick::getKick(NUWalk* walk, NUSensorsData* data, NUActionatorsData* actions)
{
    return new NAOKick(walk, data, actions);
}

NUKick::NUKick(NUWalk* walk, NUSensorsData* data, NUActionatorsData* actions) : NUMotionProvider("NUKick", data, actions)
{
    m_kicking_leg = noLeg;

    m_currentTimestamp = 0;
    m_previousTimestamp = 0;
    loadKickParameters();
}

/*! @brief Destructor for motion module
 */
NUKick::~NUKick()
{
    kill();
}

std::string NUKick::toString(KickingLeg theLeg)
{
    std::string result;
    switch(theLeg)
    {
    case leftLeg:
        result = "Left";
        break;
    case rightLeg:
        result = "Right";
        break;
    default:
        result = "None";
        break;
    }
    return result;
}


void NUKick::loadKickParameters()
{
}

/*! @brief Returns true if the kick is active */
bool NUKick::isActive()
{
    return m_kick_enabled;
}

bool NUKick::isReady()
{
    return m_kick_ready;
}

/*! @brief Returns true if the kick is using the head */
bool NUKick::isUsingHead()
{
    bool usingHead;
    return (isActive() and m_head_enabled);
}

/*! @brief Returns true if the kick is using the arms */
bool NUKick::isUsingArms()
{
    return isActive() and (m_larm_enabled or m_rarm_enabled);
}

/*! @brief Returns true if the kick is using the legs */
bool NUKick::isUsingLegs()
{
    return isActive();
}

/*! @brief Kills the kick module
 */
void NUKick::kill()
{
    #if DEBUG_NUMOTION_VERBOSITY > 3
    debug << "Kick kill called." << endl;
    #endif

    m_kick_enabled = false;
    m_kick_ready = false;
}

void NUKick::stop()
{
    stopHead();
    stopArms();
    stopLegs();
}

void NUKick::stopHead()
{
    setHeadEnabled(false);
    return;
}

void NUKick::stopArms()
{
    setArmEnabled(false, false);
    return;
}

void NUKick::stopLegs()
{   // if another module wants to use the legs, then we should stop
    #if DEBUG_NUMOTION_VERBOSITY > 3
    debug << "Kick stop called." << endl;
    #endif
    // Chose the state that can be transitioned to allowing kick to finish as soon as possible.
}

/*! @brief Sets whether each of the arms can be used by the kick
 */
void NUKick::setArmEnabled(bool leftarm, bool rightarm)
{
    m_larm_enabled = leftarm;
    m_rarm_enabled = rightarm;
    return;
}

/*! @brief Sets whether the head can be used by the kick
 */
void NUKick::setHeadEnabled(bool head)
{
    m_head_enabled = head;
    return;
}


/*! @brief Process new sensor data, and produce actionator commands
 
    @param data a pointer to the most recent sensor data storage class
    @param actions a pointer to the actionators data storage class. 
*/
void NUKick::process(NUSensorsData* data, NUActionatorsData* actions)
{
    if (actions == NULL || data == NULL)
        return;
    #if DEBUG_NUMOTION_VERBOSITY > 3
        debug << "NUKick::process(" << data << ", " << actions << ")" << endl;
        debug << "NUKick::process in " << toString(pose) << " ready: " << m_kickReady << " active: " << m_kickActive << endl;
    #endif
    
    m_data = data;
    m_actions = actions;
    m_previousTimestamp = m_currentTimestamp;
    m_currentTimestamp = data->CurrentTime;

    if(!isActive() && !isReady())
        return;
    doKick();
}

/*! @brief Process a kick job
    @param job the kick job
 */
void NUKick::process(KickJob* job)
{
    #if DEBUG_NUMOTION_VERBOSITY > 3
    debug << "void NUKick::process(KickJob* job)" << endl;
    #endif
    double time;
    vector<float> kickposition;
    vector<float> kicktarget;
    job->getKick(time, kickposition, kicktarget);
    /*
    if(kickposition[0] == 1.0 && kickposition[1] == 2.0 && kicktarget[0] == 3.0f && kicktarget[1] == 4.0f)
        m_pauseState = !m_pauseState;
    else if(kickposition[0] == 1.0 && kickposition[1] == 1.0 && kicktarget[0] == 1.0f && kicktarget[1] == 1.0f)
    {
        m_variableGainValue += 0.001;
        #if DEBUG_NUMOTION_VERBOSITY > 3
        debug << "Special kick command: increasing variable gain to " << m_variableGainValue << endl;
        #endif
    }
    else if(kickposition[0] == -1.0 && kickposition[1] == -1.0 && kicktarget[0] == -1.0f && kicktarget[1] == -1.0f)
    {
        m_variableGainValue -= 0.001;
        #if DEBUG_NUMOTION_VERBOSITY > 3
        debug << "Special kick command: decreasing variable gain to " << m_variableGainValue << endl;
        #endif
    }*/
    kickToPoint(kickposition, kicktarget);
}


void NUKick::kickToPoint(const vector<float>& position, const vector<float>& target)
{
    // Evaluate the kicking target to start a new kick, or change a kick in progress.
    m_ball_x = position[0];
    m_ball_y = position[1];

    m_target_x = target[0];
    m_target_y = target[1];

#if DEBUG_NUMOTION_VERBOSITY > 4
    debug << "void NAOKick::kickToPoint( (" << position[0] << "," << position[1] << "),(" << target[0] << "," << target[1] << ") )" << endl;
#endif

}
