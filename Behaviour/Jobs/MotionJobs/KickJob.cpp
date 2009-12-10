/*! @file KickJob.cpp
    @brief Partial implementation of KickJob class

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

#include "KickJob.h"

KickJob::KickJob(double time, const vector<float>& kickposition, const vector<float>& kicktarget) : MotionJob(Job::MOTION_WALK)
{
    m_job_time = time;
    m_kick_position = kickposition;
    m_kick_target = kicktarget;         
}

/*! @brief KickJob destructor
 */
KickJob::~KickJob()
{
    m_kick_position.clear();
    m_kick_target.clear();
}

/*! @brief Sets the kick time, position and target
    @param time the time you want to kick
    @param kickposition the point you want to kick (hopefully the position of the ball) [x(cm), y(cm)]
    @param kicktarget the point you want to kick the ball at (probably the goal) from your current position [x(cm), y(cm)]
 */
void KickJob::setKick(double time, const vector<float>& kickposition, const vector<float>& kicktarget)
{
    m_job_time = time;
    m_kick_position = kickposition;
    m_kick_target = kicktarget;
}

/*! @brief Sets the kick time and position
    @param time the time you want to kick
    @param kickposition the point you want to kick (hopefully the position of the ball) [x(cm), y(cm)]
 */
void KickJob::setKickPosition(double time, const vector<float>& kickposition)
{
    m_job_time = time;
    m_kick_position = kickposition;
}

/*! @brief Sets the kick target
    @param kicktarget the point you want to kick the ball at (probably the goal) from your current position [x(cm), y(cm)]
 */
void KickJob::setKickTarget(const vector<float>& kicktarget)
{
    m_kick_target = kicktarget;
}

/*! @brief Gets the kick time, position and target
    @param time the time you want to kick
    @param kickposition the point you want to kick (hopefully the position of the ball) [x(cm), y(cm)]
    @param kicktarget the point you want to kick the ball at (probably the goal) from your current position [x(cm), y(cm)]
 */
void KickJob::getKick(double& time, vector<float>& kickposition, vector<float>& kicktarget)
{
    time = m_job_time;
    kickposition = m_kick_position;
    kicktarget = m_kick_target;
}

/*! @brief Gets the kick time and position
    @param time the time you want to kick
    @param kickposition the point you want to kick (hopefully the position of the ball) [x(cm), y(cm)]
 */
void KickJob::getKickPosition(double& time, vector<float>& kickposition)
{
    time = m_job_time;
    kickposition = m_kick_position;
}

/*! @brief Gets the kick target
    @param kicktarget the point you want to kick the ball at (probably the goal) from your current position [x(cm), y(cm)]
 */
void KickJob::getKickTarget(vector<float>& kicktarget)
{
    kicktarget = m_kick_target;
}
