/*! @file JServo.cpp
    @brief Implementation of a JServo (A slightly extended version of Webots' Servo) class
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

#include "JServo.h"
#include "Tools/debug.h"

JServo::JServo(const std::string &name) : Servo(name)
{
    m_target_acceleration = -1;         // according to webots documentation -1 is default
    m_target_velocity = 10;
    m_target_position = 0;
    m_target_force = 0;
    m_target_gain = 10;          // according to webots documentation 10 is default
}

JServo::~JServo()
{
}

/*! @brief Sets the acceleration limit for the servo
    @param accel the new acceleration limit, set it to -1 for there to be no limit (rad/s/s)
 */
void JServo::setAcceleration(double accel)
{
    m_target_acceleration = accel;
    Servo::setAcceleration(accel);
}

/*! @brief Sets the servo velocity
    @param vel the velocity (rad/s)
 */
void JServo::setVelocity(double vel)
{
    m_target_velocity = vel;
    Servo::setVelocity(vel);
}

/*! @brief Sets the servo position
    @param position the servo position (radians)
 */
void JServo::setPosition(double position)
{
    m_target_position = position;
    Servo::setPosition(position);
}

/*! @brief Sets the servo force
    @param force the serov force in Nm
 */
void JServo::setForce(double force)
{
    m_target_force = force;
    Servo::setForce(force);
}

/*! @brief Sets the proportional gain of servo's position control
    @param p the the proportional gain
 */
void JServo::setControlP(double p)
{
    m_target_gain = p;
    Servo::setControlP(p);
}

/*! Returns the current target acceleration
 */
double JServo::getTargetAcceleration() const
{
    return m_target_acceleration;
}

/*! Returns the current target velocity
 */
double JServo::getTargetVelocity() const
{
    return m_target_velocity;
}

/*! Returns the servo's current target position
 */
double JServo::getTargetPosition() const
{
    return m_target_position;
}

/*! Returns the servo's current target force
 */
double JServo::getTargetForce() const
{
    return m_target_force;
}

/*! Returns the servo's proportional gain
 */
double JServo::getTargetGain() const
{
    return m_target_gain;
}
