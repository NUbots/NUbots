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
#include "debug.h"

#include <math.h>
#include <string>

JServo::JServo(const std::string &name) : Servo(name)
{
    m_target_acceleration = -1;         // according to webots documentation -1 is default
    m_target_velocity = 10;
    m_target_position = 0;
    m_target_force = 0;
    m_target_gain = 10;                 // according to webots documentation 10 is default
    
    // There is nothing in the Webots API to make this easy, so Ill have to do it the hard way
    if (name.compare("HeadYaw") == 0 || name.compare("LShoulderPitch") == 0 || name.compare("RShoulderPitch") == 0 || name.compare("LElbowYaw") == 0 || name.compare("RElbowYaw") == 0)
    {
        m_max_velocity = 8.25;
        m_max_force = 2.26;
    }
    else if (name.compare("HeadPitch") == 0 || name.compare("LShoulderRoll") == 0 || name.compare("RShoulderRoll") == 0 || name.compare("LElbowRoll") == 0 || name.compare("RElbowRoll") == 0)
    {
        m_max_velocity = 7.18;
        m_max_force = 2.60;
    }
    else if (name.compare("LHipPitch") == 0 || name.compare("RHipPitch") == 0 || name.compare("LKneePitch") == 0 || name.compare("RKneePitch") == 0 || name.compare("LAnklePitch") == 0 || name.compare("RAnklePitch") == 0)
    {
        m_max_velocity = 6.39;
        m_max_force = 7.77;
    }
    else
    {
        m_max_velocity = 4.15;
        m_max_force = 11.96;
    }
    m_target_max_force = m_max_force;
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
    vel = fabs(vel);
    if (vel > m_max_velocity)
        vel = m_max_velocity;
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
    @param force the servo force in Nm
 */
void JServo::setForce(double force)
{
    m_target_force = force;
    Servo::setForce(force);
}

/*! @brief Sets the servo's gain and saturation
    @param gain the % (0 to 100) of the maximum gain
 */
void JServo::setGain(double gain)
{
    if (gain < 10)
        gain = 10;
    else if (gain > 100)
        gain = 100;
    m_target_gain = gain;
    Servo::setControlP(gain*0.1);
    setMaxForce((gain/100.0)*m_max_force);
}

/*! @brief Sets the maximum motor force in Nm
    @param p the the proportional gain
 */
void JServo::setMaxForce(double maxforce)
{
    m_target_max_force = maxforce;
    Servo::setMotorForce(maxforce);
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
    float currentposition = getPosition();
    if (fabs(currentposition - m_target_position) < 0.001)
        return m_target_position;
    else
        return currentposition + m_target_velocity*0.04;
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

/*! Returns the servo's maximum velocity. 
 This is important because webots chooses to do nothing instead of getting the motor to move as fast as possible
 */
double JServo::getMaxVelocity() const
{
    return m_max_velocity;
}

/*! Returns the servo's maximum force (torque in Nm). 
 This is important because webots chooses to do nothing instead of getting the motor to move with as much force as possible
 */
double JServo::getMaxForce() const
{
    return m_max_force;
}
