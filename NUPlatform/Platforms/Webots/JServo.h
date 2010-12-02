/*! @file JServo.h
    @brief Declaration of a JServo (A slightly extended version of Webots' Servo) class
    @author Jason Kulk
 
    @class JServo
    @brief A JServo (A slightly extended version of Webots' Servo) class
 
    In particular, the JServo provides get methods for the servo targets,
    get methods for the maximum velocity and force, and get methods for the
    joint limits.
 
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

#ifndef JSERVO_H
#define JSERVO_H

#include <webots/Servo.hpp>

namespace webots 
{
    class JServo : public Servo 
    {
    public:
        JServo(const std::string &name);
        virtual ~JServo();
        
        virtual void setAcceleration(double accel);
        virtual void setVelocity(double vel);
        virtual void setPosition(double position);
        virtual void setForce(double force);
        virtual void setGain(double gain);
        virtual void setMaxForce(double maxforce);
        
        double getTargetAcceleration() const;
        double getTargetVelocity() const;
        double getTargetPosition() const;
        double getTargetForce() const;
        double getTargetGain() const;
        
        double getMaxVelocity() const;
        double getMaxForce() const;
    private:
        double m_target_acceleration;
        double m_target_velocity;
        double m_target_position;
        double m_target_force;
        double m_target_gain;
        double m_target_max_force;
        
        double m_max_velocity;
        double m_max_force;
    };
}

#endif

