/*! @file JRobot.cpp
    @brief Implementation of JRobot class
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

#include "JRobot.h"
#include "JServo.h"
#include "Tools/debug.h"

JRobot::JRobot() : Robot()
{
}

JRobot::~JRobot()
{
}

Servo* JRobot::createServo(const std::string &name) const
{
    debug << "JRobot::createServo" << endl;
    Servo* servo = new JServo(name);
    return servo;
}
