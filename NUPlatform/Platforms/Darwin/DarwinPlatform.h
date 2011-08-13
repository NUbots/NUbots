/*! @file DarwinPlatform.h
    @brief Declaration of Darwin platform class.

    @author Jason Kulk
 
  Copyright (c) 2011 Jason Kulk
 
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

#ifndef DARWINPLATFORM_H
#define DARWINPLATFORM_H

#include "NUPlatform/NUPlatform.h"

//From Darwin Library:
#include <LinuxCM730.h>	//Darwin Controller
#include <MX28.h>		//Darwin Motors
#include <JointData.h>

class DarwinPlatform : public NUPlatform
{
public:
    DarwinPlatform();
    ~DarwinPlatform();

    vector<string> 	m_servo_names;          //!< the names of the available joints (eg HeadYaw, AnklePitch etc) in the Darwin-OP robot
	vector<int> 	m_servo_IDs;			//!< Corresponding servo ids to names
	vector<float> 	m_servo_Offsets;
protected:


private:
	Robot::LinuxCM730* linux_cm730;									//!< Darwin Subcontrolller connection
	Robot::CM730* cm730;
};

#endif
