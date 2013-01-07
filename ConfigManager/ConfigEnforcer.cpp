/*! 
	@file 	ConfigEnforcer.cpp
    @brief 	This is the header file for the enforcer of the configuration system for the NUbots.
 
    @class 	ConfigEnforcer
    @brief 	This class ensures all requested changes to the tree adhere to the rules set out for each 
    		constant. Any proposed changes must go through this class to ensure they're valid to be 
    		written to the ConfigStorageManager.

    @author Sophie Calland
 
  Copyright (c) 2012 Sophie Calland
 
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

#include <string>
#include <exception>

#include "ConfigStorageManager.h"
#include "ConfigEnforcer.h"

namespace CONFIGURATION
{
	ConfigEnforcer::ConfigEnforcer()
	{
	}
	
	ConfigEnforcer::~ConfigEnforcer()
	{
		//delete data;
	}
	
	
}
