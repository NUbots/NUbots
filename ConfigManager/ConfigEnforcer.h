/*! 
	@file 	ConfigEnforcer.h
    @brief 	This is the header file for the enforcer of the configuration system for the NUbots.
 
    @class 	ConfigEnforcer
    @brief 	This class ensures all requested changes to the tree adhere to the rules set out for each 
    		constant. Any proposed changes must go through this class to ensure they're valid to be 
    		written to the ConfigStorageManager.
    		
    		Template functions are implemented in a separate ConfigEnforcer.template file.

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

#ifndef ConfigEnforcer_def
#define ConfigEnforcer_def

#include <string>
#include <exception>

#include "ConfigStorageManager.h"

using ConfigSystem::ConfigStorageManager;

namespace ConfigSystem
{
	class ConfigEnforcer
	{
		public:
			ConfigEnforcer();
			~ConfigEnforcer();
			
			
			
			/*! @brief Checks the range of the proposed value change.
 
   			@param "proposed_change" is the proposed change to write to file.
    		@return Returns "true" if range is not violated, "false" if invalid.
 			*/
 			template<typename Item>
			bool checkRangeValue(parameters<Item> proposed_change);
						
			/*! @brief Ensures all values in possible_value vector to be changed comply with range
						requirements. 
 
   			@param "proposed_change" is the proposed change to write to file.
    		@return Returns "true" if range is not violated, "false" if invalid.
 			*/
			template<typename Item>
			bool checkRangePossibleValues(parameters<Item> &proposed_change);
			
			
			
			
			//WILL REQUIRE MORE FUNCTIONS FOR CERTAIN NEWER RULES THAT MIGHT ARISE.
					
		private:
			//Some more functions maybe?
		
	};
}

#include "ConfigEnforcer.template"
#endif //ConfigEnforcer_def
