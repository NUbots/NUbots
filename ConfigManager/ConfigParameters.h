/*! 
	@file ConfigParameters.h
    @brief This is the header file for the ConfigParameters objects of the configuration system for the 
    NUbots.
 
    @class ConfigParameters
    @brief This class serves as an object for use when transferring parameters.

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

#ifndef ConfigParameters_def
#define ConfigParameters_def

#include "ConfigConflicts.h"


	
namespace ConfigSystem
{
	template<typename Item>	
	class ConfigParameters
	{
		public:
			ConfigParameters();
			ConfigParameters(const Item &value_init, const std::string &type_init, const Item &upper_init,
							const Item &lower_init, const vector<Item> &possible_values_init,
							const ConfigConflicts &conflicts_init);
			~ConfigParameters();
			
			
			
			
			void setValue(const Item);
			void setType(const std::string&);
			
			void setUpperBound(const Item);
			void setLowerBound(const Item);
			
			void setPossibleValues(const vector<Item>);
			void setConflicts(const ConfigConflicts &);
			
			
			
			
			
			Item getValue();
			std::string getType();
			
			Item getUpperBound();
			Item getLowerBound();
			
			vector<Item> getPossibleValues();
			ConfigConflict* getConflicts();
			const ConfigConflict* getConflicts();
			
		private:
			Item value;
			std::string type;
		
			Item upper_bound;
			Item lower_bound;
		
			std::vector<Item> possible_values;
		
			//ConfigConflicts *conflicts; //object stores paths etc.
	};
}
