/*! 
	@file 	ConfigManagerInterface.h
    @brief 	This is the header file for the config manager interface of the configuration system for the 
    		NUbots.
 
    @class 	ConfigManagerInterface
    @brief 	This class communicates with the different modules, pushes current and requested configurations
    		out and accepts new configurations to be written to file.

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

#include "ConfigManager.h"

namespace CONFIGURATION
{

	//This struct stores the name of the parameter to be edited as well as the parameter information.
	//PLACE THIS ONE LEVEL LOWER THAN WHAT IT IS CURRENTLY!!!
	template<typename Item>
	struct named_parameters
	{
		std::string name;
		CONFIGURATION::parameters<Item> data_information;
	};
	
	
	class ConfigManagerInterface
	{
		public:
			ConfigManagerInterface();
			~ConfigManagerInterface();
		
		
		
		
			/*! @brief Retrieves the desired parameter from the specified module and returns as a 
						named_parameters.
	 
	   			@param "path" is the specified path in the tree.
	   			@param "parameter_name" is the name of the parameter to be retrieved.
				@return Returns the desired parameter stored in the parameter struct
	 		*/
	 		template<typename Item>
			CONFIGURATION::named_parameters<Item> readParametersFromModule(std::string path, 
										std::string parameter_name);
		
			/*! @brief Writes the desired parameter to the desired module.
	 
	   			@param "path" is the specified path where the parameter needs to go.
	   			@param "new_parameter" is the new parameter to give to the module, containing newest value 
	   					from tree.
				@return Returns true upon success and false otherwise.
	 		*/														
			template<typename Item>
			bool writeParametersToModule(std::string path, 
										CONFIGURATION::named_parameters<Item> new_parameter);
		
		
		
		
		
			/*! @brief Writes a parameter to store in the current configuration tree.
	 
	   			@param "desired_path" is the desired path of the "new_parameter".
	   			@param "new_parameter" is the new parameter to store in the tree.
				@return Returns true upon success and false otherwise.
	 		*/	
			template<typename Item>
			bool writeParametersToTree(std::string desired_path, 
										CONFIGURATION::named_parameters<Item> new_parameter);
		
			/*! @brief Reads and returns a parameter from the tree with the path specified.
	 
	   			@param "path" is the path where the parameter is stored.
				@return Returns the desired parameter by "named_parameters<Item>".
	 		*/
			template<typename Item>
			CONFIGURATION::named_parameters<Item> readParametersFromTree(std::string path);
		 
	};
}

