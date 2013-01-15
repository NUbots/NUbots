/*! 
	@file 	ConfigTree.h
    @brief 	Header file for the ConfigTree class. 
 
    @class 	ConfigTree
    @brief 	Stores and converts the strings stored in the ConfigStorageManager tree to ConfigParameter
    		Objects and stores in its own property tree. 

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

#ifndef ConfigTree_def
#define ConfigTree_def

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/foreach.hpp>
#include <string>
#include <exception>

#include "ConfigStorageManager.h"

using std::string;
using ConfigSystem::ConfigStorageManager;

namespace ConfigSystem
{
	class ConfigTree
	{
		public:
			/*! @brief 	Constructor.
 
   			@param		N/A
    		@return 	Creates a new ConfigTree object.
 			*/
			ConfigTree();
		
			/*! @brief 	Constructor.
 
   			@param		String array "filename_arr" stores the config filenames to be read from.
    		@return 	Creates a new ConfigTree object and retrieves data from the filenames.
 			*/
			ConfigTree(const string &filename_arr[]);
			
			/*! @brief 	Destructor. 
 
   			@param		N/A
    		@return 	Deletes a ConfigTree object.
 			*/
			~ConfigTree();
			
			
			
			
			
						
			/*! @brief 	Retrieves a parameter stored in the data_access tree. 
 
   			@param		String "path" defines what path the parameter is going to be retrieved from
    		@return 	Returns a pointer of ConfigParameter type if the operation was a success, 
    					else if not found returns a NULL pointer.
 			*/
			ConfigParameter* retrieveParameter(const string &path);
			
			/*! @brief 	Retrieves a parameter stored in the data_access tree. 
 
   			@param		String "path" defines what path the parameter is going to be retrieved from
    		@return 	Returns a pointer of ConfigParameter type if the operation was a success, 
    					else if not found returns a NULL pointer.
 			*/
			const ConfigParameter* retrieveParameter(const string &path) const;
			
			/*! @brief 	Edits a parameter stored in the data_access tree. 
 
   			@param		String "path" defines what path the parameter is going to be retrieved from.
   						ConfigParameter "new_parameter" is the new/edited parameter to replace the one
   						currently stored.
    		@return 	Returns true upon success, false otherwise.
 			*/
			bool editParameter(const string &path, ConfigParameter &new_parameter);
			
			
			
			/*! @brief 	Writes the current configuration to file. 
 
   			@param		N/A
    		@return 	Returns true upon success, false otherwise.
 			*/
			bool writeCurrentToFile();

			/*! @brief 	Reads a specified configuration from file. 
 
   			@param		String array "filename_arr" stores the paths to the files to be read into the 
   						data_store.
    		@return 	Returns true upon success, false otherwise.
 			*/
			bool readFromFile(const string &filename_arr[]);
			
			
			
			
			
			/*! @brief 	Adds a new parameter to the data_access tree (won't be used at the moment, 
						maybe later). 
 
   			@param		String "path" defines what path the parameter is going to be added.
   						ConfigParameter "new_parameter" is the new parameter to be added. 
    		@return 	True if the operation was a success, false otherwise.
 			*/
			//bool addNewParameter(string path, ConfigParameter &new_parameter);
			
			/*! @brief 	Removes a parameter from the data_access tree (won't be used at the moment, 
						maybe later). 
 
   			@param		String "path" defines what path the parameter is going to be removed
    		@return 	True if the operation was a success, false otherwise.
 			*/
			//bool removeExistingParameter(string path);
			
		private:
			//Tree to store ConfigParameter objects.
			ptree *data_access;
			//CSM object
			ConfigStorageManager *data_store;
	};
}

#endif ConfigTree_def
