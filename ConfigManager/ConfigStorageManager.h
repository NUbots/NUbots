/*! 
	@file ConfigStorageManager.h
    @brief This is the header file for the storage manager of the configuration system for the NUbots.
 
    @class ConfigStorageManager
    @brief This class uses the Boost property tree to read and store JSON configuration files.
 
    All JSON is parsed to the ptree as strings. Conversion occurs elsewhere; this file is just to store
    the values in the tree. Will read each file individually and store it in the universal ptree.

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

#ifndef ConfigStorageManager_def
#define ConfigStorageManager_def

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/foreach.hpp>
#include <string>
//#include <set>
//#include <exception>
//#include <iostream>


//The number of components (.json files used).
#define ARR_SIZE 6

using boost::property_tree::ptree;

namespace ConfigSystem
{
	//Used to store the conflict limits 
	//FOLLOWING IS FOR COMPLICATED CONFLICTS ONLY, TO BE IMPLEMENTED LATER.
	/*template<typename Item>
	struct conflict_limits
	{
		bool modify;
		bool modified;
	
		
		//boundaries of values which aren't allowed.
		Item upper_bound;
		Item lower_bound;
		
		//array of specific values which are not allowed.
		std::vector<Item> values;
	};*/

	//Used to store parameters read in from JSON files.
	template<typename Item>
	struct parameters
	{
		Item value;
		std::vector<Item>* vector_value;
		std::string type;

		Item upper_bound;
		Item lower_bound;
		
		bool modified;
		bool locked;

		std::vector<Item> possible_values;
		
		std::vector<std::string> conflicts;
		
		//FOLLOWING IS FOR COMPLICATED CONFLICTS ONLY, TO BE IMPLEMENTED LATER).
		//vector of pairs, consisting of a string path and conflict_limits struct 
		//std::vector< std::pair< std::string, ConfigSystem::conflict_limits<Item> > > conflicts;
	};

	class ConfigStorageManager
	{
		public:
			ConfigStorageManager();
			~ConfigStorageManager();
			
			
			
			
			
			
			/*! @brief Reads all JSON files and stores them in the main property tree (data). 
 
   			@param "filename_arr" is an array storing filenames of each system.
    		@return Returns true if success, false otherwise.
 			*/
			bool fileReadAll(std::string filename_arr[]);
			
			/*! @brief Writes all current config into respective files.
 
   			@param "filename_arr" is an array storing filenames of each system.
    		@return Returns true if success, false otherwise.
 			*/
			bool fileWriteAll(bool debug, std::string filename_arr[]);
			
			
			
			
			/*! @brief Writes specific JSON files into respective files.
 
   			@param "filename" specifies the name of the file to store tree data into.
    		@return Returns true if success, false otherwise.
 			*/
			bool fileWriteComponent(int component, std::string filename);
			
			/*! @brief Reads specific JSON file into tree.
 
   			@param "filename" specifies the name of the file to read from.
    		@return Returns true if success, false otherwise.
 			*/
			bool fileReadComponent(int component, std::string filename);
			
			
			
			
			/*! @brief Finds path of files based on index.
 
   			@param "index" is the index of the array.
    		@return Returns root.(componentname) of path depending on index (0 = behaviour ... alphabetical
    			order).
 			*/
			std::string findPath(int index);
			
			
			
			
			
			
		
			/*! @brief Reads the JSON file and stores it in the property tree.
 
   			@param "filename" stores the name of the file where it's being read
    		@return Returns true if success, false otherwise.
 			*/
			bool fileRead(ptree *tree, std::string filename);
			
			/*! @brief Writes current configuration to the specified JSON file.
 
   			@param "filename" stores the name of the file where it's being written.
    		@return Returns true if success, false otherwise.
 			*/
			bool fileWrite(ptree *tree, std::string filename);
			
			
			
			
			
		
			/*! @brief Edits the entry specified by the string path.
 	
   			@param "path" stores the path to the value to be edited. 
   			@param "new_value" is the edited value as a string. 
   			@param "type" stores data type (MUST ALWAYS BE SPECIFIED).
    		@return Returns true if success, false otherwise.
 			*/
			bool editEntry(std::string path, parameters<std::string> new_entry);
		
			/*! @brief Accesses the entry specified by the string path and returns as parameters struct.
 	
   			@param "path" stores the path to the value to be edited. 
    		@return Returns the value stored at the specified path. 
 			*/
			parameters<std::string> accessEntry(std::string path);
	
		
		private:
			/*! @brief Reads the JSON file and stores it in the property tree, or writes current 
					configuration to the specified JSON file.
 
   			@param "filename" stores the name of the file where it's being read/written.
   			@param "read_write" stores true to read, and false to write.
    		@return Returns true if success, false otherwise.
 			*/
			bool fileReadWrite(ptree *tree, std::string filename, bool read_write);
			
			//Data stored in the class
			ptree *data;
	};
}


#endif //ConfigStorageManager_def
