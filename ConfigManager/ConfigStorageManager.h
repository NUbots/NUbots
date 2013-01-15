/*! 
	@file 	ConfigStorageManager.h
    @brief 	This is the header file for the storage manager of the configuration system for the NUbots.
 
    @class 	ConfigStorageManager
    @brief 	This class uses the Boost property tree to read and store JSON configuration files.
 
    All JSON is parsed to the ptree as strings. Conversion occurs elsewhere; this file is just to store
    the values in the tree. Will read each file individually and store it in the ptree (only storing
    strings).

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
//#include <exception>
//#include <iostream>


//The number of components (.json files used).
#define ARR_SIZE 6

using boost::property_tree::ptree;

namespace ConfigSystem
{	
	class ConfigStorageManager
	{
		public:
			/*! @brief Constructor. 
 
   			@param N/A.
    		@return Creates a new ConfigStorageManager object.
 			*/
			ConfigStorageManager();
			
			/*! @brief Destructor. 
 
   			@param N/A.
    		@return Deletes a ConfigStorageManager object.
 			*/
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
			
			
			
			
			
		
			/*! @brief 	Stores a single entry in a specified path. Will overwrite path if already exists.
 	
   			@param "path" stores the path the value is to be placed in.
   			@param "store" is the new ConfigParameter to be stored.
    		@return Returns true if success, false otherwise.
 			*/
			bool objectSingleStore(const string &path, const ConfigParameter &store)
			
			
	
		
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
