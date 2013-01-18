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

#include "ConfigParameter.h"
#include "ConfigRange.h"


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
			
			
			
			
			
			/*! @brief 	Stores a single ConfigParameter object into the data storage tree,
						ready to write to file (NOTE the ConfigParameter object stores its own path).
 
   			@param 		"config_store" is the ConfigParameter object to be placed into the tree.
    		@return 	Returns true if success, false otherwise.
 			*/
 			template<typename T>
			bool storeCP(const ConfigParameter &config_store);
			
			/*! @brief 	Retrieves parameter from tree and stores as a ConfigParameter object.
 
   			@param 		"path" specifies path to the variable.
   			@param		"config_return" is the ConfigParameter object to be returned.
    		@return 	Returns true if success, false otherwise.
 			*/
 			template<typename T>
			bool stringToCP(const string path, ConfigParameter &config_return);
			
			
	
		
		private:
			//Data stored in the class
			ptree *data;
		
		
		
		
			/*! @brief 	Reads the JSON file and stores it in the property tree, or writes current 
						configuration to the specified JSON file.
 
   			@param 		"filename" stores the name of the file where it's being read/written.
   			@param 		"read_write" stores true to read, and false to write.
    		@return 	Returns true if success, false otherwise.
 			*/
			bool fileReadWrite(ptree *tree, std::string filename, bool read_write);
			
			/*! @brief 	Converts the bound type to string and stores it in the tree (used by storeCP)
 
   			@param 		"path" stores the path to write the parameter in the tree to.
   			@param 		"store_me" is the BoundType to be stored.
    		@return 	Returns true if success, false otherwise.
 			*/
			bool storeBoundType(const string path, const BoundType store_me);
			
			/*! @brief 	Retrieves a generic value and range based on what type is used from a 
						ConfigParameter object (used by storeCP).
 
   			@param 		"config_store" is the ConfigParameter object to source the generic values from.
   			@param 		"value_data" is the value from the CP object.
   			@param 		"range_data" is the range from the CP object.
    		@return 	Returns true if success, false otherwise.
 			*/
			template<typename T>
			bool retrieveGenericValue(const ConfigParameter &config_store, T &value_data, 
							ConfigRange<T> &range_data);
							
							
							
							
							
			/*! @brief 	Retrieves and converts the value information from string to whatever type
						and stores it correspondingly in the ConfigParameter object (used by stringToCP).
 
   			@param 		"path" is the path where the data is stored in the tree.
   			@param 		"variable_type" is the type stored in the CP object.
   			@param 		"config_return" is the CP object where the data is retrieved.
    		@return 	Returns true if success, false otherwise.
 			*/				
			template<typename T>
			bool convertStoreValue(const string &path, const value_type &variable_type, 
							ConfigParameter &config_return);
			
			/*! @brief  Converts types of values and ranges based on what is input (used by stringToCP).
 
   			@param 		"path" is the path where the data is stored in the tree.
   			@param 		"variable_type" is the type stored in the CP object.
   			@param 		"retrieved_value" is the string form of the variable_value.
   			@param		"variable_value" is the converted form of value.
   			@param		"variable_range" is the converted form of range.
    		@return 	Returns true if success, false otherwise.
 			*/				
			template<typename T>
			bool convertType(const string path, const value_type variable_type, 
							const string retrieved_value, T &variable_value, 
							ConfigRange<T> &variable_range);
			
			/*! @brief  Converts string data stored in the tree to a meaningful ConfigRange object
						(used by stringToCP).
 
   			@param 		"path" is the path where the data is stored in the tree.
   			@param 		"converted_bounds" are the bounds fully converted (from string to ConfigRange).
    		@return 	Returns true if success, false otherwise.
 			*/				
			template<typename T>
			bool stringToBounds(const string path, ConfigRange<T> &converted_bounds);
			
			/*! @brief  Converts a string to long, double, etc (used by stringToCP).
 
   			@param 		"number_str" is the string to be converted.
   			@param 		"result" is the converted number.
    		@return 	Returns true if success, false otherwise.
 			*/	
			template<typename T>
			bool stringToNumber(const string number_str, T &result);
			
			/*! @brief  Finds type based on string and/or value_type enum. Will return both accordingly
						(used by stringToCP and storeCP).
 
   			@param 		"str_type" is the string storing/to store the type.
   			@param 		"val_type" is the value_type storing/to store the type as meaningful value_type.
    		@return 	Returns true if success, false otherwise.
 			*/
			bool findTypeFromString(string &str_type, value_type &val_type);
			
			/*! @brief  Based on a path containing the name and the path, returns a separate name and 
						path (used by stringToCP).
 
   			@param 		"path" is the full path of the parameter.
   			@param 		"return_path" is the path without the parameter name.
   			@param 		"return_name" is the parameter name.
    		@return 	Returns true if success, false otherwise.
 			*/
			bool getNameFromPath(const string path, string &return_path, string &return_name);
			
			
			
			
			
			
	};
}


#endif //ConfigStorageManager_def
