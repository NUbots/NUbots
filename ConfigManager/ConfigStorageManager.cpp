/*! 
	@file ConfigStorageManager.cpp
    @brief This is the cpp file for the storage manager of the configuration system for the NUbots.
 
    @class ConfigStorageManager
    @brief This class uses the Boost property tree to read and store JSON configuration files.
 
    All JSON is parsed to the ptree as strings. Conversion occurs elsewhere; this file is just to store
    the values in the tree. Intended to be as minimal as possible.

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

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/foreach.hpp>
#include <boost/tokenizer.hpp>
#include <string>
#include <exception>

#include "ConfigStorageManager.h"
#include "ConfigParameter.h"
#include "ConfigRange.h"

using boost::property_tree::ptree;
using ConfigSystem::ConfigStorageManager;
using std::string;
using std::stringstream;

namespace ConfigSystem
{
	//Constructor
	ConfigStorageManager::ConfigStorageManager()
	{
		data = new ptree();
	}
	
	//Destructor (shouldn't need this)
	ConfigStorageManager::~ConfigStorageManager()
	{
		delete data;
	}
	
	
	
	
	
	//Reads all of the files specified in a string and stores them in the tree.
	bool ConfigStorageManager::fileReadAll(std::string filename_arr[])
	{
		bool success = false;
		
		//Read each file
		for(int i = 0; i != ARR_SIZE; i++)
		{
			std::cout << "Reading from: " << filename_arr[i] << std::endl;
			
			success = fileReadComponent(i, filename_arr[i]);
			
			success = true;		
		}
				
		//Will always return true, will place all of the existing files into tree and miss those which
		//don't exist.
		return success;
	}
	
	//Writes to all files in the filename array "filename_arr".
	bool ConfigStorageManager::fileWriteAll(bool debug, std::string filename_arr[])
	{
		bool success = false;
		
		if(debug)
		{
			return fileWrite(data, filename_arr[0]);
		}
		else
		{
			//Need to write to each individual file.
			//Read each file
			for(int i = 0; i != ARR_SIZE; i++)
			{
				std::cout << "Writing to: " << filename_arr[i] << std::endl;
				
				success = fileWriteComponent(i, filename_arr[i]);
				success = true;
				
			}
			
			return success;
		}
	}
	
	//Writes the specific component to file specified:
	bool ConfigStorageManager::fileWriteComponent(int component, std::string filename)
	{
		std::string path;
		ptree *aux_ptr;
		bool success;
		
		//Find path of ptree
		path = findPath(component);
		//Retrieve subtree from path
		aux_ptr = &(data->get_child(path));
		//Write to component file.
		success = fileWrite(aux_ptr, filename);
		
		return success;
		
	}
	
	//Writes the specific component to the file specified
	bool ConfigStorageManager::fileReadComponent(int component, std::string filename)
	{
		std::string path;
		ptree *aux_ptree;
		bool success;
		
		aux_ptree = new ptree();
		success = fileRead(aux_ptree, filename);
		
		if(success)
		{	
			//Finds path to store.
			path = findPath(component);
			
			//Using put to ensure no multiple keys.
			data->put_child(path, *(aux_ptree));
		}
		
		delete aux_ptree;
		
		//return success;
		return true;
	}
	
	//Finds the path based on index (better way to do this?)
	std::string ConfigStorageManager::findPath(int index)
	{
		std::string new_path;
		
		switch(index)
		{
			case 0:
				new_path = "root.behaviour";
				break;
				
			case 1:
				new_path = "root.communication";
				break;
				
			case 2:
				new_path = "root.configuration";
				break;
				
			case 3:
				new_path = "root.localisation";
				break;
				
			case 4:
				new_path = "root.locomotion";
				break;
				
			case 5:
				new_path = "root.vision";
				break;
				
			default:
				new_path = "random";
				break;
		}
		
		return new_path;
	}
	
	//Reads from JSON file specified in "filename" and stores in ptree.
	bool ConfigStorageManager::fileRead(ptree *tree, std::string filename)
	{
		if(tree != NULL) return fileReadWrite(tree, filename, true);
		else return false;
	}
	
	//Writes current configuration to the specified JSON file.
	bool ConfigStorageManager::fileWrite(ptree *tree, std::string filename)
	{
		if(tree != NULL) return fileReadWrite(tree, filename, false);
		else return false;
	}
	
	
	
	//Stores a single ConfigParameter object in the tree using path specified in ConfigParameter object.
	template<typename T>
	bool ConfigStorageManager::storeCP(const ConfigParameter &config_store)
	{
		string full_path, str_type, ret_desc;
		value_type ret_type;
		T ret_value;
		ConfigRange<T> ret_bounds;
		BoundType bt_upper, bt_lower;
		
		full_path = config_store.getName() + config_store.getPath();
		ret_type = config_store.getType();
		
		//Find type in string form, place in tree at stored path.
		if( !(findTypeFromString(str_type, ret_type)) ) return false;
		data->put(full_path + ".type", str_type);
		
		//If the data isn't a vector
		if( !((ret_type == vt_1dvector_long) || (ret_type == vt_1dvector_double)) )
		{
			if( !(retrieveGenericValue(config_store, &ret_value, &ret_bounds)) ) return false;
			
			data->put(full_path + ".value", ret_value);
			data->put(full_path + ".range.max", ret_bounds.getMax());
			data->put(full_path + ".range.min", ret_bounds.getMin());
			
			//Check and store upper and lower bound types
			if( (storeBoundType(full_path, ret_bounds.getUpperBoundType())) && 
						(storeBoundType(full_path, ret_bounds.getLowerBoundType())))
			{
				return false;
			}
		}
		//if the data is a vector
		else
		{
			//Write stuff here for vectors when we do it.
		}
	}
	
	
	
	//Converts from a string and stores in a single ConfigParameter object.
	template<typename T>
	bool ConfigStorageManager::stringToCP(const std::string path, ConfigParameter &config_return)
	{
		string variable_name, variable_path, variable_description;
		value_type variable_type;
		string retrieved_type;
		
		//Retrieves variable name and path from the given path and returns false if error.
		if( !(getNameFromPath(path, variable_path, variable_name)) ) return false; /*Invalid path exc?*/
		//Sets name, path and description.
		config_return.setName(variable_name);
		config_return.setPath(variable_path);
		config_return.setDescription(data->get<std::string>(path + ".desc"));
		
		
		
		//Checks for different types, converts accordingly.
		retrieved_type = data->get<std::string>(path + ".type");
		if( !(findTypeFromString(retrieved_type, variable_type)) ) return false;
		config_return.setType(variable_type);
				
		//Converts and stores "value" in config_return
		if( !(convertStoreValue<T>(path, variable_type, config_return)) ) return false;
		
		//if no errors.
		return true;
	}
	
	
	
	
	
	
	
	
	
	
	
	//PRIVATE MEMBER FUNCTIONS:
	//FOLLOWING ARE USED BY storeCP
	//Stores a bound type in teh tree as strings
	bool ConfigStorageManager::storeBoundType(const string path, const BoundType store_me)
	{
		//store as strings
		switch(store_me)
		{
			case CLOSED:
				data->put(path + ".range.max_type", "CLOSED");
				break;
			
			case OPEN:
				data->put(path + ".range.max_type", "OPEN");
				break;
			
			case NONE:
				data->put(path + ".range.max_type", "NONE");
				break;
			
			default:
				return false;
				break;
		}
		
		return true;
	}
	
	//Retrieves a generic value from the ConfigParameter object 
	template<typename T>
	bool ConfigStorageManager::retrieveGenericValue(const ConfigParameter &config_store, 
							T &value_data, ConfigRange<T> &range_data)
	{
		value_type ret_type = config_store.getType();
		
		switch(ret_type)
		{
			case vt_bool:
				if( !(config_store.getValue_bool(value_data)) ) return false;
				
				range_data.setUpperBoundType(NONE);
				range_data.setLowerBoundType(NONE);
				range_data.setMax(false);
				range_data.setMin(false);
				break;
				
			case vt_long:
				if( !(config_store.getValue_long(value_data)) ) return false;
				if( !(config_store.getRange_long(range_data)) ) return false;
				break;
				
			case vt_double:
				if( !(config_store.getValue_double(value_data)) ) return false;
				if( !(config_store.getRange_double(range_data)) ) return false;
				break;
				
			case vt_string:
				if( !(config_store.getValue_string(value_data)) ) return false;
				
				range_data.setUpperBoundType(NONE);
				range_data.setLowerBoundType(NONE);
				range_data.setMax("");
				range_data.setMin("");
				break;
				
			case vt_1dvector_long:
				break;
				
			case vt_1dvector_double:
				break;
				
			default:
				return false;
				break;
		}
	}
	
	
	
	
	//FOLLOWING ARE USED BY stringToCP
	//Converts and stores values into the tree.
	template<typename T>
	bool ConfigStorageManager::convertStoreValue(const string &path, const value_type &variable_type, 
							ConfigParameter &config_return)
	{
		string retrieved_value = data->get<std::string>(path + ".value");
		T variable_value;
		ConfigRange<T> variable_range;
		
		if( !(convertType(path, variable_type, retrieved_value, &variable_value, &variable_range)) )
		{
			return false;
		}
		
		switch(variable_type)
		{
			case vt_bool:
				config_return.setValue_bool(variable_value);
				break;
				
			case vt_long:
				config_return.setValue_long(variable_value);
				config_return.setRange_long(variable_value);
				break;
				
			case vt_double:
				config_return.setValue_double(variable_value);
				config_return.setRange_double(variable_value);
				break;
				
			case vt_string:
				config_return.setValue_string(variable_value);
				break;
				
			case vt_1dvector_long:
				break;
				
			case vt_1dvector_double:
				break;
				
			default:
				return false;
				break;
		}
	
		return true;	
	}
	
	
	//Converts types based on what is input
	template<typename T>
	bool ConfigStorageManager::convertType(const string path, const value_type variable_type, 
					const string retrieved_value, T &variable_value, 
					ConfigRange<T> &variable_range)
	{
		if(variable_type == vt_bool)
		{
			//Convert from str to bool.
			if(retrieved_value.compare("true")) variable_value = true;
			else if(retrieved_value.compare("false")) variable_value = false;
			else return false;
			
			//Sets range to NONE.
			variable_range.setUpperBoundType(NONE);
			variable_range.setLowerBoundType(NONE);	
			
			//Might need something else?
			return true;		
		}
		else if( (variable_type == vt_long) || (variable_type == vt_double) )
		{
			//Convert value.
			if( !(stringToNumber(retrieved_value, &variable_value)) ) return false;
			
			//Convert range.
			if( !(stringToBounds(path, &variable_range)) ) return false;
			
			return true;
		}
		else if(variable_type == vt_string)
		{
			variable_value = retrieved_value;
			
			variable_range.setUpperBoundType(NONE);
			variable_range.setLowerBoundType(NONE);	
			
			return true;
		}
		else if( (variable_type == vt_1dvector_long) || (variable_type == vt_1dvector_long) )
		{
			//Have a retrieved_value string array? Move these to variable_value? Work out later ...
			
			return true;
		}
		else
		{
			return false;
		}		
	}
	
	//Converts string to bounds:
	template<typename T>
	bool ConfigStorageManager::stringToBounds(const string path, 
									ConfigRange<T> &converted_bounds)
	{
		T ub_num, lb_num;
		string ub, lb, ub_type, lb_type;
		
		ub_type = data->get<std::string>(path + ".range.uBound");
		lb_type = data->get<std::string>(path + ".range.lBound");
		
		if(ub_type.compare("NONE") != 0)
		{
			if(ub_type.compare("OPEN") == 0) converted_bounds.setUpperBoundType(OPEN);
			else if(ub_type.compare("CLOSED") == 0) converted_bounds.setUpperBoundType(CLOSED);
			else return false;
			
			ub = data->get<std::string>(path + ".range.max");
			if( !(stringToNumber(ub, &ub_num)) ) return false;
			
			converted_bounds.setMax(ub_num);
		}
		else
		{
			converted_bounds.setUpperBoundType(NONE);
		}
		
		if(lb_type.compare("NONE") != 0)
		{
			if(lb_type.compare("OPEN") == 0) converted_bounds.setLowerBoundType(OPEN);
			else if(lb_type.compare("CLOSED") == 0) converted_bounds.setLowerBoundType(CLOSED);
			else return false;
			
			lb = data->get<std::string>(path + ".range.min");
			if( !(stringToNumber(lb, &lb_num)) ) return false;
			
			converted_bounds.setMin(lb_num);
		}
		else
		{
			converted_bounds.setUpperBoundType(NONE);
		}
		
		return true;
	}
	
	//Converts string to double, long, etc.
	template<typename T>
	bool ConfigStorageManager::stringToNumber(const string number_str, T &result)
	{
		stringstream stream(number_str);
		
		if( !(stream >> result) ) return false;
		
		return true;
	}
	
	
	
	//Determines the type based on string or value_type. Returns both as specified type.
	bool ConfigStorageManager::findTypeFromString(string &str_type, value_type &val_type)
	{
		if( (str_type.compare("string") == 0) || (val_type == vt_string) ) 
		{
			val_type = vt_string;
			str_type = "string";
		}
		
		else if( (str_type.compare("double") == 0) || (val_type == vt_double) ) 
		{
			val_type = vt_double;
			str_type = "double";
		}
		
		else if( (str_type.compare("long") == 0) || (val_type == vt_long) ) 
		{
			val_type = vt_long;
			str_type = "long";
		}
		
		else if( (str_type.compare("bool") == 0) || (val_type == vt_bool) ) 
		{
			val_type = vt_bool;
			str_type = "bool";
		}
		
		else if( (str_type.compare("1dv_double") == 0) || (val_type == vt_1dvector_double) ) 
		{
			val_type = vt_1dvector_double;
			str_type = "1dv_double";
		}
		
		else if( (str_type.compare("1dv_long") == 0) || (val_type == vt_1dvector_long) ) 
		{
			val_type = vt_1dvector_long;
			str_type = "1dv_long";
		}
		
		else return false;
		
		return true;
	}
	
	//Gets the variable name from a path containing the name.
	bool ConfigStorageManager::getNameFromPath(const string path, string &return_path, 
								string &return_name)
	{
		typedef boost::tokenizer< boost::char_separator<char> > tokenizer;
	
		string variable_name, variable_path;
		boost::char_separator<char> delim(".");
		tokenizer tokens(path, delim);
		
		tokenizer::iterator tok_iter_behind;
		
		//Retrieves the name of the variable
		for(tokenizer::iterator tok_iter = tokens.begin(); tok_iter != tokens.end(); ++tok_iter)
		{
			if(tok_iter != tokens.begin()) 
			{
				variable_path += *(tok_iter_behind) + ".";
			}
			variable_name = *(tok_iter);
		
			tok_iter_behind = tok_iter;
		}
		
		return_path = variable_path;
		return_name = variable_name;
		
		return true;
	}
	
	//Reads the JSON file specified, stores in specified ptree.
	bool ConfigStorageManager::fileReadWrite(ptree *tree, std::string filename, bool read_write)
	{
		bool status = false;
		
		if(tree != NULL)
		{
			try
			{
				if(read_write) 	boost::property_tree::json_parser::read_json(filename, *(tree));
				else			boost::property_tree::json_parser::write_json(filename, *(tree));
				
				status = true;
			}
			catch(std::exception e)
			{
				//DEBUG
				std::cout << "ERROR: " << e.what() << "\n";
				status = false;
			}
		}
		
		return status;
	}
}
