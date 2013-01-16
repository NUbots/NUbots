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
#include <string>
#include <exception>

#include "ConfigStorageManager.h"
#include "ConfigParameter.h"

using boost::property_tree::ptree;
using ConfigSystem::ConfigStorageManager;

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
	
	
	//Store a single ConfigParameter object in the tree.
	/*bool ConfigStorageManager::objectSingleStore(const string &path, ConfigParameter &store)
	{
		bool status = false;
		//Retrieves type.
		ConfigSystem::value_type type = store.getType();
		
		
		if(type != vt_string)
		{
			//perform type conversion from "type" to string. Allow overwrite to store the string type.
			if(type == vt_bool)
			{
				//Converts boolean to string and stores in tree.
				status = convertBoolToStr(store);
				
				if(!status) return false;
			}
			else if(type == vt_long)
			{
				long convert;
				ConfigRange<long> range_convert;
				
				
				string value, upper, lower, u_bound_type, l_bound_type;
				
				status = store.getValue_long(convert);
				
				//Stores the type as a long.
				data->put(path + ".type", "long");
				
				if(status && store.getRange_long(range_convert))
				{
					storeNumber<long>(convert, range_convert);
				
					bool ubound_success, lbound_success;
					
					//Convert value.
					value = numberToString(convert);
					
					//Convert upper bound.
					ubound_success = getBound(range_convert.getUpperBoundType(), *(range_convert.getMax),
											&u_bound_type, &upper);
					//Convert lower bound.						
					lbound_success = getBound(range_convert.getLowerBoundType(), *(range_convert.getMin),
											&l_bound_type, &lower);
					
					if( !(ubound_success && lbound_success) ) return false;
					
					
					//Successful conversion, store strings in tree:
					data->put(path ".value", value);
					
					data->put(path ".range.upper..type", u_bound_type);
					data->put(path ".range.upper..bound", upper);
					
					data->put(path ".range.lower..type", l_bound_type);
					data->put(path ".range.lower..bound", lower);
				}
				//Errors with retrieving range and/or value, returns false.
				else
				{
					return false;
				}
				
			}
			else if(type == vt_double)
			{
			}
			else if(type == vt_1dvector_long)
			{
			}
			else if(type == vt_1dvector_double)
			{
			}
			else
			{
				//type not specified
				return false;
			}
			
			overwrite.setType(type);
		}
		
		//after type conversion, overwrite should contain the "store" data in string form:
		//checks if types are vectors or not
		if(type.compare("1d_vector") == 0) 
		{ 
			/* Store in 1d vector variable */ 
		//}
		//else if(type.compare("2d_vector") == 0) 
		//{ 
			/* Store in 2d vector variable */ 
		//}
		//else if(type.compare("3d_vector") == 0) 
		//{ 
			/* Store in 3d vector variable */ 
		//}
		//else
		//{
			/* Store in single variable */
			//data->put(path + ".value", overwrite.getValue());
		//}
		
		//Type
		//data->put(path + ".type", type);
		
		//Ranges
		/*data->put(path + ".range.lower", overwrite.getLower());
		data->put(path + ".range.upper", overwrite.getUpper());
		
		//Whatever else needs to be put in ...
		return true;
	}*/
	
	
	
	
	
	//PRIVATE MEMBER FUNCTIONS:
	//Converts from a string and stores in a ConfigParameter object.
	bool stringToCP(const string path, ConfigParameter &config_store)
	{
		/* WORKING HERE TOMORROW */
		string variable_name, variable_path, variable_description;
		value_type variable_type;
		
		//Retrieves variable name and path from the given path and returns false if error.
		if( !(getNameFromPath(path, &variable_path, &variable_name)) ) return false; /*Invalid path exc?*/
		
		config_store.setName(variable_name);
		config_store.setPath(variable_path);
		config_store.setDesc(data->get<std::string>(path + ".desc"));
		
		
		
		
	}
	
	//Determines the type and converts to a value_type enum.
	bool findTypeFromTree(const string variable_type, value_type &return_type)
	{
		if(variable_type.compare("string") == 0) return_type = vt_string;
		else if(variable_type.compare("double") == 0) return_type = vt_double;
		else if(variable_type.compare("long") == 0) return_type = vt_long;
		else if(variable_type.compare("bool") == 0) return_type = vt_bool;
		else if(variable_type.compare("1dv_double") == 0) return_type = vt_1dvector_double;
		else if(variable_type.compare("1dv_long") == 0) return_type = vt_1dvector_long;
		else return false;
		
		return true;
	}
	
	//Gets the variable name from a path containing the name.
	bool getNameFromPath(const string path, string &return_path, string &return_name)
	{
		string variable_name, variable_path;
		boost::char_separator<char> delim(".");
		boost::tokenizer< boost::char_separator<char> > tokens(path, delim);
		
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
	
	//Converts string to bounds:
	template<typename T>
	bool stringToBounds(const string path, ConfigRange<typename T> &converted_bounds)
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
	bool stringToNumber(const string number_str, typename T &result)
	{
		stringstream stream(number_str);
		
		if( !(stream >> result) ) return false;
		
		return true;
	}
	
	//Converts a boolean to a string
	bool ConfigStorageManager::convertBoolToStr(ConfigParameter &store)
	{
		bool convert;
		status = store.getValue_bool(convert);
		
		//Stores the type as a boolean.
		data->put(path + ".type", "bool");
		
		//if successful read, converts boolean to string.
		if(status)
		{
			if(convert) data->put(path + ".value", "true");
			else if(!convert) data->put(path + ".value", "false");
		}
		else
		{
			return false;
		}
		
		return status;
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
