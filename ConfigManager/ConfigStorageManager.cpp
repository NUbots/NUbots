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
	bool ConfigStorageManager::objectSingleStore(const string &path, ConfigParameter &store)
	{
		//Retrieves type.
		ConfigSystem::value_type type = store.getType();
		//"overwrite" stores the converted string version of "store".
		ConfigParameter &overwrite = store;
		
		if(type != vt_string)
		{
			//perform type conversion, allow overwrite to store the new type
			//life would be a lot easier with a template! :/
			if(type == vt_bool)
			{
			}
			else if(type == vt_long)
			else if(type == vt_double)
			else if(type == vt_vector_long)
			
			
			//if conversion fails
			return false;
		}
		
		//after type conversion, overwrite should contain the "store" data in string form:
		//checks if types are vectors or not
		if(type.compare("1d_vector") == 0) 
		{ 
			/* Store in 1d vector variable */ 
		}
		else if(type.compare("2d_vector") == 0) 
		{ 
			/* Store in 2d vector variable */ 
		}
		else if(type.compare("3d_vector") == 0) 
		{ 
			/* Store in 3d vector variable */ 
		}
		else
		{
			/* Store in single variable */
			data->put(path + ".value", overwrite.getValue());
		}
		
		//Type
		data->put(path + ".type", type);
		
		//Ranges
		data->put(path + ".range.lower", overwrite.getLower());
		data->put(path + ".range.upper", overwrite.getUpper());
		
		//Whatever else needs to be put in ...
		return true;
	}
	
	
	
	
	
	//PRIVATE MEMBER FUNCTIONS:
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
