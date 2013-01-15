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
//#include <set>
//#include <iostream>

#include "ConfigStorageManager.h"
#include "ConfigParameters.h"

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
	
	
	
	
	
	//Edits the value at the path specified to the new value and type.
	bool ConfigStorageManager::editEntry(std::string path, ConfigParameter *new_entry)
	{
		bool status = false;
		//std::ostringstream convert;
		std::string new_path; 
		
		try
		{
			// //Don't try to edit arrays using this because it won't work. Predefined values will be 
			// //arrays and should be edited and documented manually (see readme.txt).
			// data->put(path + ".value", new_entry.value);
			// data->put(path + ".type", new_entry.type);
			
			// data->put(path + ".rules.range.upper", new_entry.upper_bound);
			// data->put(path + ".rules.range.upper", new_entry.upper_bound);
			
			// new_path = path + ".rules.possiblevalues";
			
			// //loops through possible values in new_entry and stores in the edit entry
			// BOOST_FOREACH(const std::string &possible, new_entry.possible_values)
			// {
			// 	//want to store the data in new_entry array (possiblevalues) into data.
			// 	data->put(new_path + "..value", possible);
			// }
			
			// new_path = path + "rules.conflicts";
			
			// // // //Puts new conflict information into the tree.
			// // BOOST_FOREACH(const std::pair< std::string, ConfigSystem::conflict_limits<std::string> > &iter,
			// // 	      new_entry.rules.conflicts)
			// // {
			// // 	//Want to store conflicts info from the new_entry into the property tree.
			// // 	data->put(new_path + "..path", iter.first);
				
			// // 	//Store "modify" and "modified" flags for each conflict.
			// // 	data->put(new_path + "..modify", iter.second.modify);
			// // 	data->put(new_path + "..modified", iter.second.modified);
			// // }
			
			status = true;
		}
		catch(std::exception &e)
		{
			//DEBUG
			std::cout << "ERROR: " << e.what() << "\n";
			status = false;
		}
		
		return status;
	}

	
	//Accesses the specified variable and it's type, returns as parameters struct.
	ConfigParameter *ConfigStorageManager::accessEntry(std::string path)
	{
		ConfigParameter *cParam;
		// parameters<std::string> retrieved_data;	
		// std::pair<std::string, ConfigSystem::conflict_limits<std::string> retrieved_conflict;
		std::string new_path;
		
		try
		{
			// retrieved_data.value = data->get<std::string>(path + ".value");
			// retrieved_data.type = data->get<std::string>(path + ".type");
			
			// retrieved_data.upper_bound = data->get<std::string>(path + ".rules.range.upper");
			// retrieved_data.lower_bound = data->get<std::string>(path + ".rules.range.lower");
			
			
			// //Retrieves path of possible values
			// new_path = path + ".rules.possiblevalues";
			// //Stores "possiblevalues" JSON array in the possible_values vector of struct.
			// BOOST_FOREACH(const ptree::value_type &child, data->get_child(new_path))
			// {
			// 	retrieved_data.possible_values.push_back(child.second.get<std::string>("value"));
			// }
			
			// //THIS IS FOR READING THE CONFLICT MANAGEMENT STUFF:
			
			// //retrieves path to conflicts.
			// new_path = path + ".rules.conflicts";
			// //Stores "conflicts" JSON array in the conflicts pair vector of struct.
			// //iterates through children of conflicts
			// BOOST_FOREACH(const ptree::value_type &child, data->get_child(new_path))
			// {
			// 	//Need to push_back pairs of strings and conflict_limits to retrieved_data.rules.conflicts
			// 	// retrieved_conflict.first = child.second.get<std::string>("")
			
			// 	//Iterating through conflicts vector to store items
			// 	/*BOOST_FOREACH(const std::pair< std::string, ConfigSystem::conflict_limits<std::string> > 
			// 			&iter, retrieved_data.rules.conflicts)
			// 	{
			// 		//Retrieve string path of conflict.
			// 		iter.first(child.second.get<std::string>("path"));
					
			// 		//Retrieve conflict_limits of conflict.
			// 		iter.second(child.second.get<std::string>("path"));
			// 	}*/
			// }
			
			std::cout << "THIS IS THE PATH TO THE UPDATES: " << new_path << "\n";
		}
		catch(std::exception &e)
		{
			// //DEBUG
			// std::cout << "ERROR: " << e.what() << "\n";
			// retrieved_data.value = "";
			// retrieved_data.type = "";
			
			// retrieved_data.upper_bound = "";
			// retrieved_data.lower_bound = "";
			
			// //Clears the possible_values vector
			// retrieved_data.possible_values.clear();
		}
		
		
		return cParam;
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
