/*! 
	@file 	ConfigTree.cpp
    @brief 	Implementation file for the ConfigTree class. 
 
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

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/foreach.hpp>
#include <string>
#include <exception>

#include "ConfigStorageManager.h"
#include "ConfigTree.h"

using std::string;
using ConfigSystem::ConfigStorageManager;

namespace ConfigSystem
{
	//Constructor
	ConfigTree::ConfigTree()
	{
		data_access = new ptree();
		data_store = new ConfigStorageManager();
	}
	
	//Constructor (this one might be unnecessary).
	ConfigTree::ConfigTree(string filename_arr[])
	{
		bool status;
		
		data_access = new ptree();
		data_store = new ConfigStorageManager();
		
		try
		{
			status = data_store.fileReadAll(filename_arr);
		
			if(!success) throw /* Invalid filename/s exception */;
		}
		catch(std::exception &e)
		{
			//DEBUG
			std::cout << "ERROR: " << e.what() << "\n";
			status = false;
		}
		
	}
	
	//Destructor
	ConfigTree::~ConfigTree()
	{
		delete data_store;
		delete data_access;
	}
	
	
	
	
	
	
	//Retrieves a ConfigParameter stored in the tree.
	ConfigParameter* ConfigTree::retrieveParameter(const string &path)
	{
		ConfigParameter *ptr;
		
		ptr = &( data_access->get<ConfigParameter>(path) );
		
		return ptr;
	}
	
	//Retrieves a ConfigParameter stored in the tree.
	const ConfigParameter* ConfigTree::retrieveParameter(const string &path) const
	{
		ConfigParameter *ptr;
		
		ptr = &( data_access->get<ConfigParameter>(path) );
		
		return ptr;
	}
	
	//Edits a parameter in the tree.
	bool editParameter(const string &path, ConfigParameter &new_parameter)
	{
		//Data can only be edited if it exists in the access tree.
		try
		{
			ConfigParameter &edit_parameter = data_access->get<ConfigParameter>(path);
		}
		catch(boost::property_tree::ptree_bad_path &e)
		{
			//Bad path exception (i.e. does not exist).
			std::cout << "ERROR: " << e.what() << std::endl;
			return false;
		}
		
		//Reaches here if data exists.
		//Overwrites current data in access tree
		data_access->put(path, new_parameter);
		
		return true;
	}
	
	//Writes the current configuration in data_access tree to file (the data_store CSM object)
	bool ConfigTree::writeCurrentToFile()
	{
		bool status = false;
		
		//convert everything in data_access tree to strings and store in the CSM object.
	}

}
