/*! 
	@file ConfigStorageManager.cpp
    @brief This is the cpp file for the storage manager of the configuration system for the NUbots.
    
    @class ConfigStorageManager
    @brief This class uses the Boost property tree to read and store JSON configuration files.
 
    All JSON is parsed to the ptree as strings. Conversion occurs elsewhere; this file is just to store
    the values in the tree. Intended to be as minimal as possible.

    @author Sophie Calland, Mitchell Metcalfe
 
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
    ConfigStorageManager::ConfigStorageManager()
	{
		CONFIGSYS_DEBUG_CALLS;
	}

    bool ConfigStorageManager::loadConfig(ConfigTree *&loadTree, std::string configName)
    {
        CONFIGSYS_DEBUG_CALLS;

        // get the path to load from
        std::string loadPath = "nubot/" + configName + ".json";
        // load tree from file(s) as ptree<string>
        ptree rawTree;
        
        try
        {
            read_json(loadPath, rawTree);
        }
        catch(boost::property_tree::json_parser::json_parser_error &je)
        {
            std::cout   << "ConfigStorageManager::loadConfig(...): " 
                        << je.what() 
                        << std::endl;
            return false;
        }

        /* 
         * (convert to ptree<ConfigParameter>) - currently performing this 
         * conversion ConfigTree's accessor methods. 
         */

        // Set the output parameter
        ConfigTree *newTree = new ConfigTree(rawTree);
        loadTree = newTree;

        return true;
    }

    bool ConfigStorageManager::saveConfig(ConfigTree *saveTree, std::string configName)
    {
        CONFIGSYS_DEBUG_CALLS;

        std::string savePath = "ConfigurationFiles/" + configName + ".json";

        // save tree to file(s) as ptree<string>
        try
        {
            write_json(savePath, saveTree->getRoot());
        }
        catch(boost::property_tree::json_parser::json_parser_error &je)
        {
            std::cout   << "ConfigStorageManager::saveConfig(...): " 
                        << je.what() 
                        << std::endl;
            return false;
        }

        return true;
    }
}
