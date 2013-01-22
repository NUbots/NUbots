/*! 
    @file     ConfigStorageManager.h
    @brief     This is the header file for the storage manager of the configuration system for the NUbots.
 
    @class     ConfigStorageManager
    @brief     Manages persistant storage of configuration data (i.e. 
               reading/writing ConfigTrees to/from disk).
    
    All JSON is parsed to the ptree as strings. Conversion occurs elsewhere; this file is just to store
    the values in the tree. Will read each file individually and store it in the ptree (only storing
    strings).

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

#ifndef ConfigStorageManager_def
#define ConfigStorageManager_def

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/foreach.hpp>
#include <string>
//#include <exception>
//#include <iostream>

#include "ConfigTree.h"
#include "ConfigParameter.h"
#include "ConfigRange.h"

using boost::property_tree::ptree;

namespace ConfigSystem
{    
    class ConfigStorageManager
    {
    public:
        /*!
         * @brief  Constructs a new ConfigStorageManager
         */
        ConfigStorageManager();

        /*! @brief  Loads a configuration with a given name.
         *  @param  The tree into which the configuration will be loaded.
         *  @param  The name of the configuration to load.
         *  @return Returns whether or not the load succeeded.
         */
        bool loadConfig(ConfigTree *&loadTree, std::string configName);

        /*! @brief  Saves a configuration in a ConfigTree to file.
         *  @param  The tree containing the configuration to save.
         *  @param  The name to give the saved configuration.
         *  @return Returns whether or not the save succeeded.
         */
        bool saveConfig(ConfigTree *saveTree, std::string configName);
        
    };
}


#endif
