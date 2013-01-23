/*! @file ConfigManager.h
    @brief Defines the main ConfigManager class and the exceptions it throws.
    
    @class ConfigManager
    @brief The interface between the configuration system and the other modules.
    
    @author Mitchell Metcalfe, Sophie Calland
    
  Copyright (c) 2012 Mitchell Metcalfe
    
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

#ifndef ConfigManager_H
#define ConfigManager_H

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/lexical_cast.hpp>

#include "ConfigStorageManager.h"
#include "ConfigTree.h"

#include <string>
using std::string;

namespace ConfigSystem
{
    class ConfigManager
    {
    public:
    	/*!
         *  @brief Creates a configManager and loads the initial configuration
         *         specified.
         *  @param configName The name of the initial configuration to load.  
         *  @return 
         */
    	ConfigManager(std::string configName);
        
    	/*!    
         *  @brief Destroys this ConfigManager and deletes it's ConfigStore
         *         and current ConfigTree.
         */
    	~ConfigManager();
        
        
        /*! @brief  Loads a configuration with the given name.
         *  @param  The name of the configuration to load.
         *  @return Returns whether or not the load succeeded.
         */
        bool loadConfiguration(std::string configName);
        
        /*! @brief  Saves the current configuration.
         *  @param  The name to give the saved configuration.
         *  @return Returns whether or not the save succeeded.
         */
        bool saveConfiguration(std::string configName);
    	

        /*! @brief Reads an integer stored at the given path in the current configuration.
         *  @param paramPath Path to the desired parameter.
         *  @param data variable in which to store the data retrieved.
         *  @return Whether the operation was successful.
         */
        bool readIntValue    (const string &paramPath, const string &paramName, int    &data);
        bool readLongValue   (const string &paramPath, const string &paramName, long   &data);
        bool readFloatValue  (const string &paramPath, const string &paramName, float  &data);
        bool readDoubleValue (const string &paramPath, const string &paramName, double &data);
        bool readStringValue (const string &paramPath, const string &paramName, string &data);

        /*! @brief Stores the given integer in the current configuration at the given path.
         *  @param paramPath Path at which to store the parameter.
         *  @param data The data to store.
         *  @return Whether the operation was successful.
         */
        bool storeIntValue    (const string &paramPath, const string &paramName, int    data);
        bool storeLongValue   (const string &paramPath, const string &paramName, long   data);
        bool storeFloatValue  (const string &paramPath, const string &paramName, float  data);
        bool storeDoubleValue (const string &paramPath, const string &paramName, double data);
        bool storeStringValue (const string &paramPath, const string &paramName, string data);

        // bool readDoubleValue (const string &paramPath, const string &paramName, double &data);
        
    private:
        //! The Configuration System's storage manager.
        ConfigStorageManager    *_configStore   ;

        //! The config tree that stores the configuration system's current
        //! configuration.
        ConfigTree              *_currConfigTree;
    };
}
#endif
