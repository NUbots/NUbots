/*! @file ConfigManager.h
    @brief Defines the main ConfigManager class and the exceptions it throws.
    
    @class ConfigManager
    @brief 
    
    @author Mitchell Metcalfe
    
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

#include "ConfigStorageManager.h"

#include <string>
using std::string;

// class ConfigException : exception
// {
//     virtual const char* what()
//     {
//         return "ConfigException occured.";
//     }
// };

namespace CONFIGURATION
{

    class ConfigManager
    {

    public:
    	//!constructor
    	ConfigManager(std::string filename_arr[]);
    	//!destructor
    	~ConfigManager();
    	
    	//debug
    	void printAll();
    	
        
        /*! @brief Reads an integer stored at the given path in the current configuration.
         *
         *  <long description>
         *
         *  @param paramPath Path to the desired parameter.
         *  @param data variable in which to store the data retrieved.
         *  @return Whether the operation was successful.
         */
        bool readIntParam    (const string paramPath, int    &data); // throw(ConfigException);
        bool readLongParam   (const string paramPath, long   &data); // throw(ConfigException);
        bool readFloatParam  (const string paramPath, float  &data); // throw(ConfigException);
        bool readDoubleParam (const string paramPath, double &data); // throw(ConfigException);
        bool readStringParam (const string paramPath, string &data); // throw(ConfigException);


        /*! @brief Stores the given integer in the current configuration at the given path.
         *
         *  
         *
         *  @param paramPath Path at which to store the parameter.
         *  @param data The data to store.
         *  @return Whether the operation was successful.
         */
        bool storeIntParam    (const string paramPath, int    data); // throw(ConfigException);
        bool storeLongParam   (const string paramPath, long   data); // throw(ConfigException);
        bool storeFloatParam  (const string paramPath, float  data); // throw(ConfigException);
        bool storeDoubleParam (const string paramPath, double data); // throw(ConfigException);
        bool storeStringParam (const string paramPath, string data); // throw(ConfigException);

    private:

        /*! @brief The Configuration System's storage manager.
         *  
         *  Used to access, modify and save config data.
         */
         //change to ptr?
        ConfigStorageManager storageManager;
    };
}
#endif
