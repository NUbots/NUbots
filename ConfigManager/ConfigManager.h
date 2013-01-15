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
#include <boost/lexical_cast.hpp>
    
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

namespace ConfigSystem
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
    	
        
        
        bool readIntParam     (const string &paramPath, 
                               const string &paramName,
                               int    &data);
        bool storeIntParam    (const string &paramPath,
                               const string &paramName,
                               int    data);
        
        /*! @brief Reads an integer stored at the given path in the current configuration.
         *
         *  <long description>
         *
         *  @param paramPath Path to the desired parameter.
         *  @param data variable in which to store the data retrieved.
         *  @return Whether the operation was successful.
         */
        // bool readIntParam    (const string &paramPath, const string &paramName, int    &data); // throw(ConfigException);
        bool readLongParam   (const string &paramPath, const string &paramName, long   &data); // throw(ConfigException);
        bool readFloatParam  (const string &paramPath, const string &paramName, float  &data); // throw(ConfigException);
        bool readDoubleParam (const string &paramPath, const string &paramName, double &data); // throw(ConfigException);
        bool readStringParam (const string &paramPath, const string &paramName, string &data); // throw(ConfigException);

        /*! @brief Stores the given integer in the current configuration at the given path.
         *
         *  
         *
         *  @param paramPath Path at which to store the parameter.
         *  @param data The data to store.
         *  @return Whether the operation was successful.
         */
        // bool storeIntParam    (const string &paramPath, const string &paramName, int    data); // throw(ConfigException);
        bool storeLongParam   (const string &paramPath, const string &paramName, long   data); // throw(ConfigException);
        bool storeFloatParam  (const string &paramPath, const string &paramName, float  data); // throw(ConfigException);
        bool storeDoubleParam (const string &paramPath, const string &paramName, double data); // throw(ConfigException);
        bool storeStringParam (const string &paramPath, const string &paramName, string data); // throw(ConfigException);

    private:

        /*! @brief The Configuration System's storage manager.
         *  
         *  Used to access, modify and save config data.
         */
         //change to ptr?
        ConfigStorageManager storageManager;


        // template <typename T>
        // bool readParam (const string &paramPath, const string &paramName, T &data, const char* typeStr)
        // {
        //     string fullParamPath = paramPath + "." + paramName;

        //     // Get the parameter information from the pTree
        //     parameter<string> pStruct = storageManager.accessEntry(fullParamPath);
            
        //     // Aliases for parameter strings
        //     string &value = pStruct.value;
        //     string &type  = pStruct.type;

        //     if(!type.compare(typeStr))
        //         return false; // type mismatch error. (throw ConfigTypeMismatchException?)

        //     T result;

        //     try
        //     {
        //         result = boost::lexical_cast<T>(value); // perform conversion from string
        //     }
        //     catch (const boost::bad_lexical_cast &blcExc) // conversion failed
        //     {
        //         // throw error?
        //         //set default value
        //         // result = 0;
        //         // log reason for the error
        //         return false;
        //     }

        //     data = result; // assign to &data to output
        //     return true; // return success
        // };
        
        // template <typename T>
        // bool storeParam (const string paramPath, const string &paramName, T data, const char* typeStr) // throw(ConfigException)
        // {
        //     string fullParamPath = paramPath + "." + paramName;

        //     // convert data to a string
        //     string dataStr;
        //     try
        //     {
        //         dataStr = boost::lexical_cast<string>(data);
        //     }
        //     catch (const boost::bad_lexical_cast &blcExc) // conversion to string failed
        //     {
        //         // throw an error?
        //         // log reason for the error
        //         return false;
        //     }

        //     // Get this parameter's current entry:
        //     parameter<string> pStruct = storageManager.accessEntry(fullParamPath);

        //     // Check the type of the parameter
        //     if (pStruct.type.compare(typeStr) != 0) return false;

        //     // Change the parameter
        //     pStruct.value = dataStr;

        //     // Store the new parameter data into the config system.
        //     bool success = storageManager.editEntry(paramPath, pStruct);
        //     // bool success = storageManager.editEntry(paramPath, dataStr, std::string(typeStr));

        //     return success;
        // };
    };
}
#endif
