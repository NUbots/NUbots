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
        
        
        
        //1D VECTORS (any other vector types, like string?)
        /*! @brief     Reads a vector stored at the given path in the current configuration.
         *  @param     "param_path" Path to the desired parameter.
         *     @param     "param_name" Name of the desired parameter.
         *  @param     "data" Variable in which to store the data retrieved.
         *  @return Whether the operation was successful.
         */
        bool readLongVectorValue1D(const string &param_path, const string &param_name, 
                                    std::vector<long> &data);
        bool readLongVectorValue2D(const string &param_path, const string &param_name, 
        							std::vector< std::vector<long> > &data);
                                    
        bool readDoubleVectorValue1D(
            const string &param_path, 
            const string &param_name, 
            std::vector<double> &data
            );
        bool readDoubleVectorValue2D(
            const string &param_path, 
            const string &param_name, 
            std::vector<std::vector<double> > &data
            );                     
        
        
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
        
        
        //1D VECTORS (any other vector types, like string?)
        /*! @brief     Stores the given vector in the current configuration at the given path.

         *  @param     "param_path" Path at which to store the parameter.
         *  @param     "param_name" Name of the parameter to be stored.
         *  @param     "data" The data to store.
         *  @return Whether the operation was successful.
         */
                                    
        bool storeLongVectorValue1D(const string &param_path, const string &param_name, 
                                    std::vector<long> data);
                                    
		bool storeLongVectorValue2D(const string &param_path, const string &param_name, 
                                    std::vector< std::vector<long> > data);                             
                                    
        bool storeDoubleVectorValue1D(
            const string &param_path, 
            const string &param_name, 
            std::vector<double> data
            );

        bool storeDoubleVectorValue2D(
            const string &param_path, 
            const string &param_name, 
            std::vector<std::vector<double> > data
            );

        
        /* Ranges of vector types shouldn't change from what we currently have? Unless people want 
        multiple ranges for different values in the vector I suppose ... just an idea. Leaving it 
        for now. :P */
        
        
        /*! @brief     Stores the ranges in the current configuration at the given path.
         *  @param     "param_path" Path at which to store the parameter.
         *  @param     "param_name" Name of the parameter to be stored.
         *  @param     "range" The data to store.
         *  @return Whether the operation was successful.
         */
        bool storeDoubleRange  (const string &paramPath, 
                                const string &paramName, 
                                ConfigRange<double> &range);
        bool storeLongRange    (const string &paramPath, 
                                const string &paramName, 
                                ConfigRange<long> &range);
                                
                                
        /*! @brief     Reads the ranges in the current configuration at the given path.
         *  @param     "param_path" Path at which to store the parameter.
         *  @param     "param_name" Name of the parameter to be stored.
         *  @param     "range" The data to store.
         *  @return Whether the operation was successful.
         */                        
        bool readDoubleRange   (const string &paramPath, 
                                const string &paramName, 
                                ConfigRange<double> &range);
        bool readLongRange     (const string &paramPath, 
                                const string &paramName, 
                                ConfigRange<long> &range);
        
    private:
        //! The Configuration System's storage manager.
        ConfigStorageManager    *_configStore   ;

        //! The config tree that stores the configuration system's current
        //! configuration.
        ConfigTree              *_currConfigTree;
    };
}
#endif
