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
#include "Configurable.h"

#include <vector>
#include <string>

namespace ConfigSystem
{
    class ConfigManager
    {
    public:
        /*!
         *  @brief Creates a configManager and loads the initial configuration
         *         specified.
         *  @param configName The name of the initial configuration to load.  
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
        
        /*!
         *  @brief Propogates changes made to parameters in the config system
         *         since the last call to 'updateConfiguration()' to the
         *         configObjects that depend on those parameters by calling
         *         their 'Configurable::updateConfig(...)' methods.
         * 
         *         This method should be called once on each iteration of the
         *         main loop See-Think thread.
         */
        void updateConfiguration();
        

        /*! @brief  Sets the set of objects that the ConfigManager will
         *          auto-update, and calls each of their 
         *          'Configurable::loadConfig()' methods.
         *  @param  configObjects The objects to set.
         *  @return Returns whether or not the operation succeeded.
         */
        bool setConfigObjects(std::vector<Configurable*> configObjects);
        

        /*! @brief Reads a value stored at the given path in the current 
         *         configuration.
         *  @param paramPath Path to the desired parameter.
         *  @param data variable in which to store the data retrieved.
         *  @return Whether the operation was successful.
         */
        template<typename T>
        bool readValue (
            const std::string &paramPath,
            const std::string &paramName,
            T &data
            );


        // Old (explicitly named) interface.
        // This should probably be removed soon?
        bool readLongValue   (
            const std::string &paramPath, 
            const std::string &paramName, 
            long   &data
            );
        bool readDoubleValue (
            const std::string &paramPath, 
            const std::string &paramName, 
            double &data
            );
        bool readStringValue (
            const std::string &paramPath, 
            const std::string &paramName, 
            string &data
            );
        bool readLongVectorValue1D(
            const std::string &param_path, 
            const std::string &param_name, 
            std::vector<long> &data
            );
        bool readLongVectorValue2D(
            const std::string &param_path, 
            const std::string &param_name, 
            std::vector<std::vector<long> > &data
            ); 
        bool readLongVectorValue3D(
            const std::string &param_path, 
            const std::string &param_name, 
            std::vector<std::vector<std::vector<long> > > &data
            );    
        bool readDoubleVectorValue1D(
            const std::string &param_path, 
            const std::string &param_name, 
            std::vector<double> &data
            );
        bool readDoubleVectorValue2D(
            const std::string &param_path, 
            const std::string &param_name, 
            std::vector<std::vector<double> > &data
            ); 
        bool readDoubleVectorValue3D(
            const std::string &param_path, 
            const std::string &param_name, 
            std::vector<std::vector<std::vector<double> > > &data
            );                   
        
        
        /*! @brief Stores the given value in the current configuration 
         *         at the given path.
         *  @param  paramPath Path at which to store the parameter.
         *  @param  paramName Name of the parameter to be stored.
         *  @param  data The data to store.
         *  @return Whether the operation was successful.
         */
        template<typename T>
        bool storeValue(
            const std::string &paramPath,
            const std::string &paramName,
            T data
            );


        // Old (explicitly named) interface.
        // This should probably be removed soon?
        bool storeLongValue   (
            const std::string &paramPath,
            const std::string &paramName,
            long   data
            );
        bool storeDoubleValue (
            const std::string &paramPath,
            const std::string &paramName,
            double data
            );
        bool storeStringValue (
            const std::string &paramPath,
            const std::string &paramName,
            string data
            );             
        bool storeLongVectorValue1D(
            const std::string &paramPath, 
            const std::string &paramName, 
            std::vector<long> data
            );
        bool storeLongVectorValue2D(
            const std::string &paramPath, 
            const std::string &paramName, 
            std::vector<std::vector<long> > data
            );
        bool storeLongVectorValue3D(
            const std::string &paramPath, 
            const std::string &paramName, 
            std::vector<std::vector<std::vector<long> > > data
            );                            
        bool storeDoubleVectorValue1D(
            const std::string &paramPath, 
            const std::string &paramName, 
            std::vector<double> data
            );
        bool storeDoubleVectorValue2D(
            const std::string &paramPath, 
            const std::string &paramName, 
            std::vector<std::vector<double> > data
            );
        bool storeDoubleVectorValue3D(
            const std::string &paramPath, 
            const std::string &paramName, 
            std::vector<std::vector<std::vector<double> > > data
            );
        
        
        /*! @brief     Stores the ranges in the current configuration 
         *             at the given path.
         *  @param     "param_path" Path at which to store the parameter.
         *  @param     "param_name" Name of the parameter to be stored.
         *  @param     "range" The data to store.
         *  @return Whether the operation was successful.
         */
        bool storeDoubleRange  (
            const std::string &paramPath, 
            const std::string &paramName, 
            ConfigRange<double> &range
            );

        bool storeLongRange    (
            const std::string &paramPath, 
            const std::string &paramName, 
            ConfigRange<long> &range
            );
                                
                                
        /*! @brief     Reads the ranges in the current configuration 
         *             at the given path.
         *  @param     "param_path" Path at which to store the parameter.
         *  @param     "param_name" Name of the parameter to be stored.
         *  @param     "range" The data to store.
         *  @return Whether the operation was successful.
         */                        
        bool readDoubleRange   (
            const std::string &paramPath, 
            const std::string &paramName,
            ConfigRange<double> &range
            );

        bool readLongRange     (
            const std::string &paramPath,
            const std::string &paramName, 
            ConfigRange<long> &range
            );
        

    private:
        //! The Configuration System's storage manager.
        ConfigStorageManager    *_configStore   ;


        //! The config tree that stores the configuration system's current
        //! configuration.
        ConfigTree              *_currConfigTree;


        //! A list of configurable objects to manage.
        //! If a change is made to a parameter within the base path of one of
        //! these objects, the ConfigManager will notify that object of the
        //! change (by calling that object's 'updateConfig()' method).
        std::vector<Configurable*> _configObjects;


        /*! @brief     Update all configObjects that depend on the given 
         *             parameter.
         *  @param     "param_path" Path to check.
         *  @return Whether the operation was successful.
         */  
        void updateConfigObjects(
            const std::string &paramPath,
            const std::string &paramName
            );

        /*! @brief Reconfigures the configObjects by calling 'loadConfig()'
         *         on each of them.
         */  
        void reconfigureConfigObjects();
    };
}
#endif
