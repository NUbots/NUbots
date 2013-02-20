/*! @file ConfigManager.h
    @brief Defines the main ConfigManager class.
    
    @class ConfigManager
    @brief The interface between the configuration system and the other modules.
    
    A single ConfigManager instance resides on the NUBlackboard.
    It is through this instance that any other system component is intended to
    access/modify its configuration.
    Any object can access the config system, but if an object is to be notified
    of changes made to parameters in the config system that could affect its
    configuration, it should inherit from 'ConfigSystem::Configurable', and use
    'ConfigManager::addConfigObject(Configurable*)' to add itself to the list
    of objects that the ConfigManager 'manages'.
    The ConfigManager updates the objects it manages on every iteration of the
    see-think thread (updating only those that need updating).
    
    Note: Creating more that one ConfigManager will cause errors in the
          config system's persistant store (i.e. not all changes
          config parameters will be saved).
    
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
        ConfigManager(std::string configName = "defaultConfig");
        

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
         *         This method is called once in every iteration of the main 
         *         loop in the run() method of the See-Think thread.
         */
        void updateConfiguration();
        

        /*! @brief  Sets the set of objects that the ConfigManager will
         *          auto-update, and calls each of their 
         *          'Configurable::loadConfig()' methods.
         *  @param  configObjects The objects to set.
         *  @return Returns whether or not the operation succeeded.
         */
        bool setConfigObjects(std::vector<Configurable*> configObjects);

        /*! @brief  Adds the given configurable object to the set of objects
         *          that the ConfigManager will auto-update, and calls its 
         *          'Configurable::loadConfig()' method.
         *  @param  configObject The object to add.
         *  @return Returns whether or not the operation succeeded.
         *          (returns false if configObject is NULL)
         */
        bool addConfigObject(Configurable* configObject);
        
        
        
        /*! @brief Creates a new parameter with the given name, stored at the 
         *         given path, and having the given initial value.
         *         (An attempt to 'create' an existing parameter will fail)
         *  @param paramPath Path to the parameter to create.
         *  @param paramName Name of the parameter to create.
         *  @return Whether the operation was successful.
         */
        template<typename T>
        bool createParam(
            const std::string &paramPath,
            const std::string &paramName,
            T initialValue
            );
        
        /*! @brief Deletes the named parameter stored at the given path.
         *         (An attempt to delete a locked parameter will fail)
         *         Note: Will delete any path (i.e. not individual parameters).
         *  @param paramPath Path to the parameter to delete.
         *  @param paramName Name of the parameter to delete.
         *  @return Whether the operation was successful.
         */
        bool deleteParam(
            const std::string &paramPath,
            const std::string &paramName
            );
        
        /*! @brief Locks the named parameter stored at the given path.
         *         Attempts to modify or delete locked parameters will fail.
         *  @param paramPath Path to the parameter to lock.
         *  @param paramName Name of the parameter to lock.
         *  @return Whether the operation was successful.
         */
        bool lockParam(
            const std::string &paramPath,
            const std::string &paramName
            );

        /*! @brief Unlocks the named parameter stored at the given path.
         *         Attempts to modify locked parameters will fail.
         *  @param paramPath Path to the parameter to unlock.
         *  @param paramName Name of the parameter to unlock.
         *  @return Whether the operation was successful.
         */
        bool unlockParam(
            const std::string &paramPath,
            const std::string &paramName
            );


        /*! @brief Sets the descriptpo of the named parameter stored at the given path.
         *         Attempts to modify locked parameters will fail.
         *  @param paramPath Path to the parameter to have its description set.
         *  @param paramName Name of the parameter to have its description set.
         *  @return Whether the operation was successful.
         */
        bool setParamDescription(
        const std::string &paramPath,
        const std::string &paramName,
        const std::string &paramDesc
        );


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


        /*! @brief     Stores the ranges in the current configuration 
         *             at the given path.
         *  @param     "param_path" Path at which to store the parameter.
         *  @param     "param_name" Name of the parameter to be stored.
         *  @param     "range" The data to store.
         *  @return Whether the operation was successful.
         */
        bool storeRange  (
            const std::string &paramPath, 
            const std::string &paramName, 
            ConfigRange<double> &range
            );

        bool storeRange    (
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
        bool readRange   (
            const std::string &paramPath, 
            const std::string &paramName,
            ConfigRange<double> &range
            );

        bool readRange     (
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
         *  @param     paramPath Path to check.
         *  @return Whether the operation was successful.
         */  
        void updateConfigObjects();
        // void updateConfigObjects(
        //     const std::string &paramPath,
        //     const std::string &paramName
        //     );

        /*! @brief     Marks all configObjects whose base path contains the
         *             given path as having had their configurations modified.
         *  @param     paramPath Path to check.
         *  @return Whether the operation was successful.
         */  
        void markConfigObjects(
            const std::string &paramPath,
            const std::string &paramName
            );

        /*! @brief Reconfigures the configObjects by calling 
         *         'reconfigureConfigObject(Configurable*)'
         *         on each of them.
         */  
        void reconfigureConfigObjects();

        /*! @brief Reconfigures a Configurable object by calling
         *         'loadConfig()' on it, and by marking it as having a
         *         recent configuration.
         */  
        void reconfigureConfigObject(Configurable* c);
    };
}
#endif
