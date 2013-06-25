/*! @file ConfigManager.h
    @brief Defines the main ConfigManager class.
    
    @class ConfigSystem::ConfigManager
    @brief The interface between the configuration system and the other modules.
    
    A single ConfigManager instance resides on the NUBlackboard.
    It is through this instance that any other system component is intended to
    access/modify its configuration.
    Any object can access the config system, but if an object is to be notified
    of changes made to parameters in the config system that could affect its
    configuration, it should inherit from 'ConfigSystem::Configurable', and use
    'ConfigManager::AddConfigObject(Configurable*)' to add itself to the list
    of objects that the ConfigManager 'manages'.
    The ConfigManager updates the objects it manages on every iteration of the
    see-think thread (updating only those that need updating).
    
    Note: Creating more that one ConfigManager will cause errors in the
          config system's persistant store (i.e. not all changes to
          config parameters would be saved).
    
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

#include <vector>
#include <string>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/lexical_cast.hpp>

#include "ConfigStorageManager.h"
#include "ConfigTree.h"
#include "Configurable.h"

namespace ConfigSystem
{
    class ConfigManager
    {
    public:
        /*! @brief Creates a ConfigManager and loads the initial configuration
         *         specified.
         *  @param config_name The name of the initial configuration to load.  
         */
        ConfigManager(std::string config_name = "defaultConfig");

        /*! @brief Destroys this ConfigManager and deletes it's ConfigStore
         *         and current ConfigTree.
         */
        ~ConfigManager();
        
        /*! @brief  Loads a configuration with the given name.
         *  @param  config_name The name of the configuration to load.
         *  @return Returns whether or not the load succeeded.
         */
        bool LoadConfiguration(std::string config_name);
        
        /*! @brief  Saves the current configuration.
         *  @param  config_name The name to give the saved configuration.
         *  @return Returns whether or not the save succeeded.
         */
        bool SaveConfiguration(std::string config_name);
        
        /*!
         *  @brief Propogates changes made to parameters in the config system
         *         since the last call to 'UpdateConfiguration()' to the
         *         config_objects that depend on those parameters by calling
         *         their 'Configurable::updateConfig(...)' methods.
         * 
         *         This method is called once in every iteration of the main 
         *         loop in the run() method of the See-Think thread.
         */
        void UpdateConfiguration();
        
        /*! @brief  Sets the set of objects that the ConfigManager will
         *          auto-update, and calls each of their 
         *          'Configurable::loadConfig()' methods.
         *  @param  config_objects The objects to set.
         *  @return Returns whether or not the operation succeeded.
         */
        bool SetConfigObjects(std::vector<Configurable*> config_objects);

        /*! @brief  Adds the given configurable object to the set of objects
         *          that the ConfigManager will auto-update, and calls its 
         *          'Configurable::loadConfig()' method.
         *  @param  config_object The object to add.
         *  @return Returns whether or not the operation succeeded.
         *          (returns false if config_object is NULL)
         */
        bool AddConfigObject(Configurable* config_object);
        
        /*! @brief Creates a new parameter with the given name, stored at the 
         *         given path, and having the given initial value.
         *         (An attempt to 'create' an existing parameter will fail)
         *  @param param_path Path to the parameter to create.
         *  @param param_name Name of the parameter to create.
         *  @param initial_value The initial value for the parameter.
         *  @return Whether the operation was successful.
         */
        template<typename T>
        bool CreateParam(
            const std::string &param_path,
            const std::string &param_name,
            T initial_value);
        
        /*! @brief Deletes the named parameter stored at the given path.
         *         (An attempt to delete a locked parameter will fail)
         *         Note: Will delete any path (i.e. not individual parameters).
         *  @param param_path Path to the parameter to delete.
         *  @param param_name Name of the parameter to delete.
         *  @return Whether the operation was successful.
         */
        bool DeleteParam(
            const std::string &param_path,
            const std::string &param_name);
        
        /*! @brief Locks the named parameter stored at the given path.
         *         Attempts to modify or delete locked parameters will fail.
         *  @param param_path Path to the parameter to lock.
         *  @param param_name Name of the parameter to lock.
         *  @return Whether the operation was successful.
         */
        bool LockParam(
            const std::string &param_path,
            const std::string &param_name);

        /*! @brief Unlocks the named parameter stored at the given path.
         *         Attempts to modify locked parameters will fail.
         *  @param param_path Path to the parameter to unlock.
         *  @param param_name Name of the parameter to unlock.
         *  @return Whether the operation was successful.
         */
        bool UnlockParam(
            const std::string &param_path,
            const std::string &param_name);

        /*! @brief Sets the descriptpo of the named parameter stored at the given path.
         *         Attempts to modify locked parameters will fail.
         *  @param param_path Path to the parameter to have its description set.
         *  @param param_name Name of the parameter to have its description set.
         *  @return Whether the operation was successful.
         */
        bool SetDescription(
        const std::string &param_path,
        const std::string &param_name,
        const std::string &paramDesc);

        /*! @brief Reads a value stored at the given path in the current 
         *         configuration.
         *  @param param_path Path to the desired parameter.
         *  @param data variable in which to store the data retrieved.
         *  @return Whether the operation was successful.
         */
        template<typename T>
        bool ReadValue(
            const std::string &param_path,
            const std::string &param_name,
            T* value);

        /*! @brief Sets the value of the named parameter in the given path
         *         to the given value.
         *  @param  param_path Path of the parameter.
         *  @param  param_name Name of the parameter.
         *  @param  data The value to set.
         *  @return Whether the operation was successful.
         */
        template<typename T>
        bool SetValue(
            const std::string &param_path,
            const std::string &param_name,
            T value);

        /*! @brief     Stores the range in the current configuration 
         *             at the given path.
         *  @param     "param_path" Path at which to store the parameter.
         *  @param     "param_name" Name of the parameter to be stored.
         *  @param     "range" The data to store.
         *  @return Whether the operation was successful.
         */
        bool SetRange(
            const std::string &param_path, 
            const std::string &param_name, 
            ConfigRange<double> &range);

        bool SetRange(
            const std::string &param_path, 
            const std::string &param_name, 
            ConfigRange<long> &range);
        
        /*! @brief     Reads the ranges in the current configuration 
         *             at the given path.
         *  @param     "param_path" Path at which to store the parameter.
         *  @param     "param_name" Name of the parameter to be stored.
         *  @param     "range" The data to store.
         *  @return Whether the operation was successful.
         */                        
        bool ReadRange(
            const std::string &param_path, 
            const std::string &param_name,
            ConfigRange<double> *range);

        bool ReadRange(
            const std::string &param_path,
            const std::string &param_name,
            ConfigRange<long> *range);
        
    private:
        /*! The Configuration System's storage manager. */
        ConfigStorageManager *_configStore;

        /*! The config tree that stores the configuration system's current
         *  configuration. */
        ConfigTree *_currConfigTree;

        /*! A list of configurable objects to manage.
         *  If a change is made to a parameter within the base path of one of
         *  these objects, the ConfigManager will notify that object of the
         *  change (by calling that object's 'updateConfig()' method). */
         // This should be a std::unordered_set
        std::vector<Configurable*> _configObjects;

        /*! @brief     Update all config_objects that depend on the given 
         *             parameter.
         *  @param     param_path Path to check.
         *  @return Whether the operation was successful.
         */  
        void updateConfigObjects();

        /*! @brief     Marks all config_objects whose base path contains the
         *             given path as having had their configurations modified.
         *  @param     param_path Path to check.
         *  @return Whether the operation was successful.
         */  
        void markConfigObjects(
            const std::string &param_path,
            const std::string &param_name);

        /*! @brief Reconfigures the config_objects by calling 
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
