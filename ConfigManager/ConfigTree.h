/*! 
    @file     ConfigTree.h
    @brief     Header file for the ConfigTree class. 
    
    @class  ConfigTree
    @brief  Stores configuration data in a tree structure, allowing it to be 
            easily accessed using dot separated paths. The ConfigStorageManager
            saves and loads ConfigTrees from disk. The ConfigManager's 
            interface to the configuration system simply wraps access to a 
            ConfigTree containing the current configuration.
    
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

#ifndef ConfigTree_def
#define ConfigTree_def

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/foreach.hpp>
#include <string>
#include <exception>

#include "ConfigParameter.h"
#include "ConfigStorageManager.h"

// using ConfigSystem::ConfigStorageManager;
using boost::property_tree::ptree;

namespace ConfigSystem
{
    class ConfigTree
    {
    private:
        //! 
        ptree _treeRoot; // could be a pointer, but probably doesn't matter?
        
        //! 
        std::string configName;

        /*! 
         *  @brief  Converts a property tree (with the appropriate structure)
         *          into a parameter object.
         *  @param fromPtree    The ptree to convert into a parameter.
         *  @param toParam      The resulting parameter object.
         *  @return Returns whether the conversion succeeded (i.e. if the
         *          minimum set of required fields/keys were not present).
         */
        bool paramFromPtree(ptree fromPtree, ConfigParameter &toParam);

        /*! 
         *  @brief  Converts a parameter object into a property tree that fully
         *          represents the parameter.
         *  @param fromParam  The  parameter object to convert into a ptree.
         *  @param toPtree    The resulting ptree.
         *  @return Returns whether the conversion succeeded (i.e. if the
         *          minimum set of required fields/keys were not present).
         */
        bool ptreeFromParam(ConfigParameter fromParam, ptree &toPtree);

        /*! 
         *  @brief  Converts a 'conceptual' path and name into the config 
        *          tree into a full path that refers to the intended data.
         *  @param paramPath    The path to the variable.
         *  @param paramName    The variable's name
         *  @return A string containing the full path.
         */
        std::string makeFullPath(
            const std::string paramPath,
            const std::string paramName
            );

    public:
        ConfigTree(ptree root);
        ~ConfigTree();

        /*! 
         *  @brief  Gets a parameter from the ConfigTree
         *  @param paramPath The base path of the parameter.
         *  @param paramName The parameter's name.
         *  @param data The parameter to get.
         *  @return Returns whether the operation was successful.
         */
        bool getParam (
            const std::string paramPath,
            const std::string paramName,
            ConfigParameter &data
            );

        /*! 
         *  @brief  Stores a parameter into the ConfigTree
         *  @param paramPath The base path of the parameter.
         *  @param paramName The parameter's name.
         *  @param data The parameter to store.
         *  @return Returns whether the operation was successful.
         */
        bool storeParam (
            const std::string paramPath, 
            const std::string paramName, 
            ConfigParameter data
            );

        ptree getRoot();
    };
}

#endif
