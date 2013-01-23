/*! 
	@file 	ConfigTree.cpp
    @brief 	Implementation file for the ConfigTree class. 
    
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
#include <boost/property_tree/exceptions.hpp>
#include <string>
#include <exception>

#include "ConfigStorageManager.h"
#include "ConfigTree.h"

using std::string;
// using ConfigSystem::ConfigStorageManager;

using boost::property_tree::ptree;

namespace ConfigSystem
{
	ConfigTree::ConfigTree(ptree root)
    {
        CONFIGSYS_DEBUG_CALLS;

        _treeRoot = root;
    }

    ConfigTree::~ConfigTree()
    {
        CONFIGSYS_DEBUG_CALLS;
    }


    std::string ConfigTree::makeFullPath(
            const std::string paramPath,
            const std::string paramName
            )
    {
        return paramPath + "." + paramName;
    }


	bool ConfigTree::getParam (
            const std::string paramPath, 
            const std::string paramName, 
            ConfigParameter &data
            )
	{
		CONFIGSYS_DEBUG_CALLS;
        
        // Indicates successful retrieval
        bool success = false;

        // Create the full path to the desired parameter
        std::string fullPath = makeFullPath(paramPath, paramName);

		try
		{
            // Get the subtree representing the desired parameter.
            ptree paramSubtree = _treeRoot.get_child(fullPath);

            // Convert the parameter subtree into a parameter object.
            success = paramFromPtree(paramSubtree, data);
		}
		catch (boost::property_tree::ptree_error e)
		{
			std::cout   << "ConfigTree::getParam(...): ACCESS ERROR:" 
                        << e.what() 
                        << std::endl;
            return false;
		}

		return success;
	}

    bool ConfigTree::storeParam (
            const std::string paramPath, 
            const std::string paramName, 
            ConfigParameter data
            )
    {
        CONFIGSYS_DEBUG_CALLS;
        
        // Indicates successful storage
        bool success = false;

        // Create the full path to the desired parameter
        std::string fullPath = makeFullPath(paramPath, paramName);

        try
        {
            // Convert the parameter object into a parameter subtree.
            ptree paramSubtree;
            success = ptreeFromParam(data, paramSubtree);
            if(!success) return false;
            
            // Put the subtree representing the converted parameter into the
            // config tree.
            _treeRoot.put_child(fullPath, paramSubtree);
        }
        catch (boost::property_tree::ptree_error e)
        {
            std::cout   << "ConfigTree::getParam(...): ACCESS ERROR:" 
                        << e.what() 
                        << std::endl;
            return false;
        }

        return success;
    }

    bool ConfigTree::paramFromPtree(ptree fromPtree, ConfigParameter &toParam)
    {
        CONFIGSYS_DEBUG_CALLS;

        toParam = ConfigParameter(vt_double);

        double v = fromPtree.get<double>("value");
        toParam.setValue_double(v);

        return true;
    }

    bool ConfigTree::ptreeFromParam(ConfigParameter fromParam, ptree &toPtree)
    {
        CONFIGSYS_DEBUG_CALLS;

        double v;
        fromParam.getValue_double(v);

        toPtree = ptree();
        toPtree.put("value", v);

        return true;
    }
    
    ptree ConfigTree::getRoot()
    {
        return _treeRoot;
    }
}
