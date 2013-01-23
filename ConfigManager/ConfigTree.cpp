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

        std::string typStr = fromPtree.get<std::string>("type");
        value_type vt = stringToValueType(typStr);
        
        toParam = ConfigParameter(vt);

        double v = fromPtree.get<double>("value");
        toParam.setValue_double(v);

        toParam.setDescription(fromPtree.get("desc", ""));

        std::string modStr = fromPtree.get("modified", "false");
        toParam.setModified(modStr.compare("true") == 0);

        std::string lockStr = fromPtree.get("locked", "false");
        toParam.setLocked(lockStr.compare("true") == 0);

        double range_min = fromPtree.get<double>("range.min");
        double range_max = fromPtree.get<double>("range.max");

        std::string lBStr = fromPtree.get<std::string>("range.lBound");
        std::string uBStr = fromPtree.get<std::string>("range.uBound");
        BoundType range_lBound = stringToBoundType(lBStr);
        BoundType range_uBound = stringToBoundType(uBStr);

        std::string outsideStr = fromPtree.get("range.outside", "false");
        bool range_outside  = (outsideStr.compare("true") == 0);

        std::string autoClipStr = fromPtree.get("range.autoClip", "false");
        bool range_autoClip  = (autoClipStr.compare("true") == 0);

        ConfigRange<double> cr
            (
                range_min,
                range_max,
                range_outside,
                range_autoClip,
                range_lBound,
                range_uBound
            );

        toParam.setRange(cr);

        return true;
    }

    bool ConfigTree::ptreeFromParam(ConfigParameter fromParam, ptree &toPtree)
    {
        CONFIGSYS_DEBUG_CALLS;

        toPtree = ptree();

        toPtree.put("desc", fromParam.getDescription());
        
        std::string modStr = (fromParam.isModified())? "true" : "false";
        toPtree.put("modified", modStr);

        std::string lockStr = (fromParam.isLocked())? "true" : "false";
        toPtree.put("locked", lockStr);

        std::string typStr = makeValueTypeString(fromParam.getType());
        toPtree.put("type", typStr);
        
        
        double v;
        fromParam.getValue_double(v);
        toPtree.put("value", v);

        ConfigRange<double> r;
        if(fromParam.getRange_double(r))
        {
            toPtree.put("range.min"     , r.getMin());
            toPtree.put("range.max"     , r.getMax());

            std::string lBStr = makeBoundTypeString(r.getLowerBoundType());
            std::string uBStr = makeBoundTypeString(r.getUpperBoundType());
            toPtree.put("range.lBound"  , lBStr);
            toPtree.put("range.uBound"  , uBStr);
            
            std::string outsideStr = (r.getOutside())? "true" : "false";
            toPtree.put("range.outside" , outsideStr);

            std::string autoClipStr = (r.getAutoClip())? "true" : "false";
            toPtree.put("range.autoClip", autoClipStr);
        }

        return true;
    }

// "Camera": {
//     "Sharpness": {
//         "desc": "sharpness",
//         "range": {
//             "min": "0",
//             "max": "255",
//             "lBound": "CLOSED",
//             "outside": "false",
//             "uBound": "CLOSED"
//         },
//         "value": "150",
//         "type": "float"
//     },

    
    ptree ConfigTree::getRoot()
    {
        return _treeRoot;
    }
}
