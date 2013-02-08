/*! 
    @file     ConfigTree.cpp
    @brief     Implementation file for the ConfigTree class. 
    
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


    std::string ConfigTree::makeFullParamPath(const std::string paramPath, const std::string paramName)
    {
        return paramPath + "." + paramName;
    }
    
    std::string ConfigTree::makeFullPath(const std::string paramPath)
    {
        return paramPath;
    }

    bool ConfigTree::getParam (
            const std::string paramPath, 
            const std::string paramName, 
            ConfigParameter &data
            )
    {
        CONFIGSYS_DEBUG_CALLS;
        
        // std::cout << "IN getParam" << std::endl;

        // Indicates successful retrieval
        bool success = false;

        // Create the full path to the desired parameter
        std::string fullPath = makeFullParamPath(paramPath, paramName);
        
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
        std::string fullPath = makeFullParamPath(paramPath, paramName);

        try
        {
            // Convert the parameter object into a parameter subtree.
            ptree paramSubtree;
            success = ptreeFromParam(data, paramSubtree);
            if(!success)
            {
                std::cout << "ConfigTree::getParam(...): "
                          << "Could not convert the given parameter into a property tree"
                          << " (the parameter may be invalid)." 
                          << std::endl;
                return false;
            }
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


    bool ConfigTree::deleteParam(
            const std::string &paramPath,
            const std::string &paramName
            )
    {
        CONFIGSYS_DEBUG_CALLS;
        
        std::string fullPath = makeFullPath(paramPath);

        try
        {
            // get the parent node
            ptree paramParent = _treeRoot.get_child(fullPath);

            // Delete the parameter.
            // (Note: Only erases 'direct children'.
            // i.e. paramParent.erase("myname") might erase something,
            //  but paramParent.erase("my.name") will never erase anything).
            int del_n = paramParent.erase(paramName);
            if(del_n == 0)
            {
                std::cout   << "ConfigTree::deleteParam(...): Nothing to erase"
                            << " (there's no '" << paramName << "' at the given path)."
                            << std::endl;
                return false; // return failure if nothing was erased
            }
            
            // put back the modified parent node
            _treeRoot.put_child(fullPath, paramParent);
        }
        catch (boost::property_tree::ptree_error e)
        {
            std::cout   << "ConfigTree::deleteParam(...): ACCESS ERROR:" 
                        << e.what() 
                        << std::endl;
            return false;
        }

        return true;
    }


    bool ConfigTree::paramFromPtree(ptree fromPtree, ConfigParameter &toParam)
    {
        CONFIGSYS_DEBUG_CALLS;

        std::string typStr = fromPtree.get<std::string>("type");
        value_type vt = stringToValueType(typStr);
        
        toParam = ConfigParameter(vt);

        toParam.setDescription(fromPtree.get("desc", ""));

        std::string modStr = fromPtree.get("modified", "false");
        toParam.setModified(modStr.compare("true") == 0);

        std::string lockStr = fromPtree.get("locked", "false");
        toParam.setLocked(lockStr.compare("true") == 0);

        return addPtreeValueandRangeToParam(fromPtree, toParam);
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


        value_type vt = fromParam.getType();
        std::string typStr = makeValueTypeString(vt);
        toPtree.put("type", typStr);
        
        return addParamValueandRangeToPtree(fromParam, toPtree);
    }

    bool ConfigTree::addPtreeValueandRangeToParam(ptree fromPtree, ConfigParameter &toParam)
    {
        CONFIGSYS_DEBUG_CALLS;

        std::string typStr = fromPtree.get<std::string>("type");
        value_type vt = stringToValueType(typStr);

        // std::cout << "ConfigTree::addPtreeValueandRangeToParam(...): vt = " 
        //           << makeValueTypeString(vt) << std::endl;

        bool success_v = false;
        bool success_r = true ;

        switch(vt)
        {
            case vt_string: 
                success_v = addValueToParam<std::string>(fromPtree, toParam); 
                break;
                
            case vt_double: 
                success_v = addValueToParam<double>(fromPtree, toParam); 
                break;
            
            case vt_bool: 
                success_v = addValueToParam<bool>(fromPtree, toParam); 
                break;
                
            case vt_long: 
                success_v = addValueToParam<long>(fromPtree, toParam); 
                break;
            
            case vt_1dvector_double:
                success_v = addVectorValueToParam1D<double>(fromPtree, toParam);
                break;
            case vt_2dvector_double:
                success_v = addVectorValueToParam2D<double>(fromPtree, toParam);
                break;
            case vt_3dvector_double:
                success_v = addVectorValueToParam3D<double>(fromPtree, toParam);
                break;
            
            case vt_1dvector_long:
                success_v = addVectorValueToParam1D<long>(fromPtree, toParam);
                break;
            case vt_2dvector_long:
                success_v = addVectorValueToParam2D<long>(fromPtree, toParam);
                break;
            case vt_3dvector_long:
                success_v = addVectorValueToParam3D<long>(fromPtree, toParam);
                break;
            default: 
                std::cerr << __PRETTY_FUNCTION__ 
                         << ": Invalid val_type" 
                         << makeValueTypeString(vt) << "'"
                         << "'" << std::endl;
                break;
        }
        
        switch(vt)
        {
            //cases fall through as they use the same - change later if necessary.
            case vt_double         :
            case vt_1dvector_double: 
            case vt_2dvector_double: 
            case vt_3dvector_double: 
                success_r = addRangeToParam<double>(fromPtree, toParam);
                break;
                
            case vt_bool         :
            case vt_long         :            
            case vt_1dvector_long: 
            case vt_2dvector_long: 
            case vt_3dvector_long: 
                success_r = addRangeToParam<long>(fromPtree, toParam);
                break;
            default: 
                std::cerr << __PRETTY_FUNCTION__ 
                         << ": Invalid val_type" 
                         << makeValueTypeString(vt)
                         << std::endl;
                break;
        }


        // std::cout << __PRETTY_FUNCTION__ << ": success_v = " << success_v << std::endl;
        // std::cout << __PRETTY_FUNCTION__ << ": success_r = " << success_r << std::endl;
        // std::cout << __PRETTY_FUNCTION__ << ( (success_r)? ": success" : ": failed" ) << std::endl;
        return success_v && success_r;
    }

    bool ConfigTree::addParamValueandRangeToPtree(ConfigParameter fromParam, ptree &toPtree)
    {
        CONFIGSYS_DEBUG_CALLS;

        bool success_v = false;
        bool success_r = true ;

        value_type vt = fromParam.getType();
        
        // std::cout << __PRETTY_FUNCTION__ << ": vt = " << makeValueTypeString(vt) << std::endl;
        
        // Add value
        switch(vt)
        {
            case vt_string         : 
                success_v = addValueToPtree<std::string> (fromParam, toPtree); 
                break;
                
            case vt_double         : 
                success_v = addValueToPtree<double> (fromParam, toPtree); 
                break;
                
            case vt_bool           : 
                success_v = addValueToPtree<bool> (fromParam, toPtree); 
                break;
                
            case vt_long           : 
                success_v = addValueToPtree<long> (fromParam, toPtree); 
                break;
                
            case vt_1dvector_double:
                success_v = addVectorValueToPtree1D<double>(fromParam, toPtree); 
                break;
             case vt_2dvector_double:
                success_v = addVectorValueToPtree2D<double>(fromParam, toPtree); 
                break;
             case vt_3dvector_double:
                success_v = addVectorValueToPtree3D<double>(fromParam, toPtree); 
                break;
                
            case vt_1dvector_long  : 
                success_v = addVectorValueToPtree1D<long>(fromParam, toPtree); 
                break;
            case vt_2dvector_long  : 
                success_v = addVectorValueToPtree2D<long>(fromParam, toPtree); 
                break;
            case vt_3dvector_long  : 
                success_v = addVectorValueToPtree3D<long>(fromParam, toPtree); 
                break;
            default: 
                std::cerr << __PRETTY_FUNCTION__ 
                         << ": Invalid val_type" 
                         << makeValueTypeString(vt)
                         << std::endl;
                success_v = false;
                break;
        }

        // Add range
        switch(vt)
        {
            case vt_double         :
            case vt_1dvector_double:
            case vt_2dvector_double:
            case vt_3dvector_double:
                success_r = addRangeToPtree<double>(fromParam, toPtree);
                break;
            case vt_bool         :
            case vt_long         :
            case vt_1dvector_long:
            case vt_2dvector_long:
            case vt_3dvector_long:
                success_r = addRangeToPtree<long>(fromParam, toPtree);
                break;
            default: 
                std::cerr << __PRETTY_FUNCTION__ 
                         << ": Invalid val_type" 
                         << makeValueTypeString(vt)
                         << std::endl;
                success_r = false;
                break;
        }
        
        // std::cout << __PRETTY_FUNCTION__ << ": success_v = " << success_v << std::endl;
        // std::cout << __PRETTY_FUNCTION__ << ": success_r = " << success_r << std::endl;
        // std::cout << __PRETTY_FUNCTION__ << ( (success_r)? ": success" : ": failed" ) << std::endl;
        return success_v && success_r;
    }

    
    ptree ConfigTree::getRoot()
    {
        return _treeRoot;
    }
}
