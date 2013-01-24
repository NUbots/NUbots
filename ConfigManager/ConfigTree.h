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


        bool addPtreeValueandRangeToParam(ptree fromPtree, ConfigParameter &toParam);
        
        template<typename T>
        bool addValueToParam(ptree fromPtree, ConfigParameter &toParam)
        {
            T v = fromPtree.get<T>("value");
            if(!toParam.setValue(v)) return false;
            return true;
        }
        
        template<typename T>
        bool addVectorValueToParam1D(ptree from_ptree, ConfigParameter &to_param)
        {
        	std::vector<T> vector_value;
        	
        	try
		    {
		    	//Retrieve values of type T from vector in tree and place in vector.
		    	BOOST_FOREACH(ptree::value_type &child, from_ptree.get_child("value"))
		    	{
		    		vector_value.push_back(child.second.get<T>(""));
		    	}
		    	
		    	//Sets the value in the ConfigParameter object
		    	to_param.setValue(vector_value);
        	}
        	catch(std::exception &e)
        	{
        		return false;
        		std::cout << "ERROR: " << e.what() << std::endl;
        	}
        	
        	return true;
        }
        
        template<typename T>
        bool addRangeToParam(ptree fromPtree, ConfigParameter &toParam)
        {
            // Read range
            std::string lBStr = fromPtree.get("range.lBound", "none");
            std::string uBStr = fromPtree.get("range.uBound", "none");
            BoundType range_lBound = stringToBoundType(lBStr);
            BoundType range_uBound = stringToBoundType(uBStr);

            std::string outsideStr = fromPtree.get("range.outside", "false");
            bool range_outside  = (outsideStr.compare("true") == 0);

            std::string autoClipStr = fromPtree.get("range.autoClip", "false");
            bool range_autoClip  = (autoClipStr.compare("true") == 0);

            T range_min = fromPtree.get("range.min", 0.0);
            T range_max = fromPtree.get("range.max", 0.0);
            ConfigRange<T> cr
            (
                range_min,
                range_max,
                range_outside,
                range_autoClip,
                range_lBound,
                range_uBound
            );
            toParam.setRange(cr);
        }


        bool addParamValueandRangeToPtree(ConfigParameter fromParam, ptree &toPtree);
        
        // Note: should explicitly specify template params + put implementations in .cpp
        template<typename T>
        bool addValueToPtree(ConfigParameter fromParam, ptree &toPtree)
        {
            T v; 
            if(!fromParam.getValue(v)) return false;
            toPtree.put("value", v);
            return true;
        }
        
        template<typename T>
        bool addVectorValueToPtree1D(ConfigParameter from_param, ptree &to_ptree)
        {
        	ptree children;
        	ptree child;
        	std::vector<T> retrieved_vector;
        	//Retrieve vector from the CP object
        	if( !from_param.getValue(retrieved_vector) ) return false;
        	
        	try
        	{
		    	BOOST_FOREACH(const T &value, retrieved_vector)
				{
					child.put("", value);
					children.push_back(std::make_pair("", child));
				}
			
				to_ptree.add_child("value", children);
			}
			catch(std::exception &e)
			{
				std::cout << "ERROR: " << e.what() << std::endl;
				
				return false;
			}
			
			return true;
        }
        
        
        template<typename T>
        bool addRangeToPtree(ConfigParameter fromParam, ptree &toPtree)
        {
            ConfigRange<T> r;
            if(!fromParam.getRange(r)) return false;

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

            return true;
        }
        
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
            
            
        /*! 
         *  @brief  Gets a 1 dimensional vector from the ConfigTree
         *  @param	"param_path" Base path of parameter.
         *  @param 	"param_name" The parameter's name.
         *  @param 	"data" The retrieved parameter.
         *  @return Returns whether the operation was successful.
         */
        bool getVectorParam1D(const std::string param_path, const std::string param_name, 
        						ConfigParameter &data);
        						
        						
        
        ptree getRoot();
    };
}

#endif
