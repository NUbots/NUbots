/*! 
    @file 	ConfigParameter.cpp
    @brief 	This is the implementation file for the ConfigParameters objects of the configuration 
    		system for the NUbots.
    
    @class 	ConfigParameter
    @brief 	This class serves as an object for use when transferring parameters.
    
    @author Sophie Calland, Mitchell Metcalfe
    
  	Copyright (c) 2012 Sophie Calland, Mitchell Metcalfe
  
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

#include "ConfigParameter.h"

namespace ConfigSystem
{
	//Retrieves name.
    std::string ConfigParameter::getName() 
    {	
    	return _name; 
    }
    
    //Retrieves path (minus .name)
    std::string ConfigParameter::getPath() 
    {	
    	return _path; 
    }
    
    //Retrieves description
    std::string ConfigParameter::getDescription() 
    {	
    	return _desc; 
    }
    
    //Retrieves type
    value_type ConfigParameter::getType() 
    {
    	return param_value.val_type; 
    }
    
    
    
    
    
    void ConfigParameter::setName(std::string name) 
    { 
    	_name = name; 
    }
    
    void ConfigParameter::setPath(std::string path) 
    { 
    	_path = path; 
    }
    
    void ConfigParameter::setDescription(std::string desc) 
    { 
    	_desc = desc; 
    }
    
    void ConfigParameter::setType(value_type val_type) 
    {
        param_value.val_type = val_type;
    }
    
    
    
    bool ConfigParameter::getValue_bool(bool &value)
    {
        if(param_value.val_type != vt_bool) return false;
        
        value = *param_value.val_bool;
        
        return true;
    }
    
    bool ConfigParameter::getValue_long(long &value)
    {
        if(param_value.val_type != vt_long) return false;
        
        value = *param_value.val_long;
        
        return true;
    }

    bool ConfigParameter::getValue_double(double &value)
    {
        if(param_value.val_type != vt_double) return false;
        
        value = *param_value.val_double;
        
        return true;
    }

    bool ConfigParameter::getValue_string(std::string &value)
    {
        if(param_value.val_type != vt_string) return false;
        
        value = *param_value.val_string;
        
        return true;
    }

    bool ConfigParameter::getValue_1dvector_long(std::vector<long> &value)
    {
        if(param_value.val_type != vt_1dvector_long) return false;
        
        value = *param_value.val_1dvector_long;
        
        return true;
    }

    bool ConfigParameter::getValue_1dvector_double(std::vector<double> &value)
    {
        if(param_value.val_type != vt_1dvector_double) return false;
        
        value = *param_value.val_1dvector_double;
        
        return true;
    }

    bool ConfigParameter::setValue_bool(bool &value)
    {
        if(param_value.val_type != vt_bool) return false;
        
        *param_value.val_bool = value;
        
        return true;
    }

    bool ConfigParameter::setValue_long(long &value)
    {
        if(param_value.val_type != vt_long) return false;
        
        *param_value.val_long = value;
        
        return true;
    }

    bool ConfigParameter::setValue_double(double &value)
    {
        if(param_value.val_type != vt_double) return false;
        
        *param_value.val_double = value;
        
        return true;
    }

    bool ConfigParameter::setValue_string(std::string &value)
    {
        if(param_value.val_type != vt_string) return false;
        
        *param_value.val_string = value;
        
        return true;
    }

    bool ConfigParameter::setValue_1dvector_long(std::vector<long> &value)
    {
        if(param_value.val_type != vt_1dvector_long) return false;
        
        *param_value.val_1dvector_long = value;
        
        return true;
    }

    bool ConfigParameter::setValue_1dvector_double(std::vector<double> &value)
    {
        if(param_value.val_type != vt_1dvector_double) return false;
        
        *param_value.val_1dvector_double = value;
        
        return true;
    }



}

