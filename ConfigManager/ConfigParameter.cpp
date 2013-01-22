/*! 
    @file ConfigParameters.cpp
    @brief This is the implementation file for the ConfigParameters objects of the 
    configuration system for the NUbots.
    
    @class ConfigParameters
    @brief This class serves as an object for use when transferring parameters.
    
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
	// Constructors
    // ConfigParameter::ConfigParameter()
    // {
    //     param_value.val_type = vt_none;
    //     param_value.val_bool = NULL;
    // }


    ConfigParameter::ConfigParameter(value_type val_type)
    {
        // Set the value type to the type specified
        param_value.val_type = val_type;

        // Set the relevant pointer to NULL initially.
        // (in theory it doesn't matter which...)
        switch(val_type)
        {
        case vt_bool           : param_value.val_bool            = NULL; break;
        case vt_long           : param_value.val_long            = NULL; break;
        case vt_double         : param_value.val_double          = NULL; break;
        case vt_string         : param_value.val_string          = NULL; break;
        case vt_1dvector_long  : param_value.val_1dvector_long   = NULL; break;
        case vt_1dvector_double: param_value.val_1dvector_double = NULL; break;
        }
    }

    //Getting/setting string and general info stuff. 
	
    std::string ConfigParameter::getName() { return _name; }
    std::string ConfigParameter::getName() const { return _name; }
    
    std::string ConfigParameter::getPath() { return _path; }
    std::string ConfigParameter::getPath() const { return _path; }

    std::string ConfigParameter::getDescription() { return _desc; }
    std::string ConfigParameter::getDescription() const { return _desc; }
    
    value_type ConfigParameter::getType() { return param_value.val_type; }
    value_type ConfigParameter::getType() const { return param_value.val_type; }
    

    void ConfigParameter::setName(std::string new_name) 
    { 
    	_name = new_name; 
    }
    
    void ConfigParameter::setPath(std::string new_path) 
    { 
    	_path = new_path; 
    }
    
    void ConfigParameter::setDescription(std::string new_desc) 
    { 
    	_desc = new_desc; 
    }
    
    // void ConfigParameter::setType(value_type new_val_type) 
    // {
    //     param_value.val_type = new_val_type;
    // }



	//Getting/setting general value stuff
	
    bool ConfigParameter::getRange_long(ConfigRange<long> &range)
    {
        if(param_value.val_type != vt_long) return false;
        range = *(param_value.range_long);
        return true;
    }
    
    bool ConfigParameter::getRange_double(ConfigRange<double> &range)
    {
        if(param_value.val_type != vt_double) return false;
        range = *(param_value.range_double);
        return true;
    }

    bool ConfigParameter::setRange_long(ConfigRange<long> &range)
    {
        if(param_value.val_type != vt_long) return false;

        delete param_value.range_long;
        param_value.range_long = new ConfigRange<long>(range);
        
        return true;
    }

    bool ConfigParameter::setRange_double(ConfigRange<double> &range)
    {
        if(param_value.val_type != vt_double) return false;
        
        delete param_value.range_double;
        param_value.range_double = new ConfigRange<double>(range);
        
        return true;
    }



    bool ConfigParameter::getValue_bool(bool &value)
    {
        if(param_value.val_type != vt_bool) return false;
        value = *(param_value.val_bool);
        return true;
    }
    
    bool ConfigParameter::getValue_long(long &value)
    {
        if(param_value.val_type != vt_long) return false;
        value = *(param_value.val_long);
        return true;
    }

    bool ConfigParameter::getValue_double(double &value)
    {
        if(param_value.val_type != vt_double) return false;
        value = *(param_value.val_double);
        return true;
    }

    bool ConfigParameter::getValue_string(std::string &value)
    {
        if(param_value.val_type != vt_string) return false;
        value = *(param_value.val_string);
        return true;
    }

    bool ConfigParameter::getValue_vector_long(std::vector<long> &value)
    {
        if(param_value.val_type != vt_1dvector_long) return false;
        value = *(param_value.val_1dvector_long);
        return true;
    }

    bool ConfigParameter::getValue_vector_double(std::vector<double> &value)
    {
        if(param_value.val_type != vt_1dvector_double) return false;
        value = *(param_value.val_1dvector_double);
        return true;
    }

    bool ConfigParameter::setValue_bool(bool &value)
    {
        if( param_value.val_type != vt_bool) return false;
        // if(!param_value.range_bool->test(value)) return false;
        
        delete param_value.val_bool;
        param_value.val_bool = new bool(value);
        
        return true;
    }

    bool ConfigParameter::setValue_long(long &value)
    {
        if( param_value.val_type != vt_long) return false;
        if(!param_value.range_long->test(value)) return false;
        
        delete param_value.val_long;
        param_value.val_long = new long(value);
        
        return true;
    }

    bool ConfigParameter::setValue_double(double &value)
    {
        if( param_value.val_type != vt_double) return false;
        if(!param_value.range_double->test(value)) return false;
        
        delete param_value.val_double;
        param_value.val_double = new double(value);
        
        return true;
    }

    bool ConfigParameter::setValue_string(std::string &value)
    {
        if( param_value.val_type != vt_string) return false;
        // if(!param_value.range_string->test(value)) return false;
        
        delete param_value.val_string;
        param_value.val_string = new std::string(value);
        
        return true;
    }

    bool ConfigParameter::setValue_vector_long(std::vector<long> &value)
    {
        if( param_value.val_type != vt_1dvector_long) return false;
        if(!param_value.range_long->test(value)) return false;
        
        delete param_value.val_1dvector_long;
        param_value.val_1dvector_long = new std::vector<long> (value);
        
        return true;
    }

    bool ConfigParameter::setValue_vector_double(std::vector<double> &value)
    {
        if( param_value.val_type != vt_1dvector_double) return false;
        if(!param_value.range_double->test(value)) return false;
        
        delete param_value.val_1dvector_double;
        param_value.val_1dvector_double = new std::vector<double> (value);
        
        return true;
    }



}

