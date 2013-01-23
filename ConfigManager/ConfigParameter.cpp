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
	const char* makeValueTypeString(value_type vt)
    {
        switch(vt)
        {
        // case vt_none           : return "none"         ;
        case vt_bool           : return "bool"         ;
        case vt_long           : return "long"         ;
        case vt_double         : return "double"       ;
        case vt_string         : return "string"       ;
        case vt_1dvector_long  : return "vector_long"  ;
        case vt_1dvector_double: return "vector_double";
        default                : return "none"         ;
        }
    }

    value_type stringToValueType  (std::string typStr)
    {
             if(typStr.compare("bool"         ) == 0) return vt_bool           ;
        else if(typStr.compare("long"         ) == 0) return vt_long           ;
        else if(typStr.compare("double"       ) == 0) return vt_double         ;
        else if(typStr.compare("string"       ) == 0) return vt_string         ;
        else if(typStr.compare("vector_double") == 0) return vt_1dvector_long  ;
        else if(typStr.compare("vector_double") == 0) return vt_1dvector_double;
        // else if(typStr.compare("none"         ) == 0) return vt_none           ;
        else return vt_none;
    }

    const char* makeBoundTypeString(BoundType vt)
    {
        switch(vt)
        {
        case bt_none   : return "none"     ;
        case bt_open   : return "open"     ;
        case bt_closed : return "closed"   ;
        default        : return "unknown"  ;
        }
    }

    BoundType   stringToBoundType    (std::string typStr)
    {
             if(typStr.compare("none"   ) == 0) return bt_none  ;
        else if(typStr.compare("open"   ) == 0) return bt_open  ;
        else if(typStr.compare("closed" ) == 0) return bt_closed;
        else return bt_unknown;
    }


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
        
        // Set default values
        _name     = "_name";
        _path     = "_path";
        _desc     = ""   ;
        _modified = false;
        _locked   = false;
        
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
        default: 
            param_value.val_long   = NULL;
            break;
        }
        switch(val_type)
        {
        case vt_double         :
        case vt_1dvector_double:
            param_value.range_double = new ConfigRange<double>();
            break;
        case vt_bool         :
        case vt_long         :
        case vt_1dvector_long:
            param_value.range_long = new ConfigRange<long>();
            break;
        default: 
            param_value.range_long   = NULL;
            break;
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


    bool ConfigParameter::isModified()
    {
        return _modified;
    }
    void ConfigParameter::resetModified()
    {
        _modified = false;    
    }
    void ConfigParameter::setModified(bool modVal)
    {
        _modified = modVal;    
    }
    bool ConfigParameter::isLocked()
    {
        return _locked;
    }
    void ConfigParameter::setLocked(bool lockVal)
    {
        _locked = lockVal;
    }


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
        if(_locked) return false;
        if(param_value.val_type != vt_long) return false;

        delete param_value.range_long;
        param_value.range_long = new ConfigRange<long>(range);
        
        return true;
    }

    bool ConfigParameter::setRange_double(ConfigRange<double> &range)
    {
        if(_locked) return false;
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
        if(_locked) return false;
        if( param_value.val_type != vt_bool) return false;
        // if(!param_value.range_bool->apply(value)) return false;
        
        delete param_value.val_bool;
        param_value.val_bool = new bool(value);
        
        return true;
    }

    bool ConfigParameter::setValue_long(long &value)
    {
        if(_locked) return false;
        if( param_value.val_type != vt_long) return false;
        if(!param_value.range_long->apply(value)) return false;
        
        delete param_value.val_long;
        param_value.val_long = new long(value);
        
        return true;
    }

    bool ConfigParameter::setValue_double(double &value)
    {
        if(_locked) return false;
        if( param_value.val_type != vt_double) return false;
        if(!param_value.range_double->apply(value)) return false;

        delete param_value.val_double;
        param_value.val_double = new double(value);
        
        return true;
    }

    bool ConfigParameter::setValue_string(std::string &value)
    {
        if(_locked) return false;
        if( param_value.val_type != vt_string) return false;
        // if(!param_value.range_string->apply(value)) return false;
        
        delete param_value.val_string;
        param_value.val_string = new std::string(value);
        
        return true;
    }

    bool ConfigParameter::setValue_vector_long(std::vector<long> &value)
    {
        if(_locked) return false;
        if( param_value.val_type != vt_1dvector_long) return false;
        if(!param_value.range_long->apply(value)) return false;
        
        delete param_value.val_1dvector_long;
        param_value.val_1dvector_long = new std::vector<long> (value);
        
        return true;
    }

    bool ConfigParameter::setValue_vector_double(std::vector<double> &value)
    {
        if(_locked) return false;
        if( param_value.val_type != vt_1dvector_double) return false;
        if(!param_value.range_double->apply(value)) return false;
        
        delete param_value.val_1dvector_double;
        param_value.val_1dvector_double = new std::vector<double> (value);
        
        return true;
    }

    // Overloaded getters and setters
    bool ConfigParameter::setValue(bool                &value)
    { 
        setValue_bool         (value);
    }
    bool ConfigParameter::setValue(long                &value)
    { 
        setValue_long         (value);
    }
    bool ConfigParameter::setValue(double              &value)
    { 
        setValue_double       (value);
    }
    bool ConfigParameter::setValue(std::string         &value)
    { 
        setValue_string       (value);
    }
    bool ConfigParameter::setValue(std::vector<long>   &value)
    { 
        setValue_vector_long  (value);
    }
    bool ConfigParameter::setValue(std::vector<double> &value)
    { 
        setValue_vector_double(value);
    }

    bool ConfigParameter::getValue(bool                &value)
    { 
        return getValue_bool         (value);
    }
    bool ConfigParameter::getValue(long                &value)
    { 
        return getValue_long         (value);
    }
    bool ConfigParameter::getValue(double              &value)
    { 
        return getValue_double       (value);
    }
    bool ConfigParameter::getValue(std::string         &value)
    { 
        return getValue_string       (value);
    }
    bool ConfigParameter::getValue(std::vector<long>   &value)
    { 
        return getValue_vector_long  (value);
    }
    bool ConfigParameter::getValue(std::vector<double> &value)
    {
        return getValue_vector_double(value);
    }

    bool ConfigParameter::getRange(ConfigRange<long>   &range)
    {
        return getRange_long  (range);
    }
    bool ConfigParameter::getRange(ConfigRange<double> &range)
    {
        return getRange_double(range);
    }

    bool ConfigParameter::setRange(ConfigRange<long>   &range)
    {
        setRange_long  (range);
    }
    bool ConfigParameter::setRange(ConfigRange<double> &range)
    {
        setRange_double(range);
    }
}

