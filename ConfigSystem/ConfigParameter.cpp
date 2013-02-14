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
#include <iostream>

namespace ConfigSystem
{
	const char* makeValueTypeString(value_type vt)
    {
        switch(vt)
        {
        case vt_bool           : return "bool"            ;
        case vt_long           : return "long"            ;
        case vt_double         : return "double"          ;
        case vt_string         : return "string"          ;
        case vt_1dvector_long  : return "1d_vector_long"  ;
        case vt_2dvector_long  : return "2d_vector_long"  ;
        case vt_3dvector_long  : return "3d_vector_long"  ;
        case vt_1dvector_double: return "1d_vector_double";
        case vt_2dvector_double: return "2d_vector_double";
        case vt_3dvector_double: return "3d_vector_double";
        default                : return "none"            ;
        }
    }

    value_type stringToValueType  (std::string typStr)
    {
             if(typStr.compare("bool"            ) == 0) return vt_bool           ;
        else if(typStr.compare("long"            ) == 0) return vt_long           ;
        else if(typStr.compare("double"          ) == 0) return vt_double         ;
        else if(typStr.compare("string"          ) == 0) return vt_string         ;
        else if(typStr.compare("1d_vector_long"  ) == 0) return vt_1dvector_long  ;
        else if(typStr.compare("2d_vector_long"  ) == 0) return vt_2dvector_long  ;
        else if(typStr.compare("3d_vector_long"  ) == 0) return vt_3dvector_long  ;
        else if(typStr.compare("1d_vector_double") == 0) return vt_1dvector_double;
        else if(typStr.compare("2d_vector_double") == 0) return vt_2dvector_double;
        else if(typStr.compare("3d_vector_double") == 0) return vt_3dvector_double;
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


    
    // Note: These constructors are ugly and ridiculous, but make other methods
    //       far neater.  They should be refactored ASAP!
    //       I don't think you can template constructors in a non-template class,
    //       but a *single* templated method could be called from each
    //       constructor that initialises all the member variables?
    ConfigParameter::ConfigParameter(bool                                            value)
    {
        _val_type = vt_bool;
        _desc = "";
        _modified = false;
        _locked   = false;
        _val_bool = new bool(value);
        _range_long = new ConfigRange<long>();
    }
    ConfigParameter::ConfigParameter(long                                            value)
    {
        _val_type = vt_long;
        _desc = "";
        _modified = false;
        _locked   = false;
        _val_long = new long(value);
        _range_long = new ConfigRange<long>();
    }
    ConfigParameter::ConfigParameter(double                                          value)
    {
        _val_type = vt_double;
        _desc = "";
        _modified = false;
        _locked   = false;
        _val_double = new double(value);
        _range_double = new ConfigRange<double>();
    }
    ConfigParameter::ConfigParameter(std::string                                     value)
    {
        _val_type = vt_string;
        _desc = "";
        _modified = false;
        _locked   = false;
        _val_string = new std::string(value);
        _range_long = NULL;
    }
    ConfigParameter::ConfigParameter(std::vector<long>                               value)
    {
        _val_type = vt_1dvector_long;
        _desc = "";
        _modified = false;
        _locked   = false;
        _val_1dvector_long = new std::vector<long>(value);
        _range_long = new ConfigRange<long>();
    }
    ConfigParameter::ConfigParameter(std::vector<std::vector<long> >                 value)
    {
        _val_type = vt_2dvector_long;
        _desc = "";
        _modified = false;
        _locked   = false;
        _val_2dvector_long = new std::vector<std::vector<long> >(value);
        _range_long = new ConfigRange<long>();
    }
    ConfigParameter::ConfigParameter(std::vector<std::vector<std::vector<long> > >   value)
    {
        _val_type = vt_3dvector_long;
        _desc = "";
        _modified = false;
        _locked   = false;
        _val_3dvector_long = new std::vector<std::vector<std::vector<long> > >(value);
        _range_long = new ConfigRange<long>();
    }
    ConfigParameter::ConfigParameter(std::vector<double>                             value)
    {
        _val_type = vt_1dvector_double;
        _desc = "";
        _modified = false;
        _locked   = false;
        _val_1dvector_double = new std::vector<double>(value);
        _range_double = new ConfigRange<double>();
    }
    ConfigParameter::ConfigParameter(std::vector<std::vector<double> >               value)
    {
        _val_type = vt_2dvector_double;
        _desc = "";
        _modified = false;
        _locked   = false;
        _val_2dvector_double = new std::vector<std::vector<double> >(value);
        _range_double = new ConfigRange<double>();
    }
    ConfigParameter::ConfigParameter(std::vector<std::vector<std::vector<double> > > value)
    {
        _val_type = vt_3dvector_double;
        _desc = "";
        _modified = false;
        _locked   = false;
        _val_3dvector_double = new std::vector<std::vector<std::vector<double> > >(value);
        _range_double = new ConfigRange<double>();
    }

    ConfigParameter::ConfigParameter(value_type val_type)
    {
        // Set the value type to the type specified
        _val_type = val_type;
        
        // Set default values
        // _name     = "_name";
        // _path     = "_path";
        _desc     = ""   ;
        _modified = false;
        _locked   = false;
        
        // Set the relevant pointer to NULL initially.
        // (in theory it doesn't matter which...)
        switch(val_type)
        {
        case vt_none           : _val_long            = NULL; break;
        
        case vt_bool           : _val_bool            = NULL; break;
        case vt_long           : _val_long            = NULL; break;
        case vt_double         : _val_double          = NULL; break;
        case vt_string         : _val_string          = NULL; break;
        case vt_1dvector_long  : _val_1dvector_long   = NULL; break;
        case vt_2dvector_long  : _val_2dvector_long   = NULL; break;
        case vt_3dvector_long  : _val_3dvector_long   = NULL; break;
        case vt_1dvector_double: _val_1dvector_double = NULL; break;
        case vt_2dvector_double: _val_2dvector_double = NULL; break;
        case vt_3dvector_double: _val_3dvector_double = NULL; break;
        
        default: 
            std::cerr << __PRETTY_FUNCTION__ 
                      << ": Invalid val_type '" 
                      << makeValueTypeString(val_type) 
                      << "'." << std::endl;
            _val_long   = NULL;
            break;
        }

        switch(val_type)
        {
        case vt_none           :
        case vt_string         : // Strings don't have ranges
            _range_long   = NULL; 
            break;
            
        case vt_double         :
        case vt_1dvector_double:
        case vt_2dvector_double:
        case vt_3dvector_double:
            _range_double = new ConfigRange<double>();
            break;
        case vt_bool         :
        case vt_long         :
        case vt_1dvector_long:
        case vt_2dvector_long:
        case vt_3dvector_long:
            _range_long = new ConfigRange<long>();
            break;
            
        default: 
            std::cerr << __PRETTY_FUNCTION__ 
                      << ": Invalid val_type '" 
                      << makeValueTypeString(val_type) 
                      << "'." << std::endl;
            _range_long   = NULL;
            break;
        }
    }

    //Getting/setting string and general info stuff. 
	
    // std::string ConfigParameter::getName() { return _name; }
    // std::string ConfigParameter::getName() const { return _name; }
    
    // std::string ConfigParameter::getPath() { return _path; }
    // std::string ConfigParameter::getPath() const { return _path; }

    std::string ConfigParameter::getDescription() { return _desc; }
    std::string ConfigParameter::getDescription() const { return _desc; }
    
    value_type ConfigParameter::getType() { return _val_type; }
    value_type ConfigParameter::getType() const { return _val_type; }
    
    
    // void ConfigParameter::setName(std::string new_name) 
    // { 
    // 	_name = new_name; 
    // }
    
    // void ConfigParameter::setPath(std::string new_path) 
    // { 
    // 	_path = new_path; 
    // }
    
    void ConfigParameter::setDescription(std::string new_desc) 
    { 
    	_desc = new_desc; 
    }
    
    // void ConfigParameter::setType(value_type new_val_type) 
    // {
    //     _val_type = new_val_type;
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
    	value_type vt = _val_type;
    	
        if( vt != vt_long           && 
            vt != vt_1dvector_long  &&
            vt != vt_2dvector_long  &&
            vt != vt_3dvector_long
            ) return false;

        if(_range_long == NULL)
        {
            std::cout << __PRETTY_FUNCTION__ << ": " << "_range_long is NULL." << std::endl;
            return false;
        }

        range = *(_range_long);
        return true;
    }
    
    bool ConfigParameter::getRange_double(ConfigRange<double> &range)
    {
    	value_type vt = _val_type;
    
        if( vt != vt_double           && 
            vt != vt_1dvector_double  &&
            vt != vt_2dvector_double  &&
            vt != vt_3dvector_double
            ) return false;

        if(_range_double == NULL)
        {
            std::cout << __PRETTY_FUNCTION__ << ": " << "_range_double is NULL." << std::endl;
            return false;
        }

        range = *(_range_double);
        return true;
    }

    bool ConfigParameter::setRange_long(ConfigRange<long> &range)
    {
    	value_type vt = _val_type;
    
        if(_locked) return false;
        if( vt != vt_long           && 
            vt != vt_1dvector_long  &&
            vt != vt_2dvector_long  &&
            vt != vt_3dvector_long
            ) return false;

        delete _range_long;
        _range_long = new ConfigRange<long>(range);
        
        return true;
    }

    bool ConfigParameter::setRange_double(ConfigRange<double> &range)
    {
    	value_type vt = _val_type;
    	
        if(_locked) return false;
        if( vt != vt_double           && 
            vt != vt_1dvector_double  &&
            vt != vt_2dvector_double  &&
            vt != vt_3dvector_double
            ) return false;

        delete _range_double;
        _range_double = new ConfigRange<double>(range);
        
        return true;
    }



    bool ConfigParameter::getValue_bool(bool &value)
    {
        if(_val_type != vt_bool) return false;
        if(_val_bool == NULL)
        {
            std::cout << __PRETTY_FUNCTION__ << ": " << "_val_bool is NULL." << std::endl;
            return false;
        }
        value = *(_val_bool);
        return true;
    }
    
    bool ConfigParameter::getValue_long(long &value)
    {
        if(_val_type != vt_long) return false;
        if(_val_long == NULL)
        {
            std::cout << __PRETTY_FUNCTION__ << ": " << "_val_long is NULL." << std::endl;
            return false;
        }
        value = *(_val_long);
        return true;
    }

    bool ConfigParameter::getValue_double(double &value)
    {
        if(_val_type != vt_double) return false;
        if(_val_double == NULL)
        {
            std::cout << __PRETTY_FUNCTION__ << ": " << "_val_double is NULL." << std::endl;
            return false;
        }
        value = *(_val_double);
        return true;
    }

    bool ConfigParameter::getValue_string(std::string &value)
    {
        if(_val_type != vt_string) return false;
        if(_val_string == NULL)
        {
            std::cout << __PRETTY_FUNCTION__ << ": " << "_val_string is NULL." << std::endl;
            return false;
        }
        value = *(_val_string);
        return true;
    }

    bool ConfigParameter::getValue_vector1d_long(std::vector<long> &value)
    {
        if(_val_type != vt_1dvector_long) return false;
        if(_val_1dvector_long == NULL)
        {
            std::cout << __PRETTY_FUNCTION__ << ": " << "_val_1dvector_long is NULL." << std::endl;
            return false;
        }
        value = *(_val_1dvector_long);
        return true;
    }
    bool ConfigParameter::getValue_vector2d_long(std::vector<std::vector<long> > &value)
    {
        if(_val_type != vt_2dvector_long) return false;
        if(_val_2dvector_long == NULL)
        {
            std::cout << __PRETTY_FUNCTION__ << ": " << "_val_2dvector_long is NULL." << std::endl;
            return false;
        }
        value = *(_val_2dvector_long);
        return true;
    }
    bool ConfigParameter::getValue_vector3d_long(std::vector<std::vector<std::vector<long> > > &value)
    {
        if(_val_type != vt_3dvector_long) return false;
        if(_val_3dvector_long == NULL)
        {
            std::cout << __PRETTY_FUNCTION__ << ": " << "_val_3dvector_long is NULL." << std::endl;
            return false;
        }
        value = *(_val_3dvector_long);
        return true;
    }

    bool ConfigParameter::getValue_vector1d_double(std::vector<double> &value)
    {
        if(_val_type != vt_1dvector_double) return false;
        if(_val_1dvector_double == NULL)
        {
            std::cout << __PRETTY_FUNCTION__ << ": " << "_val_1dvector_double is NULL." << std::endl;
            return false;
        }
        value = *(_val_1dvector_double);
        return true;
    }
    bool ConfigParameter::getValue_vector2d_double(std::vector<std::vector<double> > &value)
    {
        if(_val_type != vt_2dvector_double) return false;
        if(_val_2dvector_double == NULL)
        {
            std::cout << __PRETTY_FUNCTION__ << ": " << "_val_2dvector_double is NULL." << std::endl;
            return false;
        }
        value = *(_val_2dvector_double);
        return true;
    }
    bool ConfigParameter::getValue_vector3d_double(std::vector<std::vector<std::vector<double> > > &value)
    {
        if(_val_type != vt_3dvector_double) return false;
        if(_val_3dvector_double == NULL)
        {
            std::cout << __PRETTY_FUNCTION__ << ": " << "_val_3dvector_double is NULL." << std::endl;
            return false;
        }
        value = *(_val_3dvector_double);
        return true;
    }

    bool ConfigParameter::setValue_bool(bool &value)
    {
        if(_locked) return false;
        if( _val_type != vt_bool) return false;
        // if(!_range_bool->apply(value)) return false;
        
        delete _val_bool;
        _val_bool = new bool(value);
        setModified(true);
        return true;
    }

    bool ConfigParameter::setValue_long(long &value)
    {
        if(_locked) return false;
        if( _val_type != vt_long) return false;
        if(!_range_long->apply(value)) return false;
        
        delete _val_long;
        _val_long = new long(value);
        setModified(true);
        return true;
    }

    bool ConfigParameter::setValue_double(double &value)
    {
        if(_locked) return false;
        if( _val_type != vt_double) return false;
        if(!_range_double->apply(value)) return false;

        delete _val_double;
        _val_double = new double(value);
        setModified(true);
        return true;
    }

    bool ConfigParameter::setValue_string(std::string &value)
    {
        if(_locked) return false;
        if( _val_type != vt_string) return false;
        // if(!_range_string->apply(value)) return false;
        
        delete _val_string;
        _val_string = new std::string(value);
        setModified(true);
        return true;
    }

    bool ConfigParameter::setValue_vector1d_long(std::vector<long> &value)
    {
        if(_locked) return false;
        if( _val_type != vt_1dvector_long) return false;
        if(!_range_long->apply(value)) return false;
        
        delete _val_1dvector_long;
        _val_1dvector_long = new std::vector<long> (value);
        setModified(true);
        return true;
    }
    bool ConfigParameter::setValue_vector2d_long(std::vector<std::vector<long> > &value)
    {
        if(_locked) return false;
        if( _val_type != vt_2dvector_long) return false;
        if(!_range_long->apply(value)) return false;
        
        delete _val_2dvector_long;
        _val_2dvector_long = new std::vector<std::vector<long> > (value);
        setModified(true);
        return true;
    }
    bool ConfigParameter::setValue_vector3d_long(std::vector<std::vector<std::vector<long> > > &value)
    {
        if(_locked) return false;
        if( _val_type != vt_3dvector_long) return false;
        if(!_range_long->apply(value)) return false;
        
        delete _val_3dvector_long;
        _val_3dvector_long = new std::vector<std::vector<std::vector<long> > > (value);
        setModified(true);
        return true;
    }

    bool ConfigParameter::setValue_vector1d_double(std::vector<double> &value)
    {
        if(_locked) return false;
        if( _val_type != vt_1dvector_double) return false;
        if(!_range_double->apply(value)) return false;
        
        delete _val_1dvector_double;
        _val_1dvector_double = new std::vector<double> (value);
        setModified(true);
        return true;
    }
    bool ConfigParameter::setValue_vector2d_double(std::vector<std::vector<double> > &value)
    {
        if(_locked) return false;
        if( _val_type != vt_2dvector_double) return false;
        if(!_range_double->apply(value)) return false;
        
        delete _val_2dvector_double;
        _val_2dvector_double = new std::vector<std::vector<double> > (value);
        setModified(true);
        return true;
    }
    bool ConfigParameter::setValue_vector3d_double(std::vector<std::vector<std::vector<double> > > &value)
    {
        if(_locked) return false;
        if( _val_type != vt_3dvector_double) return false;
        if(!_range_double->apply(value)) return false;
        
        delete _val_3dvector_double;
        _val_3dvector_double = new std::vector<std::vector<std::vector<double> > > (value);
        setModified(true);
        return true;
    }

    // Overloaded getters and setters
    bool ConfigParameter::setValue(bool                &value)
    { 
        return setValue_bool         (value);
    }
    bool ConfigParameter::setValue(long                &value)
    { 
        return setValue_long         (value);
    }
    bool ConfigParameter::setValue(double              &value)
    { 
        return setValue_double       (value);
    }
    bool ConfigParameter::setValue(std::string         &value)
    { 
        return setValue_string       (value);
    }
    bool ConfigParameter::setValue(std::vector<long>   &value)
    { 
        return setValue_vector1d_long  (value);
    }
    bool ConfigParameter::setValue(std::vector<std::vector<long> >   &value)
    { 
        return setValue_vector2d_long  (value);
    }
    bool ConfigParameter::setValue(std::vector<std::vector<std::vector<long> > >   &value)
    { 
        return setValue_vector3d_long  (value);
    }
    bool ConfigParameter::setValue(std::vector<double> &value)
    { 
        return setValue_vector1d_double(value);
    }
    bool ConfigParameter::setValue(std::vector<std::vector<double> > &value)
    { 
        return setValue_vector2d_double(value);
    }
    bool ConfigParameter::setValue(std::vector<std::vector<std::vector<double> > > &value)
    { 
        return setValue_vector3d_double(value);
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
        return getValue_vector1d_long  (value);
    }
    bool ConfigParameter::getValue(std::vector<std::vector<long> >   &value)
    { 
        return getValue_vector2d_long  (value);
    }
    bool ConfigParameter::getValue(std::vector<std::vector<std::vector<long> > >   &value)
    { 
        return getValue_vector3d_long  (value);
    }
    bool ConfigParameter::getValue(std::vector<double> &value)
    {
        return getValue_vector1d_double(value);
    }
    bool ConfigParameter::getValue(std::vector<std::vector<double> > &value)
    {
        return getValue_vector2d_double(value);
    }
    bool ConfigParameter::getValue(std::vector<std::vector<std::vector<double> > > &value)
    {
        return getValue_vector3d_double(value);
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
        return setRange_long  (range);
    }
    bool ConfigParameter::setRange(ConfigRange<double> &range)
    {
        return setRange_double(range);
    }
}

