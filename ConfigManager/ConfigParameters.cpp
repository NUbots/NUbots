#include "ConfigParameters.h"

namespace ConfigSystem
{
    std::string ConfigParameter::getName       () {	return _name; }
    std::string ConfigParameter::getPath       () {	return _path; }
    std::string ConfigParameter::getDescription() {	return _desc; }
    value_type ConfigParameter::getType() { return _paramValue.val_type; }
    
    void ConfigParameter::setName       (std::string name) { _name = name; }
    void ConfigParameter::setPath       (std::string path) { _path = path; }
    void ConfigParameter::setDescription(std::string desc) { _desc = desc; }
    void ConfigParameter::setType(value_type val_type) 
    {
        _paramValue.val_type = val_type;
    }



    bool ConfigParameter::getValue_bool         (bool                &value)
    {
        if(_paramValue.val_type != vt_bool         ) return false;
        value = *_paramValue.val_bool         ;
        return true;
    }

    bool ConfigParameter::getValue_long         (long                &value)
    {
        if(_paramValue.val_type != vt_long         ) return false;
        value = *_paramValue.val_long         ;
        return true;
    }

    bool ConfigParameter::getValue_double       (double              &value)
    {
        if(_paramValue.val_type != vt_double       ) return false;
        value = *_paramValue.val_double       ;
        return true;
    }

    bool ConfigParameter::getValue_string       (std::string         &value)
    {
        if(_paramValue.val_type != vt_string       ) return false;
        value = *_paramValue.val_string       ;
        return true;
    }

    bool ConfigParameter::getValue_vector_long  (std::vector<long  > &value)
    {
        if(_paramValue.val_type != vt_vector_long  ) return false;
        value = *_paramValue.val_vector_long  ;
        return true;
    }

    bool ConfigParameter::getValue_vector_double(std::vector<double> &value)
    {
        if(_paramValue.val_type != vt_vector_double) return false;
        value = *_paramValue.val_vector_double;
        return true;
    }

    bool ConfigParameter::setValue_bool         (bool                &value)
    {
        if( _paramValue.val_type != vt_bool         ) return false;
        // if(!_paramValue.range_bool  ->test(value)) return false;
        *_paramValue.val_bool          = value;
        return true;
    }

    bool ConfigParameter::setValue_long         (long                &value)
    {
        if( _paramValue.val_type != vt_long         ) return false;
        if(!_paramValue.range_long  ->test(value)) return false;
        *_paramValue.val_long          = value;
        return true;
    }

    bool ConfigParameter::setValue_double       (double              &value)
    {
        if( _paramValue.val_type != vt_double       ) return false;
        if(!_paramValue.range_double->test(value)) return false;
        *_paramValue.val_double        = value;
        return true;
    }

    bool ConfigParameter::setValue_string       (std::string         &value)
    {
        if( _paramValue.val_type != vt_string       ) return false;
        // if(!_paramValue.range_string->test(value)) return false;
        *_paramValue.val_string        = value;
        return true;
    }
    
    bool ConfigParameter::setValue_vector_long  (std::vector<long  > &value)
    {
        if( _paramValue.val_type != vt_vector_long  ) return false;
        if(!_paramValue.range_long  ->test(value)) return false;
        *_paramValue.val_vector_long   = value;
        return true;
    }

    bool ConfigParameter::setValue_vector_double(std::vector<double> &value)
    {
        if( _paramValue.val_type != vt_vector_double) return false;
        if(!_paramValue.range_double->test(value)) return false;
        *_paramValue.val_vector_double = value;
        return true;
    }



}

