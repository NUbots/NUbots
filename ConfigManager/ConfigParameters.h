/*! 
    @file ConfigParameters.h
    @brief This is the header file for the ConfigParameters objects of the configuration system for the 
    NUbots.
    
    @class ConfigParameters
    @brief This class serves as an object for use when transferring parameters.
    
    @author Sophie Calland
    
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

#ifndef ConfigParameters_def
#define ConfigParameters_def

#include "ConfigRange.h"

#include <iostream>
#include <string>
#include <vector>
// #include <boost/static_assert.hpp>




namespace ConfigSystem
{
    /// Used to identify the type of a ConfigParameter's value.
    enum value_type 
    {
        vt_bool, vt_long, vt_double, vt_string,
        vt_vector_long, vt_vector_double
    };
    
    /*!
     * 
     */
    class ConfigParameter
    {
    private:
        std::string _name; //! This parameter's name.
        std::string _path; //! This parameter's path in the config system (not including '.<name>').
        std::string _desc; //! A description of this parameter.

        // Following are a variety of variables intended to represent this 
        // node's value.
        // Supported types should (minimally) be:
        //   - bool
        //   - long   (int, char, unsiged types?)
        //   - double (float)
        //   - string
        //   - vector<long>/long[]
        //   - vector<double>/double[]
        
        struct ParameterValue
        {
            /// The type of this parameter's value
            value_type val_type;
            union 
            {
                bool                *val_bool         ;
                long                *val_long         ;
                double              *val_double       ;
                std::string         *val_string       ;
                std::vector<long  > *val_vector_long  ;
                std::vector<double> *val_vector_double;
            };
        } _paramValue;
        
    public:
        ConfigParameter();
        ConfigParameter(
            std::string name,
            std::string path,
            value_type val_type
            );

        //! Returns this parameter's name.
        std::string getName       ();
        
        //! Returns the path to this parameter in the ConfigTree (not including the final '.<name>').
        std::string getPath       ();
        
        //! Returns a meaningful description of this parameter's purpose.
        std::string getDescription();
        
        //! Return an enum value representing the type of this parameter's value.
        value_type getType();
        
        //! Sets the name of this parameter in the ConfigTree.
        void   setName       (std::string name);
        
        //! Sets the path of this parameter in the ConfigTree
        void   setPath       (std::string path);
        
        //! Sets the description of this ConfigParameter.
        void   setDescription(std::string desc);

        //! Set an enum value representing the type of this parameter's value.
        void setType(value_type val_type);

        
        //! If this parameter's value type is bool          , return the value.
        bool getValue_bool         (bool                &value);
        //! If this parameter's value type is long          , return the value.
        bool getValue_long         (long                &value);
        //! If this parameter's value type is double        , return the value.
        bool getValue_double       (double              &value);
        //! If this parameter's value type is string        , return the value.
        bool getValue_string       (std::string         &value);
        //! If this parameter's value type is vector<long  >, return the value.
        bool getValue_vector_long  (std::vector<long  > &value);
        //! If this parameter's value type is vector<double>, return the value.
        bool getValue_vector_double(std::vector<double> &value); 

        //! If this parameter's value type is bool          , set the value.
        bool setValue_bool         (bool                &value);
        //! If this parameter's value type is long          , set the value.
        bool setValue_long         (long                &value);
        //! If this parameter's value type is double        , set the value.
        bool setValue_double       (double              &value);
        //! If this parameter's value type is string        , set the value.
        bool setValue_string       (std::string         &value);
        //! If this parameter's value type is vector<long  >, set the value.
        bool setValue_vector_long  (std::vector<long  > &value);
        //! If this parameter's value type is vector<double>, set the value.
        bool setValue_vector_double(std::vector<double> &value); 
    };
}

#endif
