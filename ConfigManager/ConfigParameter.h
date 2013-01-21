/*! 
    @file ConfigParameters.h
    @brief This is the header file for the ConfigParameters objects of the 
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

#ifndef ConfigParameter_def
#define ConfigParameter_def

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
        vt_1dvector_long, vt_1dvector_double
    };
    
    /*!
     * 
     */
    class ConfigParameter
    {
		public:
		    ConfigParameter();
		    ConfigParameter(std::string name, std::string path, value_type val_type);

		    //! Returns this parameter's name.
		    std::string getName();
		    std::string getName() const;
		    
		    //! Returns the path to this parameter in the ConfigTree 
		    //! (not including the final '.<name>').
		    std::string getPath();
		    std::string getPath() const;
		    
		    //! Returns a meaningful description of this parameter's purpose.
		    std::string getDescription();
		    std::string getDescription() const;
		    
		    //! Return an enum value representing the type of this
		    //! parameter's value.
		    value_type getType();
		    value_type getType() const;
		    
		    //! Sets the name of this parameter in the ConfigTree.
		    void setName(std::string new_name);
		    
		    //! Sets the path of this parameter in the ConfigTree
		    void setPath(std::string new_path);
		    
		    //! Sets the description of this ConfigParameter.
		    void setDescription(std::string new_desc);

		    //! Set an enum value representing the type of this parameter's value.
		    void setType(value_type new_val_type);


		    //! Returns long   range if val_type is a long.
		    bool getRange_long(ConfigRange<long> &range);
		    
		    //! Returns double range if val_type is a double.
		    bool getRange_double(ConfigRange<double> &range);

		    //! Sets long   range if val_type is a long.
		    bool setRange_long(ConfigRange<long> &range);
		    
		    //! Sets double range if val_type is a double.
		    bool setRange_double(ConfigRange<double> &range);
		    

		    //! If this parameter's value type is bool          , return the value.
		    bool getValue_bool(bool &value);
		    
		    //! If this parameter's value type is long          , return the value.
		    bool getValue_long(long &value);
		    
		    //! If this parameter's value type is double        , return the value.
		    bool getValue_double(double &value);
		    
		    //! If this parameter's value type is string        , return the value.
		    bool getValue_string(std::string &value);
		    
		    //! If this parameter's value type is vector<long  >, return the value.
		    bool getValue_vector_long(std::vector<long> &value);
		    
		    //! If this parameter's value type is vector<double>, return the value.
		    bool getValue_vector_double(std::vector<double> &value); 

		    //! If this parameter's value type is bool          , set the value.
		    bool setValue_bool(bool &value);
		    
		    
		    
		    //! If this parameter's value type is long          , set the value.
		    bool setValue_long(long &value);
		    
		    //! If this parameter's value type is double        , set the value.
		    bool setValue_double(double &value);
		    
		    //! If this parameter's value type is string        , set the value.
		    bool setValue_string (std::string &value);
		    
		    //! If this parameter's value type is vector<long  >, set the value.
		    bool setValue_vector_long(std::vector<long> &value);
		    
		    //! If this parameter's value type is vector<double>, set the value.
		    bool setValue_vector_double(std::vector<double> &value); 
		    
		    
		    
		private:
		    //! This parameter's name.
		    std::string _name;

		    //! This parameter's path in the config system 
		    //! (not including '.<name>').
		    std::string _path;

		    //! A description of this parameter.
		    std::string _desc;

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
		            
		            std::vector<long> *val_1dvector_long  ;
		            std::vector<double> *val_1dvector_double;
		        };
		        
		        union 
		        {
		            ConfigRange<long> *range_long  ;
		            ConfigRange<double> *range_double;
		        };
		    } param_value;
    };
}

#endif
