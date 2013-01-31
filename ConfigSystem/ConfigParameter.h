/*! 
    @file ConfigParameters.h
    @brief This is the header file for the ConfigParameters objects of the 
    configuration system for the NUbots.
    
    @class ConfigParameters
    @brief This class represents a generic configuration parameter object.
    
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

#define CONFIGSYS_DEBUG_CALLS /*std::cout << "DEBUG> " << __PRETTY_FUNCTION__ << std::endl; */

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
        vt_none,
        vt_bool, 
        vt_long, 
        vt_double, 
        vt_string,
        vt_1dvector_long, 
        vt_2dvector_long, 
        vt_3dvector_long, 
        vt_1dvector_double,
        vt_2dvector_double,
        vt_3dvector_double
    };
    
    //Should these be in a class or better here?
    const char* makeValueTypeString   (value_type vt);
    value_type stringToValueType  (std::string typStr);

    const char* makeBoundTypeString  (BoundType vt)      ;
    BoundType   stringToBoundType    (std::string typStr);

    /*!
     * 
     */
    class ConfigParameter
    {
        public:

        // Note: If constructor doesn't specify type,
        //       type must be set to vt_none.
        //   ConfigParameter();

            ConfigParameter(value_type val_type);

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

            //! Indicates whether this ConfigParameter has been modified since
            //! this flag was last reset...
            //! (automatically updated by 'set' methods...?)
            bool isModified();
            //! Resets the 'modified' flag to false.
            void resetModified();
            void setModified(bool modVal);

            //! Returns whether or not the value of this parameter has been
            //! locked.
            bool isLocked();
            //! Sets whether or not the value of this parameter has been
            //! locked.
            void setLocked(bool lockVal);


            // NOTE: Changing the type of a ConfigParameter is dangerous.
            // //! Set an enum value representing the type of this parameter's value.
            // void setType(value_type new_val_type);


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
            bool getValue_vector1d_long(std::vector<long> &value);
            //! If this parameter's value type is vector<std::vector<long>  >, return the value.
            bool getValue_vector2d_long(std::vector<std::vector<long> > &value);
            //! If this parameter's value type is vector<std::vector<std::vector<long> >  >, return the value.
            bool getValue_vector3d_long(std::vector<std::vector<std::vector<long> > > &value);
            //! If this parameter's value type is vector<double>, return the value.
            bool getValue_vector1d_double(std::vector<double> &value); 
            //! If this parameter's value type is vector<std::vector<double> >, return the value.
            bool getValue_vector2d_double(std::vector<std::vector<double> > &value); 
            //! If this parameter's value type is vector<std::vector<std::vector<double> > >, return the value.
            bool getValue_vector3d_double(std::vector<std::vector<std::vector<double> > > &value); 

            //! If this parameter's value type is bool          , set the value.
            bool setValue_bool(bool &value);
            //! If this parameter's value type is long          , set the value.
            bool setValue_long(long &value);
            //! If this parameter's value type is double        , set the value.
            bool setValue_double(double &value);
            //! If this parameter's value type is string        , set the value.
            bool setValue_string (std::string &value);
            //! If this parameter's value type is vector<long>, set the value.
            bool setValue_vector1d_long(std::vector<long> &value); 
            //! If this parameter's value type is vector<std::vector<long> >, set the value.
            bool setValue_vector2d_long(std::vector<std::vector<long> > &value); 
            //! If this parameter's value type is vector<std::vector<std::vector<long> > >, set the value.
            bool setValue_vector3d_long(std::vector<std::vector<std::vector<long> > > &value); 
            //! If this parameter's value type is vector<double>, set the value.
            bool setValue_vector1d_double(std::vector<double> &value); 
            //! If this parameter's value type is vector<std::vector<double> >, set the value.
            bool setValue_vector2d_double(std::vector<std::vector<double> > &value); 
            //! If this parameter's value type is vector<std::vector<std::vector<double> > >, set the value.
            bool setValue_vector3d_double(std::vector<std::vector<std::vector<double> > > &value); 
            

            // Overloaded getters and setters:
            // These simply call the appropriate getter/setter from above.
            // (these could instead be implemented using 
            //  templates + specializations)
            bool setValue(bool                &value);
            bool setValue(long                &value);
            bool setValue(double              &value);
            bool setValue(std::string         &value);
            bool setValue(std::vector<long>   &value);
            bool setValue(std::vector<std::vector<long> >   &value);
            bool setValue(std::vector<std::vector<std::vector<long> > >   &value);
            bool setValue(std::vector<double> &value);
            bool setValue(std::vector<std::vector<double> > &value);
            bool setValue(std::vector<std::vector<std::vector<double> > > &value);

            bool getValue(bool                &value);
            bool getValue(long                &value);
            bool getValue(double              &value);
            bool getValue(std::string         &value);
            bool getValue(std::vector<long>   &value);
            bool getValue(std::vector<std::vector<long> >   &value);
            bool getValue(std::vector<std::vector<std::vector<long> > >   &value);
            bool getValue(std::vector<double> &value);
            bool getValue(std::vector<std::vector<double> > &value);
            bool getValue(std::vector<std::vector<std::vector<double> > > &value);

            bool getRange(ConfigRange<long>   &range);
            bool getRange(ConfigRange<double> &range);

            bool setRange(ConfigRange<long>   &range);
            bool setRange(ConfigRange<double> &range);
            
            
        private:
            //! This parameter's name.
            std::string _name;

            //! This parameter's path in the config system 
            //! (not including '.<name>').
            std::string _path;

            //! A description of this parameter.
            std::string _desc;

            //! Has this parameter been modified (since this flag was last reset).
            bool _modified    ;
            //! Is this parameter 'locked'? (i.e. have changes been disallowed)
            //! The mutator methods of a locked ConfigParameter will all fail
            //! (i.e. return false).
            bool _locked      ;

            // Following are a variety of variables intended to represent this 
            // node's value.
            // Supported types should (minimally) be:
            //   - bool
            //   - long   (int, char, unsiged types?)
            //   - double (float)
            //   - string
            //   - vector<long>/long[]
            //   - vector<double>/double[]
            
            // Note: this really does't need to (i.e. shouldn't) be a struct...
            // struct ParameterValue
            // {
                /// The type of this parameter's value
                value_type _val_type;
                
                /// union containing a pointer to the parameter's value
                // NOTE: A union is used instead of a void* to allow the
                //       individual variables to have meaningful names
                //       (avoiding casts everywhere) and so that the 'union'
                //       keyword (and braces) can simply be commented out 
                //       to convert the union into a simple set of variables
                //       (which is a valid, safe implementation that has the 
                //       drawback of using a fair bit more memory.
                //       It could be useful for debugging).
                union 
                {
                    bool                              *_val_bool           ;
                    long                              *_val_long           ;
                    double                            *_val_double         ;
                    std::string                       *_val_string         ;
                    
                    std::vector<long>                 *_val_1dvector_long  ;
                    std::vector<std::vector<long> >   *_val_2dvector_long  ;
                    std::vector<std::vector<std::vector<long> > >   *_val_3dvector_long  ;
                    std::vector<double>               *_val_1dvector_double;
                    std::vector<std::vector<double> > *_val_2dvector_double;
                    std::vector<std::vector<std::vector<double> > > *_val_3dvector_double;
                };
                
                /// union containing a pointer to the parameter's range
                // See the note on the union above.
                union 
                {
                    ConfigRange<long>   *_range_long  ;
                    ConfigRange<double> *_range_double;
                };
            // } param_value;
    };
}

#endif
