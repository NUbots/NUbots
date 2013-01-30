/*! @file Configurable.h
    @brief This file contains the definition of the Configurable interface for
           the Configuration module.
    
    @class Configurable
    @brief Defines an interface for classes that 
           can be configured via the Config system.
    
    @author Mitchell Metcalfe
    
  Copyright (c) 2012 Mitchell Metcalfe
    
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


#ifndef Configurable_H
#define Configurable_H

#include <string>

class Configurable
{
public: 
    /*! @brief Configures all of this class's parameters.
     *
     *  Retrieves relevant data from the Configuration System,
     *  then assigns them to this module's parameters.
     *
     *  @param No input.
     *  @return No return value.
     */
    virtual void loadConfig()    = 0;
    
    /*! @brief Called by the config system to send updated 
     *         parameters to this class.
     *
     *  The config system sends the path and value of the parameter that was
     *  updated.  This method uses the given path and name to update the
     *  correct parameter.
     *
     *  Note: The simplest, but least efficient, implementation of this method
     *        may be to just call loadConfig().
     *
     *  @param paramPath The path to the variable that changed.
     *  @param paramName The name of the variable that changed.
     *  @return No return value.
     */
    virtual void updateConfig() = 0;
    // virtual void updateConfig(
    //     const std::string& paramPath,
    //     const std::string& paramName
    //     ) = 0;


// protected:

    //! The base path of this configurable object.
    //! Changes made on this path within the config system will cause this object's
    //! updateConfig method to be called.
    std::string _configBasePath;
    #warning Configurable::_configBasePath and Configurable::_configModified are never initialised!!

    //! Indicates whether this Configurable's configuration has been modified
    //! in the config system since last being loaded.
    //! (i.e. Is set to true when this object's configuration becomes outdated.)
    bool _configModified;
};

#endif

