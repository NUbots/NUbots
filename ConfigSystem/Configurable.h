/*! @file Configurable.h
    @brief This file contains the definition of the Configurable interface for
           the Configuration module.
    
    @class Configurable
    @brief Defines an interface for classes that 
           can be configured via the Config system.

    Any object can access the config system, but if an object is to be notified
    of changes made to parameters in the config system that could affect its
    configuration, it should inherit from 'ConfigSystem::Configurable', and use
    'ConfigManager::AddConfigObject(Configurable*)' to add itself to the list
    of objects that the ConfigManager 'manages'.
    The ConfigManager updates the objects it manages on every iteration of the
    see-think thread (updating only those that need updating).
    
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
    
    // Initialise member variables upon construction.
    Configurable() : _configBasePath(""), _configModified(false) {}

    /*! @brief Completely configures this object.
     *
     *  Retrieves all relevant data from the Configuration System,
     *  then assigns them to this module's parameters.
     * 
     *  @param No input.
     *  @return No return value.
     */
    virtual void loadConfig()    = 0;
        
    /*! @brief Called by the config system to notify this object of
     *         changes to the config tree within this object's base path.
     *         (i.e. a more recent configuration is available for this object)
     *
     *  Note: The simplest implementation of this method is to just call loadConfig().
     */
    virtual void updateConfig() = 0;
    // virtual void updateConfig(
    //     const std::string& paramPath,
    //     const std::string& paramName
    //     ) = 0;
    
    //! Sets this Configurable's base path.
    void setConfigBasePath(std::string configBasePath);
    //! Returns this Configurable's base path.
    std::string getConfigBasePath();

    /*!
     * @brief Returns whether this Configurable's configuration has changed in
     *        the config system since this instance was last updated.
     *        (i.e. checks the _configModified flag)
     */
    bool isConfigOutdated();

    /*!
     * @brief Indicates to this Configurable that its config is recent and does
     *        not need to be updated.
     *        (i.e. sets the _configModified flag to false)
     */
    void setConfigAsRecent();

    /*!
     * @brief Indicates to this Configurable that its config is outdated and
     *        needs to be updated.
     *        (i.e. sets the _configModified flag to true)
     */
    void setConfigAsOutdated();

protected:

    //! The base path of this configurable object.
    //! Changes made on this path within the config system will cause this object's
    //! updateConfig method to be called.
    std::string _configBasePath;

    //! Indicates whether this Configurable's configuration has been modified
    //! in the config system since last being loaded.
    //! (i.e. Is set to true when this object's configuration becomes outdated.)
    bool _configModified;
};

#endif

