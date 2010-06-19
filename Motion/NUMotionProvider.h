/*! @file NUMotionProvider.h
    @brief Declaration of an abstract motion provider class
 
    @class NUMotionProvider
    @brief An abstract module for motion provider to inherit from.
 
    @author Jason Kulk
 
  Copyright (c) 2010 Jason Kulk
 
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

#ifndef NUMOTION_PROVIDER_H
#define NUMOTION_PROVIDER_H

class NUSensorsData;
class NUActionatorsData;

#include <string>

class NUMotionProvider
{
public:
    NUMotionProvider(std::string name, NUSensorsData* data, NUActionatorsData* actions)
    {
        m_name = name;
        m_data = data;
        m_actions = actions;
    }
    
    /*! @brief Returns the name of this motion provider */
    std::string getName() {
        return m_name;}
    
    virtual void process(NUSensorsData* data, NUActionatorsData* actions) = 0;
    
    virtual void stop() = 0;
    virtual void stopHead() = 0;
    virtual void stopArms() = 0;
    virtual void stopLegs() = 0;
    virtual void kill() = 0;
    
    virtual bool isActive() = 0;
    virtual bool isUsingHead() = 0;
    virtual bool isUsingArms() = 0;
    virtual bool isUsingLegs() = 0;
    
    virtual bool isReady() {return true;}
    virtual bool requiresHead() = 0;
    virtual bool requiresArms() = 0;
    virtual bool requiresLegs() = 0;
protected:
    std::string m_name;
    NUSensorsData* m_data;
    NUActionatorsData* m_actions;
};

#endif

