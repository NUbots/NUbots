/*! @file NAOPlatform.h
 @brief Implementation of NAOPlatform class.
 
 @author Jason Kulk
 
 Copyright (c) 2009 Jason Kulk
 
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

#include <albroker.h>
#include <alproxy.h>
#include <almemoryproxy.h>
#include <almemoryfastaccess.h>

#include <iostream>
#include <fstream>

#include "NAOPlatform.h"

class NAOPlatform : public AL::ALModule
{
public:
    NAOPlatform(AL::ALPtr<AL::ALBroker> pBroker, const std::string& pName): ALModule(pBroker, pName)
    {
        setModuleDescription("A module that provides basic access to the NAOs sensors and actuators.");
        std::ofstream thelog;
        thelog.open("/var/log/jason.log");
        thelog << "NAOPlatform.cpp: NAOPlatform::NAOPlatform" << endl;
    }

    virtual ~NAOPlatform()
    {
    }
    
    void dataChanged(const std::string& pDataName, const ALValue& pValue, const std::string& pMessage)
    {
    }
    bool innerTest() 
    {
        return true;
    }
}; 

extern "C" int _createModule(AL::ALPtr<AL::ALBroker> pBroker)
{
    AL::ALModule::createModule<NAO>(pBroker, "NAOPlatform");
    cout << "NAO.cpp: _createModule" << endl;
    return 0;
}

extern "C" int _closeModule()
{
    return 0;
}





