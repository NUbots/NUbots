/*! @file NUNAO.h
    @brief Declaration of NUNAO class.
 
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
#ifndef NUNAO_H
#define NUNAO_H

#include <alcommon/albroker.h>
#include <alcommon/alproxy.h>
using namespace AL;

#include <iostream>
#include <fstream>
#include <string>
#include <time.h>
using namespace std;

class NUbot;

class NUNAO : public ALModule
{
public:
    static ALPtr<ALBroker> m_broker;
    NUbot* m_nubot;
    
public:
    NUNAO(ALPtr<ALBroker> pBroker, const string& pName);
    virtual ~NUNAO();
    
    void dataChanged(const string& pDataName, const ALValue& pValue, const string& pMessage) {};
    
    bool innerTest() {return true;};
}; 

#endif



