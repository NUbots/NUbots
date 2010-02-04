/*! @file ALNAO.h
 @brief Implementation of ALNAO class.
 
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

#include "NUbot.h"

#include <albroker.h>
#include <alproxy.h>
#include <almemoryproxy.h>
#include <almemoryfastaccess.h>
using namespace AL;

#include <iostream>
#include <fstream>
#include <string>
#include <time.h>
using namespace std;

ofstream debug;
ofstream errorlog;



class NUNAO : public ALModule
{
public:
    NUbot* m_nubot;
public:
    NUNAO(ALPtr<ALBroker> pBroker, const string& pName): ALModule(pBroker, pName)
    {
        debug << "NUNAO.cpp: NUNAO::NUNAO" << endl;
        m_nubot = new NUbot(0, NULL);
        getParentBroker()->getProxy("DCM")->getModule()->atPostProcess(NUbot::signalMotion);
        m_nubot->run();
    };

    virtual ~NUNAO() {delete m_nubot;};
    
    void onDCMPostProcess() {};
    
    void dataChanged(const string& pDataName, const ALValue& pValue, const string& pMessage) {};
    
    bool innerTest() {return true;};
}; 

extern "C" int _createModule(ALPtr<ALBroker> pBroker)
{
    debug.open("/var/log/debug.log");
    debug << "NUbot Debug Log" << endl;
    debug << "NUNAO.cpp: _createModule" << endl;
    errorlog.open("/var/log/error.log");
    errorlog << "NUbot Error Log" << endl;
    ALModule::createModule<NUNAO>(pBroker, "NUNAO");
    return 0;
}

extern "C" int _closeModule()
{
    return 0;
}





